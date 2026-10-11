

#include <unistd.h>
#include <limits.h>
#include <thread>
#include <fstream>

#include <plog/Log.h>

#include "arm_common/Utils.hpp"
#include "arm_common/RosTopicManager.hpp"

#include "arm_controller/Manipulator.h"
#include "arm_controller/JointPositionWaypoint.h"
#include "arm_controller/TaskPositionWaypoint.h"

#include "ptera_msgs/msg/manipulator_state.hpp"
#include "arm_msgs/msg/hardware_state.hpp"

Manipulator::Manipulator(ConfigManager::Config aConfig)
    : mConfig(aConfig), mWaypointExecutor(std::make_unique<WaypointExecutor>(mConfig)),
      mKinematicsHandler(std::make_shared<KinematicsHandler>()),
      mExecutorReady(false), mEnabled(false), mRunning(true), mIsArrived(false)
{
    mHardware = std::make_shared<HardwareInterface>();
    mHardware->init();

    if(mConfig.mobileBase)
    {
        mBase = std::make_shared<MobileBaseInterface>(*mConfig.mobileBase, mConfig.controlRateHz);
        mBase->init();
    }

    if(!mKinematicsHandler->init(mConfig))
    {
        throw std::runtime_error("Failed to initialize kinematics");
    }

    int numCtrlJoints = getNrCtrlJoints();
    mWaypointExecutor->init(numCtrlJoints, mConfig.ctrlOffset, *mKinematicsHandler.get());

    KDL::JntArray firstWp(numCtrlJoints);
    KDL::JntArray firstTol(numCtrlJoints);

    for(int i = 0; i < numCtrlJoints; i++)
    {
        firstWp(i) = mConfig.ctrlInitialPositions[i];
        firstTol(i) = 0.01;
    }

    mInitialGoalWp = std::make_shared<JointPositionWaypoint>(firstWp, firstTol);

    if(mConfig.gripper)
    {
        KDL::JntArray firstGripperWp(mConfig.gripper->jointNames.size());
        for(size_t i = 0; i < mConfig.gripper->jointNames.size(); i++)
        {
            firstGripperWp(i) = mConfig.gripper->initialPositions[i];
        }
        mHardware->sendGripperCommand(firstGripperWp);
    }

    RosTopicManager::getInstance()->createPublisher<ptera_msgs::msg::ManipulatorState>("arm/state");
    RosTopicManager::getInstance()->createPublisher<arm_msgs::msg::HardwareState>("arm/hardware/state");
    mHardwareStateThread = std::thread(&Manipulator::publishHardwareStateLoop, this);
}

Manipulator::~Manipulator()
{
    mRunning = false;
    if(mHardwareStateThread.joinable())
    {
        mHardwareStateThread.join();
    }

    if(mControlThread.joinable())
    {
        mControlThread.join();
    }

    if(mArrivalThread.joinable())
    {
        mArrivalThread.join();
    }

    if(mPublishThread.joinable())
    {
        mPublishThread.join();
    }
}

bool Manipulator::sendToPose(Manipulator::POSE aPose)
{
    if(mArmPoseMap.find(aPose) != mArmPoseMap.end())
    {
        setJointPositionGoal(mArmPoseMap.at(aPose));
        return true;
    }

    return false;
}

void Manipulator::setJointPositionGoal(const KDL::JntArray& aNewJntPos)
{
    std::lock_guard<std::mutex> lock(mGoalJntPosMutex);
    mGoalJntPos = aNewJntPos;
}

void Manipulator::setEnabledState(bool anEnabledFlag)
{
    std::lock_guard<std::mutex> lock(mEnabledMutex);
    mEnabled = anEnabledFlag;
}

bool Manipulator::isEnabled()
{
    std::lock_guard<std::mutex> lock(mEnabledMutex);
    return mEnabled;
}

bool Manipulator::setGoalWaypoint(std::shared_ptr<IWaypoint> aWp, bool aResetState)
{
    std::lock_guard<std::mutex> lock(mGoalMutex);

    if(aResetState)
    {
        mGoalWaypoint = aWp;
    }
    else if(aWp != mGoalWaypoint)
    {
        // stale refresh — a newer waypoint was set since aWp was read
        return false;
    }

    auto wpType = aWp->type();

    // full kinematic joint space [base..., arm...], no gripper, task
    // waypoints seed IK with this so the measured base pose is respected
    const KDL::JntArray q_cur = getFullJointPositions();
    const KDL::JntArray qdot_cur = getFullJointVelocities();

    if(q_cur.rows() == 0 || qdot_cur.rows() == 0)
    {
        // retried every control tick until state arrives — warn once a second
        static std::chrono::steady_clock::time_point lastWarn;
        const auto now = std::chrono::steady_clock::now();
        if(now - lastWarn > std::chrono::seconds(1))
        {
            LOGW << "No joint state yet (waiting on arm telemetry"
                 << (mBase ? " and robot/state" : "") << "); cannot set goal";
            lastWarn = now;
        }
        if(aResetState)
        {
            mExecutorReady = false;
        }
        return false;
    }

    // Task position refreshes seed IK from the last solution so the target
    // only moves with the (measured) uncontrolled base, not with the arm's
    // tracking error. Controlled joints come from the previous solution.
    KDL::JntArray q_seed = q_cur;
    if(!aResetState && wpType == IWaypoint::Type::TaskPosition && mLastTaskSolution.rows() == q_cur.rows())
    {
        q_seed = mLastTaskSolution;
        for(int i = 0; i < mConfig.ctrlOffset; i++)
        {
            q_seed(i) = q_cur(i);
        }
    }

    // full-size for task waypoints, controlled-size for joint waypoints
    KDL::JntArray q_goal;
    if (!mGoalWaypoint->toJointGoal(q_seed, qdot_cur, *mKinematicsHandler, q_goal))
    {
        LOGW << "Could not produce joint-space goal from waypoint";
        return false;
    }

    if(wpType == IWaypoint::Type::TaskPosition)
    {
        mLastTaskSolution = q_goal;
    }

    ruckig::ControlInterface controlType = ruckig::ControlInterface::Position;

    switch (wpType)
    {
        case IWaypoint::Type::TaskPosition:
            q_goal = toCtrl(q_goal);
            [[fallthrough]];
        case IWaypoint::Type::JointPosition:
            controlType = ruckig::ControlInterface::Position;
            break;

        case IWaypoint::Type::TaskVelocity:
            q_goal = toCtrl(q_goal);
            [[fallthrough]];
        case IWaypoint::Type::JointVelocity:
            controlType = ruckig::ControlInterface::Velocity;
            break;
        default:
            break;
    }

    // Refresh only retargets — Ruckig keeps its own state so the profile
    // continues rather than restarting from measured state each tick
    if(!aResetState)
    {
        return mExecutorReady && mWaypointExecutor->setGoalState(q_goal, controlType);
    }

    LOGD << "Set ControlInterface to "
         << (controlType == ruckig::ControlInterface::Position ? "Position" : "Velocity");

    const KDL::JntArray q_cur_ctrl = toCtrl(q_cur);
    for(int i = 0; i < getNrCtrlJoints(); i++)
    {
        LOGD << "Current Joint " << i << ": " << q_cur_ctrl(i) << ", Goal Joint " << i << ": " << q_goal(i);
    }

    // q_goal is joint pos or joint vel based on waypoint type
    mExecutorReady = mWaypointExecutor->initializeExecutor(q_goal, q_cur_ctrl, toCtrl(qdot_cur), controlType);
    return mExecutorReady;
}

KDL::JntArray Manipulator::toCtrl(const KDL::JntArray& aFull) const
{
    const int numCtrlJoints = getNrCtrlJoints();
    KDL::JntArray ctrl(numCtrlJoints);

    for(int i = 0; i < numCtrlJoints; i++)
    {
        ctrl(i) = aFull(mConfig.ctrlOffset + i);
    }

    return ctrl;
}

KDL::JntArray Manipulator::composeFull(const KDL::JntArray& aBase, const KDL::JntArray& aArm) const
{
    const int numBase = mBase ? mBase->getNrJoints() : 0;
    const int numArm = (int)mConfig.manipulator.jointNames.size();

    if((int)aBase.rows() < numBase || (int)aArm.rows() < numArm)
    {
        return KDL::JntArray();
    }

    KDL::JntArray full(numBase + numArm);
    for(int i = 0; i < numBase; i++)
    {
        full(i) = aBase(i);
    }
    for(int i = 0; i < numArm; i++)
    {
        full(numBase + i) = aArm(i);
    }

    return full;
}

KDL::JntArray Manipulator::getFullJointPositions()
{
    if(mBase && !mBase->hasState())
    {
        return KDL::JntArray();
    }

    return composeFull(mBase ? mBase->getJointPositions() : KDL::JntArray(), mHardware->getJointPositions());
}

KDL::JntArray Manipulator::getFullJointVelocities()
{
    if(mBase && !mBase->hasState())
    {
        return KDL::JntArray();
    }

    return composeFull(mBase ? mBase->getJointVelocities() : KDL::JntArray(), mHardware->getJointVelocities());
}

void Manipulator::sendCommand(const KDL::JntArray& aCtrlCmd)
{
    // Controllable base: the controlled joints are [base..., arm...]
    const int numBaseCmd = (mBase && mConfig.ctrlOffset == 0) ? mBase->getNrJoints() : 0;
    if(numBaseCmd == 0)
    {
        mHardware->sendJointCommand(aCtrlCmd);
        return;
    }

    if((int)aCtrlCmd.rows() < numBaseCmd)
    {
        LOGW << "Command of size " << aCtrlCmd.rows() << " too small for " << numBaseCmd << " base joints";
        return;
    }

    KDL::JntArray baseCmd(numBaseCmd);
    KDL::JntArray armCmd(aCtrlCmd.rows() - numBaseCmd);
    for(int i = 0; i < numBaseCmd; i++)
    {
        baseCmd(i) = aCtrlCmd(i);
    }
    for(unsigned int i = 0; i < armCmd.rows(); i++)
    {
        armCmd(i) = aCtrlCmd(numBaseCmd + i);
    }

    mBase->sendCommand(baseCmd);
    mHardware->sendJointCommand(armCmd);
}

void Manipulator::setGripperGoal(const KDL::JntArray& aGripperGoal)
{
    mHardware->sendGripperCommand(aGripperGoal);
}

std::shared_ptr<IWaypoint> Manipulator::getGoalWaypoint()
{
    std::lock_guard<std::mutex> lock(mGoalMutex);
    return mGoalWaypoint;
}

bool Manipulator::checkArrival()
{
    // full kinematic joint space, no gripper
    const KDL::JntArray currentJointPos = getFullJointPositions();
    const KDL::JntArray currentJointVel = getFullJointVelocities();

    if(currentJointPos.rows() == 0 || currentJointVel.rows() == 0)
    {
        return false;
    }

    const KDL::JntArray& q_full = currentJointPos;

    // current state — joint waypoints compare against controlled joints only
    IWaypoint::ControlInputs s;
    s.q = toCtrl(currentJointPos);
    s.qdot = toCtrl(currentJointVel);

    // Only need to compute FK for task position waypoint
    auto goalWp = getGoalWaypoint();
    if (goalWp->type() == IWaypoint::Type::TaskPosition)
    {
        if (!mKinematicsHandler->solveFk(q_full, s.T))
        {
            LOGW << "FK failed; not arrived";
            return false;
        }
    }

    return goalWp->arrived(s);
}

bool Manipulator::isArrived()
{
    std::lock_guard<std::mutex> lock(mArrivalMutex);
    return mIsArrived;
}

void Manipulator::setArrivalState(bool aFlag)
{
    std::lock_guard<std::mutex> lock(mArrivalMutex);
    mIsArrived = aFlag;
}

void Manipulator::printArrivedState()
{
    KDL::JntArray state = getFullJointPositions();
    if(state.rows() == 0)
    {
        return;
    }

    switch (mGoalWaypoint->type())
    {
    case IWaypoint::Type::JointPosition:
    {
        std::shared_ptr<JointPositionWaypoint> jointPosWp = std::dynamic_pointer_cast<JointPositionWaypoint>(mGoalWaypoint);
        KDL::JntArray goal = jointPosWp->goal();
        state = toCtrl(state);

        std::stringstream ss;
        ss << std::fixed << std::setprecision(3);
        ss << "Error: " << "\n";
        for(int i = 0; i < getNrCtrlJoints(); i++)
        {
            ss << "J" << i << " (deg): " << (180.0/M_PI) * (state(i) - goal(i)) << "\n";
        }

        LOGD << ss.str();
        break;
    }
    case IWaypoint::Type::TaskPosition:
    {
        std::shared_ptr<TaskPositionWaypoint> taskPosWp = std::dynamic_pointer_cast<TaskPositionWaypoint>(mGoalWaypoint);
        KDL::Frame goal = taskPosWp->goal();

        // current state
        IWaypoint::ControlInputs s;
        s.q = state;

        if (!mKinematicsHandler->solveFk(s.q, s.T))
        {
            LOGW << "FK failed; not arrived";
        }

        // Linear error: err = goal * current^{-1}
        const KDL::Frame err = goal * s.T.Inverse();

        std::stringstream ss;
        ss << std::fixed << std::setprecision(3);
        ss << "Task Error: " << "\n";
        std::vector<std::string> axes = {"x: ", "y: ", "z: "};

        for(int i = 0; i < 3; i++)
        {
            ss << axes[i] << " (m): " << err.p.data[i] << "\n";
        }

        LOGD << ss.str();
    }
    default:
        break;
    }
}

void Manipulator::startControl()
{
    setGoalWaypoint(mInitialGoalWp);

    if (mControlThread.joinable())
    {
        mControlThread.join();
    }

    if(mArrivalThread.joinable())
    {
        mArrivalThread.join();
    }

    if(mPublishThread.joinable())
    {
        mPublishThread.join();
    }

    mControlThread = std::thread(&Manipulator::controlLoop, this);
    mArrivalThread = std::thread(&Manipulator::arrivalLoop, this);
    mPublishThread = std::thread(&Manipulator::publishStateLoop, this);
}

void Manipulator::controlLoop()
{
    LOGD << "Starting manipulator control loop!";
    RateController controlRate(mConfig.controlRateHz);

    while(isEnabled())
    {
        controlRate.start();

        auto goalWp = getGoalWaypoint();
        bool ready;
        {
            std::lock_guard<std::mutex> lock(mGoalMutex);
            ready = mExecutorReady;
        }

        // The goal couldn't be initialized (e.g. no base/arm state yet) — retry
        // each tick and hold off commanding until it succeeds
        if(!ready)
        {
            if(!goalWp || !setGoalWaypoint(goalWp))
            {
                controlRate.block();
                continue;
            }
        }
        // Task waypoints are re-solved against the latest measured state so
        // the goal tracks a moving base. On IK failure the previous target holds.
        else if(goalWp && (goalWp->type() == IWaypoint::Type::TaskPosition ||
                           goalWp->type() == IWaypoint::Type::TaskVelocity))
        {
            setGoalWaypoint(goalWp, false);
        }

        KDL::JntArray wp;
        {
            std::lock_guard<std::mutex> lock(mGoalMutex);
            wp = mWaypointExecutor->getNextWaypoint();
        }
        sendCommand(wp);

        controlRate.block();
    }
}

void Manipulator::arrivalLoop()
{
    RateController arrivalCheckRate(mConfig.controlRateHz);
    std::chrono::time_point<std::chrono::steady_clock> arrivalTime;
    std::chrono::duration<double> arrivalThreshold = std::chrono::seconds(1);
    bool arrivalAcknowledged = false;

    while(isEnabled())
    {
        arrivalCheckRate.start();

        if(checkArrival())
        {
            if(arrivalTime.time_since_epoch().count() == 0)
            {
                arrivalTime = std::chrono::steady_clock::now();
            }
            else
            {
                auto now = std::chrono::steady_clock::now();
                if(now - arrivalTime >= arrivalThreshold && !arrivalAcknowledged)
                {
                    LOGD << "Waypoint arrived!";
                    setArrivalState(true);
                    printArrivedState();
                    arrivalAcknowledged = true;
                }
            }
        }
        else
        {
            // Manipulator moved away from goal
            if (arrivalAcknowledged)
            {
                LOGW << "Manipulator deviated from goal!";
                setArrivalState(false);
                arrivalAcknowledged = false;
            }

            arrivalTime = std::chrono::time_point<std::chrono::steady_clock>();
        }

        arrivalCheckRate.block();
    }
}

void Manipulator::publishStateLoop()
{
    RateController rate(10);

    while(isEnabled())
    {
        rate.start();

        // [base..., arm...] layout
        KDL::JntArray pos = getFullJointPositions();
        KDL::JntArray vel = getFullJointVelocities();
        if(pos.rows() == 0 || vel.rows() == 0)
        {
            rate.block();
            continue;
        }

        ptera_msgs::msg::ManipulatorState state;
        state.state = ptera_msgs::msg::ManipulatorState::MOVING;

        bool arrived = isArrived();
        if(arrived)
        {
            state.state = ptera_msgs::msg::ManipulatorState::ARRIVED;
        }

        std::vector<double> position(pos.rows());
        std::vector<double> velocity(vel.rows());

        for(int i = 0; i < pos.rows(); i++)
        {
            position[i] = pos(i);
            velocity[i] = vel(i);
        }

        state.set__position(position);
        state.set__velocity(velocity);

        RosTopicManager::getInstance()->publishMessage("arm/state", state);
        rate.block();
    }
}

void Manipulator::publishHardwareStateLoop()
{
    RateController rate(mConfig.controlRateHz);

    const int numJoints = mKinematicsHandler->getNrJoints();
    const std::string rootFrame = mKinematicsHandler->getRootFrame();

    arm_msgs::msg::HardwareState state;
    state.header.frame_id = rootFrame;
    state.name = mConfig.manipName;

    while(mRunning)
    {
        rate.start();

        // full kinematic joint space [base..., arm...], no gripper
        const KDL::JntArray pos = getFullJointPositions();
        const KDL::JntArray vel = getFullJointVelocities();

        // no (or incomplete) telemetry yet
        if((int)pos.rows() < numJoints || (int)vel.rows() < numJoints)
        {
            rate.block();
            continue;
        }

        KDL::JntArray q(numJoints);
        KDL::JntArray qdot(numJoints);
        for(int i = 0; i < numJoints; i++)
        {
            q(i) = pos(i);
            qdot(i) = vel(i);
        }

        KDL::Frame T;
        KDL::Twist twist;
        if(!mKinematicsHandler->solveFk(q, T) || !mKinematicsHandler->solveTaskVelocity(q, qdot, twist))
        {
            rate.block();
            continue;
        }

        state.header.stamp = RosTopicManager::getInstance()->now();
        state.position.assign(q.data.data(), q.data.data() + numJoints);
        state.velocity.assign(qdot.data.data(), qdot.data.data() + numJoints);

        state.task_pose.position.x = T.p.x();
        state.task_pose.position.y = T.p.y();
        state.task_pose.position.z = T.p.z();
        T.M.GetQuaternion(state.task_pose.orientation.x, state.task_pose.orientation.y,
                          state.task_pose.orientation.z, state.task_pose.orientation.w);

        state.task_velocity.linear.x  = twist.vel.x();
        state.task_velocity.linear.y  = twist.vel.y();
        state.task_velocity.linear.z  = twist.vel.z();
        state.task_velocity.angular.x = twist.rot.x();
        state.task_velocity.angular.y = twist.rot.y();
        state.task_velocity.angular.z = twist.rot.z();

        RosTopicManager::getInstance()->publishMessage("arm/hardware/state", state);
        rate.block();
    }
}
