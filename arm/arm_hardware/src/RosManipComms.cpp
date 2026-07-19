
#include "std_msgs/msg/float64_multi_array.hpp"
#include "plog/Log.h"

#include "arm_common/RosTopicManager.hpp"
#include "arm_hardware/RosManipComms.h"

RosManipComms::RosManipComms(const ConfigManager::Config& aConfig) :
        mPrevJointPos(0), mPrevTime(std::chrono::steady_clock::now()),
        mPrevVehiclePos(3), mPrevVehicleTime(std::chrono::steady_clock::now()),
        mFirstRcvd(false)
{
    mNumBaseJoints    = aConfig.mobileBase ? static_cast<int>(aConfig.mobileBase->jointNames.size()) : 0;
    mNumArmJoints     = static_cast<int>(aConfig.manipulator.jointNames.size());
    mNumGripperJoints = aConfig.gripper ? static_cast<int>(aConfig.gripper->jointNames.size()) : 0;

    const int totalJoints = mNumBaseJoints + mNumArmJoints + mNumGripperJoints;

    mPrevJointPos.resize(mNumArmJoints + mNumGripperJoints);
    mJointPositions.resize(totalJoints);
    mJointVelocities.resize(totalJoints);

    mLastArmCmd.assign(mNumArmJoints, 0.0);
    mLastGripperCmd.assign(mNumGripperJoints, 0.0);

    for(int i = 0; i < mPrevJointPos.rows(); i++)
    {
        mPrevJointPos(i) = 0.0;
    }

    for(int i = 0; i < mJointPositions.rows(); i++)
    {
        mJointPositions(i) = 0.0;
        mJointVelocities(i) = 0.0;
    }

    for(int i = 0; i < mPrevVehiclePos.rows(); i++)
    {
        mPrevVehiclePos(i) = 0.0;
    }
}

RosManipComms::~RosManipComms()
{

}

bool RosManipComms::init()
{
    RosTopicManager::getInstance()->
        createSubscriber<std_msgs::msg::Float64MultiArray>("gazebo/joint_positions",
                                                            std::bind(&RosManipComms::armStateCallback,
                                                                    this,
                                                                    std::placeholders::_1));

    RosTopicManager::getInstance()->
        createPublisher<std_msgs::msg::Float64MultiArray>("gazebo/joint_commands");

    if(mNumBaseJoints > 0)
    {
        RosTopicManager::getInstance()->
            createSubscriber<ptera_msgs::msg::RobotState>("/robot/state",
                                                        std::bind(&RosManipComms::vehicleStateCallback,
                                                                    this,
                                                                    std::placeholders::_1));
    }

    return true;
}

KDL::JntArray RosManipComms::getJointPositions()
{
    std::lock_guard<std::mutex> lock(mJointPosMutex);
    KDL::JntArray armPos(mNumBaseJoints + mNumArmJoints);
    for(int i = 0; i < armPos.rows(); i++)
    {
        armPos(i) = mJointPositions(i);
    }
    return armPos;
}

KDL::JntArray RosManipComms::getJointVelocities()
{
    std::lock_guard<std::mutex> lock(mJointVelMutex);
    KDL::JntArray armVel(mNumBaseJoints + mNumArmJoints);
    for(int i = 0; i < armVel.rows(); i++)
    {
        armVel(i) = mJointVelocities(i);
    }
    return armVel;
}

KDL::JntArray RosManipComms::getGripperPositions()
{
    std::lock_guard<std::mutex> lock(mJointPosMutex);
    KDL::JntArray gripperPos(mNumGripperJoints);
    for(int i = 0; i < mNumGripperJoints; i++)
    {
        gripperPos(i) = mJointPositions(mNumBaseJoints + mNumArmJoints + i);
    }
    return gripperPos;
}

KDL::JntArray RosManipComms::getGripperVelocities()
{
    std::lock_guard<std::mutex> lock(mJointVelMutex);
    KDL::JntArray gripperVel(mNumGripperJoints);
    for(int i = 0; i < mNumGripperJoints; i++)
    {
        gripperVel(i) = mJointVelocities(mNumBaseJoints + mNumArmJoints + i);
    }
    return gripperVel;
}

void RosManipComms::sendJointCommand(const KDL::JntArray &aCmd)
{
    // Gazebo only simulates the arm — strip the leading mobile-base DOFs (if
    // any) before caching.
    const int numArmJoints = static_cast<int>(aCmd.rows()) - mNumBaseJoints;

    std::lock_guard<std::mutex> lock(mCmdMutex);
    for(int i = 0; i < numArmJoints; i++)
    {
        mLastArmCmd[i] = aCmd(mNumBaseJoints + i);
    }
    publishCombinedCommand();
}

void RosManipComms::sendGripperCommand(const KDL::JntArray &aCmd)
{
    std::lock_guard<std::mutex> lock(mCmdMutex);
    for(int i = 0; i < static_cast<int>(aCmd.rows()) && i < mNumGripperJoints; i++)
    {
        mLastGripperCmd[i] = aCmd(i);
    }
    publishCombinedCommand();
}

// Arm and gripper share one combined Float64MultiArray on the wire
// (UR/joint_commands) — publish the merge of whichever last changed.
// Caller must hold mCmdMutex.
void RosManipComms::publishCombinedCommand()
{
    std::vector<double> posCmd;
    posCmd.reserve(mLastArmCmd.size() + mLastGripperCmd.size());
    posCmd.insert(posCmd.end(), mLastArmCmd.begin(), mLastArmCmd.end());
    posCmd.insert(posCmd.end(), mLastGripperCmd.begin(), mLastGripperCmd.end());

    std_msgs::msg::Float64MultiArray cmd;
    cmd.set__data(posCmd);
    RosTopicManager::getInstance()->publishMessage<std_msgs::msg::Float64MultiArray>("gazebo/joint_commands", cmd);
}

void RosManipComms::armStateCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    auto now = std::chrono::steady_clock::now();
    const std::size_t n = msg->data.size();

    {
        std::lock_guard<std::mutex> lock(mJointPosMutex);
        for(std::size_t i = 0; i < n; i++)
        {
            mJointPositions(mNumBaseJoints + i) = msg->data[i];
        }
    }

    {
        std::lock_guard<std::mutex> lock(mJointVelMutex);
        for(std::size_t i = 0; i < n; i++)
        {
            mJointVelocities(mNumBaseJoints + i) =
                (msg->data[i] - mPrevJointPos(i)) / (now.time_since_epoch().count() - mPrevTime.time_since_epoch().count());
        }
    }

    for(std::size_t i = 0; i < n; i++)
    {
        mPrevJointPos(i) = msg->data[i];
    }
    mPrevTime = now;
}

void RosManipComms::vehicleStateCallback(ptera_msgs::msg::RobotState::SharedPtr msg)
{
    const double x = msg->position.x;
    const double y = msg->position.y;
    const double yaw = msg->euler.yaw;

    auto now = std::chrono::steady_clock::now();

    double vx = 0.0, vy = 0.0, vyaw = 0.0;

    if(mFirstRcvd)
    {
        const double dt = now.time_since_epoch().count() - mPrevVehicleTime.time_since_epoch().count();
        vx   = (x   - mPrevVehiclePos(0)) / dt;
        vy   = (y   - mPrevVehiclePos(1)) / dt;
        vyaw = (yaw - mPrevVehiclePos(2)) / dt;
    }
    else
    {
        mFirstRcvd = true;
    }

    {
        std::lock_guard<std::mutex> lock(mJointPosMutex);
        mJointPositions(0) = x;
        mJointPositions(1) = y;
        mJointPositions(2) = yaw;
    }

    {
        std::lock_guard<std::mutex> lock(mJointVelMutex);
        mJointVelocities(0) = vx;
        mJointVelocities(1) = vy;
        mJointVelocities(2) = vyaw;
    }

    mPrevVehiclePos(0) = x;
    mPrevVehiclePos(1) = y;
    mPrevVehiclePos(2) = yaw;
    mPrevVehicleTime = now;
}
