#include "plog/Log.h"

#include "arm_common/RosTopicManager.hpp"

#include "ptera_msgs/msg/vehicle_waypoint.hpp"

#include "arm_controller/MobileBaseInterface.h"

static constexpr const char* ROBOT_STATE_TOPIC = "robot/state";

MobileBaseInterface::MobileBaseInterface(const ConfigManager::MobileBaseConfig& aConfig, int aControlRateHz) :
    mJointAxes(aConfig.jointAxes), mCommandTopic(aConfig.commandTopic),
    mControllable(aConfig.controllable), mControlPeriod(1.0 / aControlRateHz),
    mHasState(false)
{

}

MobileBaseInterface::~MobileBaseInterface()
{

}

bool MobileBaseInterface::init()
{
    RosTopicManager::getInstance()->createSubscriber<ptera_msgs::msg::RobotState>(ROBOT_STATE_TOPIC,
                                                                 std::bind(&MobileBaseInterface::stateCallback,
                                                                           this,
                                                                           std::placeholders::_1));

    if(mControllable)
    {
        RosTopicManager::getInstance()->createPublisher<ptera_msgs::msg::VehicleWaypoint>(mCommandTopic);
    }

    return true;
}

bool MobileBaseInterface::hasState()
{
    std::lock_guard<std::mutex> lock(mStateMutex);
    return mHasState;
}

BaseState MobileBaseInterface::getState()
{
    std::lock_guard<std::mutex> lock(mStateMutex);
    return mLatestState;
}

KDL::JntArray MobileBaseInterface::getJointPositions()
{
    return getState().jointPositions(mJointAxes);
}

KDL::JntArray MobileBaseInterface::getJointVelocities()
{
    return getState().jointVelocities(mJointAxes);
}

void MobileBaseInterface::sendCommand(const KDL::JntArray& aCmd)
{
    if(!mControllable)
    {
        return;
    }

    if((int)aCmd.rows() != getNrJoints())
    {
        LOGW << "Base command size mismatch: got " << aCmd.rows() << ", expected " << getNrJoints();
        return;
    }

    BaseState target = getState();
    target.setJointPositions(mJointAxes, aCmd);

    ptera_msgs::msg::VehicleWaypoint wp;
    wp.position.x = target.p.x();
    wp.position.y = target.p.y();
    wp.position.z = target.p.z();
    wp.euler.roll  = target.roll;
    wp.euler.pitch = target.pitch;
    wp.euler.yaw   = target.yaw;
    KDL::Rotation::RPY(target.roll, target.pitch, target.yaw)
        .GetQuaternion(wp.quat.x, wp.quat.y, wp.quat.z, wp.quat.w);

    // TODO: make configurable
    wp.position_tolerance.x = 0.01;
    wp.position_tolerance.y = 0.01;
    wp.position_tolerance.z = 0.01;
    wp.euler_tolerance.roll  = 0.01;
    wp.euler_tolerance.pitch = 0.01;
    wp.euler_tolerance.yaw   = 0.01;

    // Streamed once per control tick, so each segment spans one period
    wp.execution_duration = mControlPeriod;

    RosTopicManager::getInstance()->publishMessage<ptera_msgs::msg::VehicleWaypoint>(mCommandTopic, wp);
}

void MobileBaseInterface::stateCallback(ptera_msgs::msg::RobotState::SharedPtr aState)
{
    BaseState state;
    state.p = KDL::Vector(aState->position.x, aState->position.y, aState->position.z);
    state.roll  = aState->euler.roll;
    state.pitch = aState->euler.pitch;
    state.yaw   = aState->euler.yaw;

    // robot/state velocities are expressed in the world frame
    state.twist.vel = KDL::Vector(aState->velocity.x, aState->velocity.y, aState->velocity.z);
    state.twist.rot = KDL::Vector(aState->angular_velocity.x, aState->angular_velocity.y, aState->angular_velocity.z);

    std::lock_guard<std::mutex> lock(mStateMutex);
    mLatestState = state;
    mHasState = true;
}
