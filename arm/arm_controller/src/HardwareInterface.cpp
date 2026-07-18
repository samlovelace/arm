#include "plog/Log.h"

#include "arm_common/RosTopicManager.hpp"

#include "arm_msgs/msg/hardware_goal.hpp"

#include "arm_controller/HardwareInterface.h"

HardwareInterface::HardwareInterface()
{

}

HardwareInterface::~HardwareInterface()
{

}

bool HardwareInterface::init()
{
    // Create publishers and subscribers to interface with arm_hardware
    RosTopicManager::getInstance()->createSubscriber<arm_msgs::msg::HardwareState>("arm/hardware/state",
                                                                 std::bind(&HardwareInterface::stateCallback,
                                                                           this,
                                                                           std::placeholders::_1));

    RosTopicManager::getInstance()->createPublisher<arm_msgs::msg::HardwareGoal>("arm/hardware/goal");

    // Parallel gripper channel — unconditionally created; harmless if no
    // gripper is configured (arm_hardware simply never publishes state on it).
    RosTopicManager::getInstance()->createSubscriber<arm_msgs::msg::HardwareState>("arm/hardware/gripper_state",
                                                                 std::bind(&HardwareInterface::gripperStateCallback,
                                                                           this,
                                                                           std::placeholders::_1));

    RosTopicManager::getInstance()->createPublisher<arm_msgs::msg::HardwareGoal>("arm/hardware/gripper_goal");
}

KDL::JntArray HardwareInterface::getJointPositions()
{
    std::lock_guard<std::mutex> lock(mStateMutex);
    return mLatestPosition;
}

KDL::JntArray HardwareInterface::getJointVelocities()
{
    std::lock_guard<std::mutex> lock(mStateMutex);
    return mLatestVelocity;
}

void HardwareInterface::sendJointCommand(const KDL::JntArray &aCmd)
{
    std::vector<double> goal(aCmd.rows());
    for(int i = 0; i < goal.size(); i++)
    {
        //LOGV << "Sending joint command: aCmd(" << i << "): " << aCmd(i);
        goal[i] = aCmd(i);
    }

    arm_msgs::msg::HardwareGoal goalMsg;
    goalMsg.set__position(goal);

    RosTopicManager::getInstance()->publishMessage("arm/hardware/goal", goalMsg);
}

void HardwareInterface::stateCallback(arm_msgs::msg::HardwareState::SharedPtr aState)
{
    KDL::JntArray pos(aState->position.size());
    KDL::JntArray vel(aState->velocity.size());

    for(int i = 0; i < pos.rows(); i++)
    {
        pos(i) = aState->position[i];
        vel(i) = aState->velocity[i];
    }

    std::lock_guard<std::mutex> lock(mStateMutex);
    mLatestPosition = pos;
    mLatestVelocity = vel;
}

KDL::JntArray HardwareInterface::getGripperPositions()
{
    std::lock_guard<std::mutex> lock(mGripperStateMutex);
    return mLatestGripperPosition;
}

KDL::JntArray HardwareInterface::getGripperVelocities()
{
    std::lock_guard<std::mutex> lock(mGripperStateMutex);
    return mLatestGripperVelocity;
}

void HardwareInterface::sendGripperCommand(const KDL::JntArray &aCmd)
{
    std::vector<double> goal(aCmd.rows());
    for(int i = 0; i < goal.size(); i++)
    {
        goal[i] = aCmd(i);
    }

    arm_msgs::msg::HardwareGoal goalMsg;
    goalMsg.set__position(goal);

    RosTopicManager::getInstance()->publishMessage("arm/hardware/gripper_goal", goalMsg);
}

void HardwareInterface::gripperStateCallback(arm_msgs::msg::HardwareState::SharedPtr aState)
{
    KDL::JntArray pos(aState->position.size());
    KDL::JntArray vel(aState->velocity.size());

    for(int i = 0; i < pos.rows(); i++)
    {
        pos(i) = aState->position[i];
        vel(i) = aState->velocity[i];
    }

    std::lock_guard<std::mutex> lock(mGripperStateMutex);
    mLatestGripperPosition = pos;
    mLatestGripperVelocity = vel;
}
