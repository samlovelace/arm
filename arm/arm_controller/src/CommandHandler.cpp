

#include <plog/Log.h>
#include <kdl/jntarray.hpp>

#include "arm_common/RosTopicManager.hpp"
#include "arm_common/Utils.hpp"

#include "arm_controller/CommandHandler.h"

#include "arm_controller/JointPositionWaypoint.h"
#include "arm_controller/TaskPositionWaypoint.h"
#include "arm_controller/TaskVelocityWaypoint.h"
#include "arm_controller/JointVelocityWaypoint.h"

CommandHandler::CommandHandler(std::shared_ptr<StateMachine> msm, std::shared_ptr<Manipulator> manip) :
    mStateMachine(msm), mManip(manip)
{
    auto topicManager = RosTopicManager::getInstance();
    topicManager->createSubscriber<ptera_msgs::msg::JointPositionWaypoint>("arm/joint_position_waypoint",
                                                                          std::bind(&CommandHandler::jointPosWaypointCallback,
                                                                                    this,
                                                                                    std::placeholders::_1));

    topicManager->createSubscriber<ptera_msgs::msg::JointPositionWaypoint>("arm/gripper_position_waypoint",
                                                                          std::bind(&CommandHandler::gripperPosWaypointCallback,
                                                                                    this,
                                                                                    std::placeholders::_1));

    topicManager->createSubscriber<ptera_msgs::msg::Enable>("arm/enable",
                                                           std::bind(&CommandHandler::enableCallback,
                                                                     this,
                                                                     std::placeholders::_1));

    topicManager->createSubscriber<ptera_msgs::msg::TaskPositionWaypoint>("arm/task_position_waypoint",
                                                                         std::bind(&CommandHandler::taskPosWaypointCallback,
                                                                                   this,
                                                                                   std::placeholders::_1));

    topicManager->createSubscriber<ptera_msgs::msg::TaskVelocityWaypoint>("arm/task_velocity_waypoint",
                                                                         std::bind(&CommandHandler::taskVelWaypointCallback,
                                                                                  this,
                                                                                  std::placeholders::_1));
    topicManager->createSubscriber<ptera_msgs::msg::JointVelocityWaypoint>("arm/joint_velocity_waypoint",
                                                                         std::bind(&CommandHandler::jointVelWaypointCallback,
                                                                                    this,
                                                                                    std::placeholders::_1));

    while (!topicManager->isROSInitialized())
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    LOGD << "ROS Comms Initialized";
}

CommandHandler::~CommandHandler()
{

}

void CommandHandler::setNewActiveState(StateMachine::STATE aNewState)
{
    auto currentState = mStateMachine->getActiveState();

    if(aNewState != currentState)
    {
        mStateMachine->setActiveState(aNewState);
    }
}

void CommandHandler::enableCallback(const ptera_msgs::msg::Enable::SharedPtr anEnabledCmd)
{
    // TODO: check hardware name in message and use to inform enable behavior

    // determine how to transition the state machine
    if(anEnabledCmd->enabled && !mManip->isEnabled())
    {
        LOGV << "Recieved enable command";
        mManip->setEnabledState(anEnabledCmd->enabled);
        mManip->startControl();
        setNewActiveState(StateMachine::STATE::IDLE);
    }
    else if (!anEnabledCmd->enabled && mManip->isEnabled())
    {
        LOGV << "Recieved disable command";
        mManip->setEnabledState(anEnabledCmd->enabled);
        setNewActiveState(StateMachine::STATE::DISABLED);
    }
}

void CommandHandler::jointPosWaypointCallback(const ptera_msgs::msg::JointPositionWaypoint::SharedPtr aMsg)
{
    int numJoints = mManip->getKinematicsHandler()->getNrJoints();
    int goalSize = aMsg->positions.size();

    handleWaypoint<ptera_msgs::msg::JointPositionWaypoint,
                   JointPositionWaypoint>(aMsg, goalSize, numJoints, [](const auto& m)
    {
        return JointPositionWaypoint(utils::toJntArray(m->positions), utils::toJntArray(m->tolerances));
    });
}

void CommandHandler::gripperPosWaypointCallback(const ptera_msgs::msg::JointPositionWaypoint::SharedPtr aMsg)
{
    // TODO: once hardware names used, move gripper pos setting to jointPosWaypoint path, check for gripper name

    if (!mManip->isEnabled())
    {
        LOGW << "Manipulator not enabled. Cannot accept gripper waypoint";
        return;
    }

    int properSize = mManip->getGripperNrJoints();
    int goalSize = aMsg->positions.size();

    if (goalSize != properSize)
    {
        LOGW << "Rejecting gripper waypoint with goal size (" << goalSize
             << ") not equal to proper size (" << properSize << ").";
        return;
    }

    // Direct pass-through — no WaypointExecutor/Ruckig, no state-machine
    // transition. Gripper motion doesn't need arm-style arrival tracking.
    mManip->setGripperGoal(utils::toJntArray(aMsg->positions));
}

void CommandHandler::jointVelWaypointCallback(const ptera_msgs::msg::JointVelocityWaypoint::SharedPtr aMsg)
{
    int numJoints = mManip->getKinematicsHandler()->getNrJoints();
    int goalSize = aMsg->velocities.size();

    handleWaypoint<ptera_msgs::msg::JointVelocityWaypoint, JointVelocityWaypoint>(aMsg, goalSize, numJoints,
         [](const auto& m)
    {
        return JointVelocityWaypoint(utils::toJntArray(m->velocities), utils::toJntArray(m->tolerances));
    });
}

void CommandHandler::taskPosWaypointCallback(const ptera_msgs::msg::TaskPositionWaypoint::SharedPtr aMsg)
{

    handleWaypoint<ptera_msgs::msg::TaskPositionWaypoint, TaskPositionWaypoint>(aMsg, -1, -1,
        [](const auto& wp)
    {
        return TaskPositionWaypoint(utils::toFrame(wp->pose), utils::toArray6(wp->tolerance));
    });
}

void CommandHandler::taskVelWaypointCallback(const ptera_msgs::msg::TaskVelocityWaypoint::SharedPtr aMsg)
{
    handleWaypoint<ptera_msgs::msg::TaskVelocityWaypoint, TaskVelocityWaypoint>(aMsg, -1, -1,
        [](const auto& wp)
    {
        return TaskVelocityWaypoint(utils::toTwist(wp->goal), utils::toTwist(wp->tolerance));
    });
}
