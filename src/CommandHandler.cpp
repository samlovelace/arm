
#include "CommandHandler.h"
#include "RosTopicManager.hpp"
#include "plog/Log.h"

CommandHandler::CommandHandler(StateMachine* msm) : mStateMachine(msm) 
{
    auto topicManager = RosTopicManager::getInstance(); 
    topicManager->createSubscriber<arm_idl::msg::JointPositionWaypoint>("arm/joint_position_waypoint", 
                                    std::bind(&CommandHandler::jointPosWaypointCallback, this, std::placeholders::_1)); 

    /**
     * Implement subscriber for custom msgs representing different manip waypoint types i.e joint pos, joint vel, task pos, task vel 
     * and the different options for each of those waypoints. 
     * 
     * Implement a callback function to attach to a subscriber for those topics. Callback function will parse the command, convert 
     * to internal data type and change the state machine STATE to move towards that goal waypoint. I think maybe there only needs to be a 
     * MOVING state and it may do different things based on the TYPE of waypoint commanded in order to send joint pos commands 
     * 
     */

    topicManager->spinNode(); 

    while (!topicManager->isROSInitialized())
    {
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

void CommandHandler::jointPosWaypointCallback(const arm_idl::msg::JointPositionWaypoint::SharedPtr aMsg)
{
    
     
}

