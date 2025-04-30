
#include "CommandHandler.h"
#include "RosTopicManager.hpp"
#include <std_msgs/msg/float64.hpp> 
#include "plog/Log.h"

CommandHandler::CommandHandler(StateMachine* msm) : mStateMachine(msm) 
{
    auto topicManager = RosTopicManager::getInstance(); 
    topicManager->createSubscriber<std_msgs::msg::Float64>("test", 
                                    std::bind(&CommandHandler::commandCallback, this, std::placeholders::_1)); 

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

void CommandHandler::commandCallback(const std_msgs::msg::Float64::SharedPtr aMsg)
{
    LOGD << "GOT A MESSAGE"; 
}

