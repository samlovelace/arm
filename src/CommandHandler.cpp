
#include "CommandHandler.h"
#include "RosTopicManager.hpp"
#include <std_msgs/msg/float64.hpp> 
#include "plog/Log.h"

CommandHandler::CommandHandler(StateMachine* msm) : mStateMachine(msm) 
{
    auto topicManager = RosTopicManager::getInstance(); 
    topicManager->createSubscriber<std_msgs::msg::Float64>("test", 
                                    std::bind(&CommandHandler::commandCallback, this, std::placeholders::_1)); 

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

