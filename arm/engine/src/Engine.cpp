
#include "Engine.h"
#include "common/RosTopicManager.hpp"
#include "common/RateController.hpp"
#include "common/ConfigManager.h"
#include "plog/Log.h"

Engine::Engine()
{
    RosTopicManager::getInstance()->createSubscriber<robot_idl::msg::ManipulationCommand>("/arm/command", 
                                                                                          std::bind(&Engine::commandCallback, 
                                                                                                    this, 
                                                                                                    std::placeholders::_1)); 
    RosTopicManager::getInstance()->spinNode(); 
}

Engine::~Engine()
{

}

bool Engine::init()
{
    while(!rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::seconds(1)); 
    }

    return true; 
}

void Engine::run()
{
    if(!init())
    {
        LOGE << "Failed to initialize Arm Engine"; 
        return; 
    }

    LOGI << "Engine Initialized Successfully!"; 

    int rate_hz = ConfigManager::getInstance()->getValue<int>("Engine.rate");
    RateController rate(rate_hz); 

    while(true)
    {
        rate.start(); 

        rate.block(); 
    }

}

void Engine::commandCallback(robot_idl::msg::ManipulationCommand::SharedPtr aCmd)
{
    using namespace robot_idl::msg; 

    switch (aCmd->cmd)
    {
    case ManipulationCommand::CMD_PICK:
        LOGD << "GOT PICK CMD"; 
        break;
    case ManipulationCommand::CMD_PLACE: 
        LOGD << "GOT PLACE CMD"; 
        break; 
    case ManipulationCommand::CMD_PLACE_REL:
        LOGD << "GOT PLACE REL CMD"; 
        break; 
    default:
        break;
    }
}