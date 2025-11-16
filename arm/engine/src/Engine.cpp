
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

}

void Engine::run()
{
    int rate_hz = ConfigManager::getInstance()->getValue<int>("Engine.rate");
    RateController rate(rate_hz); 

    while(true)
    {
        rate.start(); 

        LOGV << "Engine running...";

        rate.block(); 
    }

}

void Engine::commandCallback(robot_idl::msg::ManipulationCommand::SharedPtr aCmd)
{

}