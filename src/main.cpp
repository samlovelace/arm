
#include <cstdio>
#include "Logger.h"
#include "ConfigManager.h"
#include "Manipulator.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "StateMachine.h"
#include "CommandHandler.h"
#include <rclcpp/rclcpp.hpp>

void handle(int signal)
{
    exit(0); 
}

int main()
{
    std::signal(SIGINT, handle); 

    rclcpp::init(0, nullptr);
    createLogger();

    std::string package_path = ament_index_cpp::get_package_share_directory("arm_configs");
    std::string config_path = package_path + "/config.yaml";
    auto configManager = ConfigManager::getInstance(); 
    configManager->loadConfig(config_path); 

    auto manip = std::make_shared<Manipulator>(configManager->getConfig()); 

    auto sm = std::make_shared<StateMachine>(manip); 
    CommandHandler* cm = new CommandHandler(sm, manip); 
    sm->run(); 

    return 0; 
}