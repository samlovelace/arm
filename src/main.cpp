
#include <cstdio>
#include "Logger.h"
#include "ConfigManager.h"
#include "Manipulator.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "StateMachine.h"
#include "CommandHandler.h"
#include <rclcpp/rclcpp.hpp>

int main()
{
    rclcpp::init(0, nullptr);
    createLogger();

    std::string package_path = ament_index_cpp::get_package_share_directory("arm");
    std::string config_path = package_path + "/configuration/config.yaml";
    auto configManager = ConfigManager::getInstance(); 
    configManager->loadConfig(config_path); 

    Manipulator* manip = new Manipulator(configManager->getConfig()); 

    StateMachine* sm = new StateMachine(); 
    CommandHandler* cm = new CommandHandler(sm); 
    sm->run(); 

    return 0; 
}