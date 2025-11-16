
#include <cstdio>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "common/Logger.hpp"
#include "common/ConfigManager.h"
#include "Manipulator.h"
#include "StateMachine.h"
#include "CommandHandler.h"
#include "PlannerFactory.h"


int main()
{
    std::signal(SIGINT, signalHandler); 

    rclcpp::init(0, nullptr);
    createLogger("arm_controller_log-");

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