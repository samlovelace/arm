
#include <cstdio>
#include "Logger.h"
#include "ConfigManager.h"
#include "Manipulator.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "StateMachine.h"
#include "CommandHandler.h"
#include "PlannerFactory.h"
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
    auto graspPlanner = PlannerFactory::createGraspPlanner(configManager->getRawConfig()["GraspPlanning"].as<YAML::Node>()); 
    auto planner = PlannerFactory::createArmTaskPlanner(configManager->getRawConfig()["Planning"]["type"].as<std::string>());
    planner->setGraspPlanner(graspPlanner); 

    auto sm = std::make_shared<StateMachine>(manip); 
    CommandHandler* cm = new CommandHandler(sm, manip, planner);
    sm->run(); 

    return 0; 
}