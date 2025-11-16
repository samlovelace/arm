

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "common/ConfigManager.h"
#include "common/Logger.hpp"
#include "Engine.h"


int main()
{
    std::signal(SIGINT, signalHandler); 
    rclcpp::init(0, nullptr);
    createLogger("arm_engine_log-");

    std::string package_path = ament_index_cpp::get_package_share_directory("arm_configs");
    std::string config_path = package_path + "/config.yaml";
    auto configManager = ConfigManager::getInstance(); 
    configManager->loadConfig(config_path); 

    Engine e; 
    e.run(); 

    rclcpp::shutdown(); 
}