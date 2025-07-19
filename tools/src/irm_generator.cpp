#include <iostream>
#include "ConfigManager.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "IrmGenerator.h"

int main()
{
    std::string package_path = ament_index_cpp::get_package_share_directory("arm_configs");
    std::string config_path = package_path + "/config.yaml";
    auto configManager = ConfigManager::getInstance(); 
    configManager->loadConfig(config_path);

    auto config = configManager->getConfig(); 

    IrmGenerator irm(config); 
    irm.generate(); 

    return 0; 
}