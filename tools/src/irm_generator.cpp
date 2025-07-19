#include <iostream>
#include "ConfigManager.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono> 

#include "IrmGenerator.h"

int main()
{
    std::string package_path = ament_index_cpp::get_package_share_directory("arm_configs");
    std::string config_path = package_path + "/config.yaml";
    auto configManager = ConfigManager::getInstance(); 
    configManager->loadConfig(config_path);

    auto config = configManager->getConfig(); 

    IrmGenerator irm(config);
    auto start = std::chrono::steady_clock::now();  
    irm.generate(); 
    auto end = std::chrono::steady_clock::now(); 

    std::cout << "Generated Inverse Reachability Map in " << std::chrono::duration_cast<std::chrono::seconds>(end-start).count() << " s" << std::endl; 
    return 0; 
}