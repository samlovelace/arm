#include <iostream>
#include "ConfigManager.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "KinematicsHandler.h"

int main()
{
    std::string package_path = ament_index_cpp::get_package_share_directory("arm_configs");
    std::string config_path = package_path + "/config.yaml";
    auto configManager = ConfigManager::getInstance(); 
    configManager->loadConfig(config_path);

    auto config = configManager->getConfig(); 

    std::string urdfFilePath = config.shareDir + "manipulators/" + config.manipType + "/manipulator.urdf";
    std::cout << "urdfFilePath: " << urdfFilePath << std::endl; 

    KinematicsHandler kh; 
    if(!kh.init(urdfFilePath))
    {
        std::cerr << "Failed to initialize kinematics handler\n"; 
        return 0; 
    } 

    std::cout << "Initializd kinematics handler!\n"; 
    return 0; 
}