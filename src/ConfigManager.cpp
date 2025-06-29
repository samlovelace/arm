
#include "ConfigManager.h"
#include <yaml-cpp/yaml.h>
#include "plog/Log.h"

void ConfigManager::loadConfig(const std::string& aConfigFilepath)
{
    std::string shareDirFull = aConfigFilepath; 
    YAML::Node yamlConfig = YAML::LoadFile(aConfigFilepath); 
    LOGD << YAML::Dump(yamlConfig); 

    // Substring to find and remove
    std::string to_remove = "config.yaml";
    size_t pos = shareDirFull.find(to_remove);
    
    if (pos != std::string::npos) {
        // Remove the substring
        shareDirFull.erase(pos, to_remove.length());
        LOGD << "Package share dir: " << shareDirFull; 
    }

    // TODO: error checking on the things im accessing, im lazy rn
    mConfig.shareDir = shareDirFull;
    mConfig.manipType = yamlConfig["Manipulator"]["type"].as<std::string>();
    mConfig.manipCommsType = yamlConfig["Manipulator"]["comms"].as<std::string>();  
    mConfig.manipControlRate = yamlConfig["Manipulator"]["rate"].as<int>(); 
    mConfig.initialPosition = yamlConfig["Manipulator"]["initial_positions"].as<std::vector<double>>();
    mConfig.accelLimit = yamlConfig["Manipulator"]["accel_limits"].as<std::vector<double>>();
    mConfig.jerkLimit = yamlConfig["Manipulator"]["jerk_limits"].as<std::vector<double>>(); 
}