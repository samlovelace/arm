
#include "ConfigManager.h"
#include <yaml-cpp/yaml.h>
#include "plog/Log.h"

void ConfigManager::loadConfig(const std::string& aConfigFilepath)
{
    YAML::Node yamlConfig = YAML::LoadFile(aConfigFilepath); 
    LOGD << YAML::Dump(yamlConfig); 

    mConfig.manipType = yamlConfig["manipulator"].as<std::string>(); 
}