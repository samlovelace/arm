
#include "ConfigManager.h"
#include <yaml-cpp/yaml.h>
#include "plog/Log.h"

void ConfigManager::loadConfig(const std::string& aConfigFilepath)
{
    std::string shareDirFull = aConfigFilepath; 
    mYamlConfig = YAML::LoadFile(aConfigFilepath); 
    LOGD << YAML::Dump(mYamlConfig); 

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

    std::string manipConfigPath = shareDirFull + "manipulators/" + mYamlConfig["Manipulator"]["name"].as<std::string>();
    LOGD << "manipulator config path: " << manipConfigPath; 

    YAML::Node manipConfig = YAML::LoadFile(manipConfigPath + "/manip.yaml");
    mManipConfig = manipConfig; // save as member var
    LOGD << "Manip Config: " << YAML::Dump(manipConfig); 

    mConfig.manipType = manipConfig["type"].as<std::string>();
    mConfig.manipCommsType = manipConfig["comms"].as<std::string>();  
    mConfig.manipControlRate = manipConfig["rate"].as<int>(); 
    mConfig.urdfPath = manipConfigPath + "/" + manipConfig["urdf"].as<std::string>(); 

    if(manipConfig["irm"])
    {
        mConfig.inverseReachMap = manipConfigPath + "/" + manipConfig["irm"].as<std::string>(); 
    }
    
    mConfig.initialPosition = manipConfig["initial_positions"].as<std::vector<double>>();
    mConfig.accelLimit = manipConfig["accel_limits"].as<std::vector<double>>();
    mConfig.jerkLimit = manipConfig["jerk_limits"].as<std::vector<double>>(); 
    mConfig.velLimitFraction = manipConfig["vel_limit_fraction"].as<double>(); 

    auto xyz = manipConfig["mount"]["position"].as<std::vector<double>>(); 
    auto q = manipConfig["mount"]["quaternion"].as<std::vector<double>>(); 

    KDL::Vector p(xyz[0], xyz[1], xyz[2]); 
    KDL::Rotation r(KDL::Rotation::Quaternion(q[0],q[1],q[2],q[3]));
    
    KDL::Frame T_V_B(r, p); 
    mConfig.T_V_B = T_V_B; 
}