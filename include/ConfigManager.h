#ifndef CONFIGMANAGER_H
#define CONFIGMANAGER_H

#include <string> 
#include <vector>
#include <yaml-cpp/yaml.h>
#include <kdl/frames.hpp>

class ConfigManager
{
public:
    
    static ConfigManager* getInstance()
    {
        static ConfigManager mInstance;  
        return &mInstance; 
    }

    struct Config   
    {
        std::string shareDir; 
        std::string manipType;
        std::string manipCommsType;  
        std::string urdfPath; 
        std::string inverseReachMap = ""; 
        int manipControlRate; 
        std::vector<double> initialPosition; 
        std::vector<double> accelLimit; 
        std::vector<double> jerkLimit; 
        double velLimitFraction; 
        KDL::Frame T_V_B; 
    };

    void loadConfig(const std::string& aConfigFilepath); 
    Config getConfig() {return mConfig; }
    YAML::Node& getRawConfig() {return mYamlConfig; }
    YAML::Node& getManipConfig() {return mManipConfig; }

private:
    ConfigManager(/* args */) {}
    ~ConfigManager() {}

    ConfigManager* mInstance; 

    Config mConfig; 
    YAML::Node mYamlConfig; 
    YAML::Node mManipConfig; 

};
#endif 
