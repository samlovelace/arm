#ifndef CONFIGMANAGER_H
#define CONFIGMANAGER_H

#include <string> 
#include <vector>

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
        int manipControlRate; 
        std::vector<double> initialPosition; 
        std::vector<double> accelLimit; 
        std::vector<double> jerkLimit; 
    };

    void loadConfig(const std::string& aConfigFilepath); 
    Config getConfig() {return mConfig; }

private:
    ConfigManager(/* args */) {}
    ~ConfigManager() {}

    ConfigManager* mInstance; 

    Config mConfig; 

};
#endif 
