#ifndef CONFIGMANAGER_H
#define CONFIGMANAGER_H
#include <string> 

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
