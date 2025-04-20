
#include <cstdio>
#include "Logger.h"
#include "ConfigManager.h"
#include "Manipulator.h"

int main()
{
    createLogger();
    std::string file = "./install/arm/share/arm/config.yaml";
    auto configManager = ConfigManager::getInstance(); 
    configManager->loadConfig(file); 

    Manipulator* manip = new Manipulator(configManager->getConfig().manipType); 

    return 0; 
}