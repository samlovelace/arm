
#include <cstdio>
#include "Logger.h"
#include "ConfigManager.h"
#include "Manipulator.h"
#include "ManipulatorFactory.h"

int main()
{
    createLogger();
    std::string file = "./install/arm/share/arm/config.yaml";
    auto configManager = ConfigManager::getInstance(); 
    configManager->loadConfig(file); 

    IManipComms* manipComms = ManipulatorFactory::create(configManager->getConfig().manipType); 

    Manipulator* manip = new Manipulator(manipComms); 

    return 0; 
}