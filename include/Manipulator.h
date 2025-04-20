#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include "IManipComms.h"
#include "ConfigManager.h"
#include <memory> 

class Manipulator
{
public:
    Manipulator(const ConfigManager::Config aConfig);
    ~Manipulator();
private:
    std::unique_ptr<IManipComms> mManipComms;
    ConfigManager::Config mConfig; 

};
#endif //MANIPULATOR_H
