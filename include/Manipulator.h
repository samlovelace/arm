#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include "IManipComms.h"
#include "ConfigManager.h"
#include <memory> 
#include <kdl/jntarray.hpp>

class Manipulator
{
public:
    Manipulator(const ConfigManager::Config aConfig);
    ~Manipulator();

    bool updateJointPositionGoal(const KDL::JntArray &aNewJntPos); 

private:
    std::unique_ptr<IManipComms> mManipComms;
    ConfigManager::Config mConfig; 

};
#endif //MANIPULATOR_H
