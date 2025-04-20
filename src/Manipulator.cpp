
#include "Manipulator.h"
//#include <kdl_parser/kdl_parser/kdl_parser.hpp>
#include <urdf/model.h> 
#include "ManipulatorFactory.h"
#include "plog/Log.h"

#include <unistd.h>
#include <limits.h>

Manipulator::Manipulator(ConfigManager::Config aConfig) : mConfig(aConfig) 
{
    //mManipComms = ManipulatorFactory::create(aManipType); 

    std::string urdfFilePath = mConfig.shareDir + mConfig.manipType + "/manipulator.urdf";
    LOGW << "urdfFilePath: " << urdfFilePath;

    urdf::Model model;
    if(!model.initFile(urdfFilePath))
    {
        LOGE << "Could not load manipulator urdf at " << urdfFilePath; 
        return; 
    }

    LOGD << "Parsed urdf for " << model.getName(); 

}

Manipulator::~Manipulator()
{

}