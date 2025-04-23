
#include "Manipulator.h"
#include <kdl_parser/kdl_parser.hpp>
#include "ManipulatorFactory.h"
#include "plog/Log.h"

#include <unistd.h>
#include <limits.h>

Manipulator::Manipulator(ConfigManager::Config aConfig) : mConfig(aConfig) 
{
    //mManipComms = ManipulatorFactory::create(aManipType); 

    std::string urdfFilePath = mConfig.shareDir + mConfig.manipType + "/manipulator.urdf";
    LOGW << "urdfFilePath: " << urdfFilePath;

    KDL::Tree tree; 
    if(!kdl_parser::treeFromFile(urdfFilePath, tree))
    {
        LOGE << "Failed to parse urdf to KDL::Tree"; 
        return; 
    } 

    LOGD << "Parsed manipulator with " << tree.getNrOfJoints() << " joints"; 
}

Manipulator::~Manipulator()
{

}