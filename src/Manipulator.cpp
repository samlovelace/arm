
#include "Manipulator.h"
//#include <kdl_parser/kdl_parser/kdl_parser.hpp>
#include <urdf/model.h> 
#include "ManipulatorFactory.h"
#include "plog/Log.h"

Manipulator::Manipulator(const std::string& aManipType) 
{
    //mManipComms = ManipulatorFactory::create(aManipType); 

    std::string urdfFilePath = "../../share/arm/" + aManipType + "/manipulator.urdf";

    urdf::Model model;
    if(!model.initFile(urdfFilePath))
    {
        LOGE << "Could not load manipulator urdf at " << urdfFilePath; 
        return; 
    }

}

Manipulator::~Manipulator()
{

}