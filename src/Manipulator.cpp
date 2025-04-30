
#include "Manipulator.h"
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
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

    KDL::Chain chain; 
    if(!tree.getChain("base_link", "tool0", chain))
    {
        LOGE << "Failed to get chain from base_link to tool0"; 
        return; 
    }

    size_t nj = chain.getNrOfJoints(); 
    KDL::JntArray joint_positions(nj);

    // Set some example joint positions (adjust for your robot)
    joint_positions(0) = 0.0;
    joint_positions(1) = -0.5;
    joint_positions(2) = 1.0;
    joint_positions(3) = 0.0;
    joint_positions(4) = 1.2;
    joint_positions(5) = -0.8;

    KDL::ChainFkSolverPos_recursive fk_solver(chain); 



}

Manipulator::~Manipulator()
{

}