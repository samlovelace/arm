
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

/**
 * COuld own a trajectoryPLanner object repsonsible for finding a smooth polynomial to comand the manip through 
 * to acheive the goal joint pos from the current joint pos. 
 * 
 * Take in different types of waypoints and map to joint pos waypoint, then find smooth polynomial trajectory 
 * 
 * Joint position waypoint: directly compute smooth polynomial to execute 
 * Joint vel waypoint? 
 * 
 * Task Position waypoint: 
 *  - inverse kinematics to map to joint space, global would need full system jacobian (including mobile base)
 *  - command frame: gl, base
 * 
 * Task Vel waypoint ? 
 */

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

bool Manipulator::updateJointPositionGoal(const KDL::JntArray &aNewJntPos)
{
    
}