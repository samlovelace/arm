
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
#include <thread>

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
    mManipComms = ManipulatorFactory::create(aConfig.manipType); 
    
    //TODO: get this from config 
    mGoalJntPos = KDL::JntArray(6); 

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
    if(mControlThread.joinable())
    {
        mControlThread.join(); 
    }
}

bool Manipulator::sendToPose(Manipulator::POSE aPose)
{
    if(mArmPoseMap.find(aPose) != mArmPoseMap.end())
    {
        setJointPositionGoal(mArmPoseMap.at(aPose)); 
        return true; 
    }

    return false; 
}

void Manipulator::setJointPositionGoal(const KDL::JntArray &aNewJntPos)
{
    std::lock_guard<std::mutex> lock(mGoalJntPosMutex); 
    mGoalJntPos = aNewJntPos; 
}

void Manipulator::setEnabledState(bool anEnabledFlag)
{
    std::lock_guard<std::mutex> lock(mEnabledMutex); 
    mEnabled = anEnabledFlag; 
}

bool Manipulator::isEnabled()
{
    std::lock_guard<std::mutex> lock(mEnabledMutex); 
    return mEnabled; 
}

void Manipulator::setGoalWaypoint(const JointPositionWaypoint& aWp)
{
    mGoalWaypoint = aWp; 
} 

JointPositionWaypoint Manipulator::getGoalWaypoint()
{
    return mGoalWaypoint;     
}

bool Manipulator::isArrived()
{
    // get the goal waypoint manip is currently tracking
    JointPositionWaypoint goal = getGoalWaypoint(); 
    KDL::JntArray curr = mManipComms->getJointPositions(); 
    KDL::JntArray tol = goal.arrivalTolerance(); 

    for(int i = 0; i < curr.rows(); i++)
    {
        // TODO:: this probs wont work that well, need to be arrived for some time or check that vel is really small too
        if(abs(curr(i)) > tol(i))
        {
            return false; 
        }
    }
    return true;  
}

void Manipulator::startControl()
{
    mManipComms->init(); 
    mControlThread = std::thread(&Manipulator::controlLoop, this); 
    // anything else?? ? 
}

void Manipulator::controlLoop()
{
    LOGD << "Starting manipulator control loop!"; 
    // TODO: get arm control rate from config
    mArmControlRate = std::make_unique<RateController>(10); 

    while(isEnabled())
    {
        mArmControlRate->start(); 

        //KDL::JntArray wp = mTrajectoryPlanner->getNextWaypoint();
        JointPositionWaypoint wp = getGoalWaypoint(); 
        mManipComms->sendJointCommand(wp.jointPositionGoal()); 
    
        mArmControlRate->block(); 
    }
}