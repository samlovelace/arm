
#include "Manipulator.h"

#include "ManipulatorFactory.h"
#include "plog/Log.h"
#include "Utils.h"

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

Manipulator::Manipulator(ConfigManager::Config aConfig) : mConfig(aConfig), mTrajectoryPlanner(std::make_unique<TrajectoryPlanner>(mConfig)), 
mGoalWaypoint(std::make_shared<JointPositionWaypoint>()), mKinematicsHandler(std::make_shared<KinematicsHandler>())
{
    mManipComms = ManipulatorFactory::create(aConfig.manipType); 
    mManipComms->init(); 
    
    //TODO: get this from config 
    KDL::JntArray firstWp(6);
    KDL::JntArray firstTol(6); 
    for(int i = 0; i < 6; i++)
    {
        firstWp(i) = 1.0; 
        firstTol(i) = 0.01;
    }

    mGoalWaypoint->setJointPositionGoal(firstWp); 
    mGoalWaypoint->setArrivalTolerance(firstTol); 

    std::string urdfFilePath = mConfig.shareDir + "manipulators/" + mConfig.manipType + "/manipulator.urdf";
    LOGW << "urdfFilePath: " << urdfFilePath;

    mKinematicsHandler->init(urdfFilePath);
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

void Manipulator::setGoalWaypoint(std::shared_ptr<JointPositionWaypoint> aWp)
{
    mGoalWaypoint = aWp; 
    mTrajectoryPlanner->initializePlanner(aWp, mManipComms->getJointPositions(), mManipComms->getJointVelocities()); 
} 

std::shared_ptr<JointPositionWaypoint> Manipulator::getGoalWaypoint()
{
    return mGoalWaypoint;     
}

bool Manipulator::isArrived()
{
    // get the goal waypoint manip is currently tracking
    auto goal = getGoalWaypoint(); 
    KDL::JntArray curr = mManipComms->getJointPositions(); 
    KDL::JntArray tol = goal->arrivalTolerance(); 

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
    mTrajectoryPlanner->initializePlanner(mGoalWaypoint, mManipComms->getJointPositions(), mManipComms->getJointVelocities()); 
    mControlThread = std::thread(&Manipulator::controlLoop, this); 
    
    // anything else?? 
}

void Manipulator::controlLoop()
{
    LOGD << "Starting manipulator control loop!"; 
    
    mArmControlRate = std::make_unique<RateController>(mConfig.manipControlRate); 

    while(isEnabled())
    {
        mArmControlRate->start(); 

        KDL::JntArray wp = mTrajectoryPlanner->getNextWaypoint();
        mManipComms->sendJointCommand(wp); 

        mArmControlRate->block();
    }
}

void Manipulator::setTaskGoal(std::shared_ptr<TaskPositionWaypoint> aWp)
{
    KDL::JntArray curPos = mManipComms->getJointPositions(); 
    KDL::JntArray resultPos(6); 

    mKinematicsHandler->solveIK(curPos, aWp->getGoalPose(), resultPos);

    utils::printJntArray(resultPos, "IK Results"); 

    if(0 != resultPos.rows())
    {
        // TODO: not a huge fan of this section, maybe there is a better way to track a certain goal joint position
        // TODO: this arrival is nonsensical but i want to test the task pos 
        std::vector<double> tol = aWp->getArrivalTolerances(); 
        KDL::JntArray arrival(aWp->getArrivalTolerances().size()); 

        for(int i = 0; i < aWp->getArrivalTolerances().size(); i++)
        {
            arrival(i) = tol[i]; 
        } 

        auto wp = std::make_shared<JointPositionWaypoint>(resultPos, arrival); 
        setGoalWaypoint(wp);
    }

}