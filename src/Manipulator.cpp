
#include "Manipulator.h"

#include "ManipulatorFactory.h"
#include "plog/Log.h"
#include "Utils.h"

#include <unistd.h>
#include <limits.h>
#include <thread>
#include <fstream> 
#include <nlohmann/json.hpp>


Manipulator::Manipulator(ConfigManager::Config aConfig) : mConfig(aConfig),mWaypointExecutor(std::make_unique<WaypointExecutor>(mConfig)), 
mGoalWaypoint(std::make_shared<JointPositionWaypoint>()), mKinematicsHandler(std::make_shared<KinematicsHandler>()), mEnabled(false)
{
    mManipComms = ManipulatorFactory::create(aConfig.manipType, aConfig.manipCommsType); 
    mManipComms->init(); 
    
    KDL::JntArray firstWp(6);
    KDL::JntArray firstTol(6); 
    for(int i = 0; i < 6; i++)
    {
        firstWp(i) = aConfig.initialPosition[i]; 
        firstTol(i) = 0.01;
    }

    mGoalWaypoint->setJointPositionGoal(firstWp); 
    mGoalWaypoint->setArrivalTolerance(firstTol); 

    mKinematicsHandler->init(mConfig.urdfPath);

    if(mConfig.inverseReachMap != "")
        loadInverseReachabilityMap(mConfig.inverseReachMap); 
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
    mWaypointExecutor->initializeExecutor(aWp, mManipComms->getJointPositions(), mManipComms->getJointVelocities()); 
} 

std::shared_ptr<JointPositionWaypoint> Manipulator::getGoalWaypoint()
{
    return mGoalWaypoint;     
}

bool Manipulator::isArrived()
{
    // get the goal waypoint manip is currently tracking
    KDL::JntArray goal = getGoalWaypoint()->jointPositionGoal();
    KDL::JntArray tol = getGoalWaypoint()->arrivalTolerance();  
    
    // get the current joint position 
    KDL::JntArray curr = mManipComms->getJointPositions(); 

    for(int i = 0; i < curr.rows(); i++)
    {
        // TODO:: this probs wont work that well, need to be arrived for some time or check that vel is really small too
        if(abs(curr(i) - goal(i)) > tol(i))
        {
            return false; 
        }
    }

    // arrived so we can log final state
    std::stringstream ss;  
    ss << "Error: " << "\n"; 
    for(int i = 0; i < curr.rows(); i++)
    {
        ss << "J" << i << " (deg): " << (180.0/M_PI) * (curr(i) - goal(i)) << "\n"; 
    }

    LOGD << ss.str(); 
    return true;  
}

void Manipulator::startControl()
{
    mWaypointExecutor->initializeExecutor(mGoalWaypoint, mManipComms->getJointPositions(), mManipComms->getJointVelocities()); 
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

        KDL::JntArray wp = mWaypointExecutor->getNextWaypoint();
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

void Manipulator::loadInverseReachabilityMap(const std::string& aFilePath)
{   
    std::ifstream file(mConfig.inverseReachMap); 
    
    if (!file.is_open()) 
    {
        LOGE << "Failed to open " << mConfig.inverseReachMap; 
        return;
    }

    nlohmann::json jsonData;
    try 
    {
        file >> jsonData;
    } 
    catch (const std::exception& e) 
    {
        LOGE << "Error parsing JSON: " << e.what();
        return;
    }

    for(const auto& entry : jsonData["irm"])
    {
        // Parse position
        double x = entry["P_ee_base"][0];
        double y = entry["P_ee_base"][1];
        double z = entry["P_ee_base"][2];

        KDL::Vector pos(x, y, z);

        // Parse rotation (row-major order)
        const auto& R = entry["R_ee_base"];
        KDL::Rotation rot(
            R[0], R[1], R[2],  // Row 0
            R[3], R[4], R[5],  // Row 1
            R[6], R[7], R[8]   // Row 2
        );

        KDL::Frame frame(rot, pos); 
        KDL::JntArray jntCfg;
        
        auto jnts = entry["joints"]; 
        jntCfg.resize(jnts.size()); 

        for(int i = 0; i < jnts.size(); i++)
        {
            jntCfg(i) = jnts[i]; 
        }

        IrmEntry irmEntry; 
        irmEntry.T_ee_base = frame; 
        irmEntry.manipulability = entry["manipulability"]; 
        irmEntry.jntAngles = jntCfg; 

        mInverseReachabilityMap.push_back(irmEntry); 
    }

    LOGD << "Parsed manipulator inverse reachability map!";
}