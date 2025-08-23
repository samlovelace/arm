
#include "Manipulator.h"

#include "ManipulatorFactory.h"
#include "plog/Log.h"
#include "Utils.h"

#include <unistd.h>
#include <limits.h>
#include <thread>
#include <fstream> 
#include <simdjson.h>


Manipulator::Manipulator(ConfigManager::Config aConfig) : mConfig(aConfig),mWaypointExecutor(std::make_unique<WaypointExecutor>(mConfig)), 
mGoalWaypoint(std::make_shared<JointPositionWaypoint>()), mKinematicsHandler(std::make_shared<KinematicsHandler>()), mEnabled(false)
{
    mManipComms = ManipulatorFactory::create(aConfig.manipType, aConfig.manipCommsType); 
    mManipComms->init(); 

    if(!mKinematicsHandler->init(mConfig.urdfPath))
    {
        throw std::runtime_error("Failed to initialize kinematics"); 
    }

    int numJoints = mKinematicsHandler->getNrJoints(); 
    KDL::JntArray firstWp(numJoints);
    KDL::JntArray firstTol(numJoints); 

    for(int i = 0; i < numJoints; i++)
    {
        firstWp(i) = mConfig.initialPosition[i]; 
        firstTol(i) = 0.01;
    }

    mGoalWaypoint->setJointPositionGoal(firstWp); 
    mGoalWaypoint->setArrivalTolerance(firstTol); 

    if(mConfig.inverseReachMap != "")
    {
        loadInverseReachabilityMap(mConfig.inverseReachMap);
        
        if(mInverseReachabilityMap.empty())
        {
            throw std::runtime_error("Inverse Reachability Map is empty"); 
        }
    } 
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
    
    // check if thread needs joined first, required for multiple enable/disable cmds
    if(mControlThread.joinable())
    {
        mControlThread.join(); 
    }

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
    std::ifstream infile(aFilePath, std::ios::binary);
    if (!infile.is_open())
    {
        LOGE << "Failed to open IRM binary file: " << aFilePath;
        return;
    }

    // Determine file size and number of entries
    infile.seekg(0, std::ios::end);
    size_t fileSize = infile.tellg();
    infile.seekg(0, std::ios::beg);

    size_t numEntries = fileSize / sizeof(IrmEntryBinary);
    if (numEntries == 0)
    {
        LOGE << "IRM binary file is empty or invalid: " << aFilePath;
        return;
    }

    std::vector<IrmEntryBinary> rawEntries(numEntries);
    infile.read(reinterpret_cast<char*>(rawEntries.data()), fileSize);
    infile.close();

    mInverseReachabilityMap.clear();
    mInverseReachabilityMap.reserve(numEntries);

    for (const auto& raw : rawEntries)
    {
        // Build KDL::Frame from binary entry
        KDL::Rotation rot(
            raw.orientation[0], raw.orientation[1], raw.orientation[2],
            raw.orientation[3], raw.orientation[4], raw.orientation[5],
            raw.orientation[6], raw.orientation[7], raw.orientation[8]
        );

        KDL::Vector pos(raw.position[0], raw.position[1], raw.position[2]);
        KDL::Frame frame(rot, pos);

        IrmEntry irmEntry;
        irmEntry.T_ee_base = frame;
        irmEntry.manipulability = raw.manipulability;

        mInverseReachabilityMap.push_back(std::move(irmEntry));
    }

    LOGD << "Loaded " << mInverseReachabilityMap.size() << " IRM entries from binary file!";
}

