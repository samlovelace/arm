#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include <memory> 
#include <kdl/jntarray.hpp>
#include <mutex> 
#include <map> 
#include <thread> 

#include "IManipComms.h"
#include "ConfigManager.h"
#include "RateController.h"
#include "WaypointExecutor.h"
#include "JointPositionWaypoint.h"
#include "TaskPositionWaypoint.h"
#include "KinematicsHandler.h"
#include "IrmEntry.hpp"

class Manipulator
{
public:
    Manipulator(const ConfigManager::Config aConfig);
    ~Manipulator();

    enum class POSE
    {
        STOW, 
        DEPLOY, 
        NUM_TYPES
    }; 

    void setJointPositionGoal(const KDL::JntArray &aNewJntPos); 
    void setGoalWaypoint(std::shared_ptr<JointPositionWaypoint> aWp); 
    void setTaskGoal(std::shared_ptr<TaskPositionWaypoint> aWp); 

    std::shared_ptr<JointPositionWaypoint> getGoalWaypoint(); 
    bool isArrived(); 
    bool sendToPose(Manipulator::POSE aPose); 
    void setEnabledState(bool anEnabledFlag); 

    std::vector<IrmEntry>& getInverseReachabilityMap() {return mInverseReachabilityMap; }
    std::shared_ptr<KinematicsHandler> getKinematicsHandler() {return mKinematicsHandler; }
    KDL::JntArray getCurrentJointPos() {return mManipComms->getJointPositions();}

    void startControl(); 

    bool isEnabled(); 

private:
    std::shared_ptr<IManipComms> mManipComms;
    ConfigManager::Config mConfig; 
    std::unique_ptr<WaypointExecutor> mWaypointExecutor; 
    std::shared_ptr<KinematicsHandler> mKinematicsHandler; 

    std::mutex mGoalJntPosMutex; 
    KDL::JntArray mGoalJntPos;
    std::unique_ptr<RateController> mArmControlRate; 

    std::mutex mEnabledMutex; 
    bool mEnabled; 

    std::shared_ptr<JointPositionWaypoint> mGoalWaypoint; 

    std::map<POSE, KDL::JntArray> mArmPoseMap; 

    std::vector<IrmEntry> mInverseReachabilityMap; 

    std::thread mControlThread; 
    void controlLoop(); 

    void logWaypointError(); 

    void loadInverseReachabilityMap(const std::string& aFilePath); 

};
#endif //MANIPULATOR_H
