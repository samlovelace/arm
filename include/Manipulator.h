#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include <memory> 
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <mutex> 
#include <map> 
#include <thread> 

#include "IManipComms.h"
#include "ConfigManager.h"
#include "RateController.h"
#include "WaypointExecutor.h"
#include "IWaypoint.hpp"
#include "TaskPositionWaypoint.h"
#include "JointPositionWaypoint.h"
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
    bool setGoalWaypoint(std::shared_ptr<IWaypoint> aWp); 

    std::shared_ptr<IWaypoint> getGoalWaypoint(); 
    bool isArrived(); 
    bool sendToPose(Manipulator::POSE aPose); 
    void setEnabledState(bool anEnabledFlag); 

    const std::vector<IrmEntry>& getInverseReachabilityMap() const {return mInverseReachabilityMap; }
    std::shared_ptr<KinematicsHandler> getKinematicsHandler() {return mKinematicsHandler; }
    KDL::JntArray getCurrentJointPos() {return mManipComms->getJointPositions();}
    KDL::Frame getBaseInVehicleFrame() {return mConfig.T_V_B;}

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

    std::shared_ptr<IWaypoint> mInitialGoalWp; 
    std::shared_ptr<IWaypoint> mGoalWaypoint; 

    std::map<POSE, KDL::JntArray> mArmPoseMap; 

    std::vector<IrmEntry> mInverseReachabilityMap; 

    std::thread mControlThread; 
    void controlLoop(); 

    void logWaypointError(); 

    void loadInverseReachabilityMap(const std::string& aFilePath); 

};
#endif //MANIPULATOR_H
