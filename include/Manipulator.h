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
#include "TrajectoryPlanner.h"
#include "JointPositionWaypoint.h"

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
    std::shared_ptr<JointPositionWaypoint> getGoalWaypoint(); 
    bool isArrived(); 
    bool sendToPose(Manipulator::POSE aPose); 
    void setEnabledState(bool anEnabledFlag); 

    

    void startControl(); 

    bool isEnabled(); 

private:
    std::shared_ptr<IManipComms> mManipComms;
    ConfigManager::Config mConfig; 
    std::unique_ptr<TrajectoryPlanner> mTrajectoryPlanner; 

    std::mutex mGoalJntPosMutex; 
    KDL::JntArray mGoalJntPos;
    std::unique_ptr<RateController> mArmControlRate; 

    std::mutex mEnabledMutex; 
    bool mEnabled; 

    std::shared_ptr<JointPositionWaypoint> mGoalWaypoint; 

    std::map<POSE, KDL::JntArray> mArmPoseMap; 

    std::thread mControlThread; 
    void controlLoop(); 



};
#endif //MANIPULATOR_H
