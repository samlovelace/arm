#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include <memory> 
#include <kdl/jntarray.hpp>
#include <mutex> 
#include <map> 

#include "IManipComms.h"
#include "ConfigManager.h"
#include "RateController.h"
#include "TrajectoryPlanner.h"

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
    bool sendToPose(Manipulator::POSE aPose); 
    void setEnabledState(bool anEnabledFlag); 

    bool isEnabled(); 

private:
    std::unique_ptr<IManipComms> mManipComms;
    ConfigManager::Config mConfig; 
    std::unique_ptr<TrajectoryPlanner> mTrajectoryPlanner; 

    std::mutex mGoalJntPosMutex; 
    KDL::JntArray mGoalJntPos;
    std::unique_ptr<RateController> mArmControlRate; 

    std::mutex mEnabledMutex; 
    bool mEnabled; 

    std::map<POSE, KDL::JntArray> mArmPoseMap; 

    void controlLoop(); 



};
#endif //MANIPULATOR_H
