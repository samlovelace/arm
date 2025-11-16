#ifndef WAYPOINTEXECUTOR_H
#define WAYPOINTEXECUTOR_H
 
#include <kdl/jntarray.hpp>
#include <memory> 

#include "ConfigManager.h"
#include "JointPositionWaypoint.h"
#include "ruckig/ruckig.hpp"
#include "common/KinematicsHandler.h"

class WaypointExecutor 
{ 
public:
    WaypointExecutor(ConfigManager::Config& aConfig);
    ~WaypointExecutor();

    void init(int aNumDof, KinematicsHandler& aKinematicsHandler);
    bool initializeExecutor(KDL::JntArray aGoalJointPos, KDL::JntArray aCurrentJointPos, KDL::JntArray aCurrentJointVel, ruckig::ControlInterface aControlType);
    KDL::JntArray getNextWaypoint();

private:
    std::unique_ptr<ruckig::Ruckig<0>> mExecutor; // 0 indicates dynamic
    std::unique_ptr<ruckig::InputParameter<0>> mInput;
    std::unique_ptr<ruckig::OutputParameter<0>> mOutput;

    ConfigManager::Config mConfig;
    std::vector<double> mVelocityLimits; 
    std::vector<double> mLowerPosLimits; 
    std::vector<double> mUpperPosLimits;  

    bool mInitialSet;

    int mNumDof; 
    
    void setInitialState(const KDL::JntArray& aPos, const KDL::JntArray& aVel);
    bool setGoalState(KDL::JntArray aGoalJointPos, ruckig::ControlInterface aControlType);
   
};
#endif //TRAJECTORYExecutor_H    