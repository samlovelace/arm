#ifndef WAYPOINTEXECUTOR_H
#define WAYPOINTEXECUTOR_H
 
#include <kdl/jntarray.hpp>
#include <memory> 
#include "ConfigManager.h"
#include "JointPositionWaypoint.h"
#include "ruckig/ruckig.hpp"

class WaypointExecutor 
{ 
public:
    WaypointExecutor(ConfigManager::Config& aConfig);
    ~WaypointExecutor();

    void initializeExecutor(std::shared_ptr<JointPositionWaypoint> aGoalWaypoint, KDL::JntArray aCurrentJointPos, KDL::JntArray aCurrentJointVel);
    KDL::JntArray getNextWaypoint();

private:
    ruckig::Ruckig<6> mExecutor; 
    ruckig::InputParameter<6> mInput; 
    ruckig::OutputParameter<6> mOutput; 

    ConfigManager::Config mConfig; 

    bool mInitialSet;
    
    void setInitialState(const KDL::JntArray& aPos, const KDL::JntArray& aVel);
    void setGoalState(std::shared_ptr<JointPositionWaypoint> aGoalWaypoint);
   
};
#endif //TRAJECTORYExecutor_H    