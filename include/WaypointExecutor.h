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
    std::unique_ptr<ruckig::Ruckig<0>> mExecutor; // 0 indicates dynamic
    std::unique_ptr<ruckig::InputParameter<0>> mInput;
    std::unique_ptr<ruckig::OutputParameter<0>> mOutput;

    ConfigManager::Config mConfig; 

    bool mInitialSet;

    int mNumDof; 
    
    void setInitialState(const KDL::JntArray& aPos, const KDL::JntArray& aVel);
    void setGoalState(std::shared_ptr<JointPositionWaypoint> aGoalWaypoint);
   
};
#endif //TRAJECTORYExecutor_H    