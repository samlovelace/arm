#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H
 
#include <kdl/jntarray.hpp>
#include <memory> 
#include "ConfigManager.h"
#include "JointPositionWaypoint.h"
#include "ruckig/ruckig.hpp"

class TrajectoryPlanner 
{ 
public:
    TrajectoryPlanner(ConfigManager::Config& aConfig);
    ~TrajectoryPlanner();

    void initializePlanner(std::shared_ptr<JointPositionWaypoint> aGoalWaypoint, KDL::JntArray aCurrentJointPos, KDL::JntArray aCurrentJointVel);
    KDL::JntArray getNextWaypoint();

private:
    ruckig::Ruckig<6> mPlanner; 
    ruckig::InputParameter<6> mInput; 
    ruckig::OutputParameter<6> mOutput; 

    bool mInitialSet;
    
    void setInitialState(const KDL::JntArray& aPos, const KDL::JntArray& aVel);
    void setGoalState(std::shared_ptr<JointPositionWaypoint> aGoalWaypoint);
   
};
#endif //TRAJECTORYPLANNER_H    