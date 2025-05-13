#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H
 
#include <kdl/jntarray.hpp>
#include "ConfigManager.h"
#include "JointPositionWaypoint.h"
#include "ruckig/ruckig.hpp"

class TrajectoryPlanner 
{ 
public:
    TrajectoryPlanner(ConfigManager::Config& aConfig);
    ~TrajectoryPlanner();

    KDL::JntArray getNextWaypoint(JointPositionWaypoint aGoalWp, KDL::JntArray aCurrentJointPos, KDL::JntArray aCurrentJointVel);

private:
    ruckig::Ruckig<6> mPlanner; 
   
};
#endif //TRAJECTORYPLANNER_H    