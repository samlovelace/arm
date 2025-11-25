#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H

#include "common/KinematicsHandler.h"

// undefine logger macros before touching OMPL
#undef LOG_DEBUG
#undef LOG_INFO
#undef LOG_WARN
#undef LOG_ERROR
#undef LOG_NONE

#include "kdl/jntarray.hpp"
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ob = ompl::base; 
namespace og = ompl::geometric; 

class TrajectoryPlanner
{
public:
    TrajectoryPlanner(std::shared_ptr<KinematicsHandler> aKine);
    ~TrajectoryPlanner();

    
bool plan(const KDL::JntArray& aStartState, const KDL::JntArray& aGoalState,
          std::vector<KDL::JntArray>& aPathOut);

    ob::StateSpacePtr mStateSpace; 
    ob::SpaceInformationPtr mSpaceInfo; 
    ob::ProblemDefinitionPtr mProblemDef; 
    ob::PlannerPtr mPlanner; 

    std::shared_ptr<KinematicsHandler> mKinematicsHandler; 

};
#endif