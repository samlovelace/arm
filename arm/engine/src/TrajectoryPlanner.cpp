
#include "TrajectoryPlanner.h"
#include "plog/Log.h"

TrajectoryPlanner::TrajectoryPlanner(std::shared_ptr<KinematicsHandler> aKine) : mKinematicsHandler(aKine)
{

}

TrajectoryPlanner::~TrajectoryPlanner()
{

}

bool TrajectoryPlanner::plan(const KDL::JntArray& aStartState, const KDL::JntArray& aGoalState,
                             std::vector<KDL::JntArray>& aPathOut)
{
    int dof = mKinematicsHandler->getNrJoints();  
    auto space = std::make_shared<ob::RealVectorStateSpace>(dof); 

    ob::RealVectorBounds bounds(dof); 
    KDL::JntArray lower = mKinematicsHandler->getJointLimits("lower"); 
    KDL::JntArray upper = mKinematicsHandler->getJointLimits("upper"); 

    for(int i = 0; i < dof; i++)
    {
        bounds.setLow(i, lower(i)); 
        bounds.setHigh(i, upper(i)); 
    }

    space->setBounds(bounds); 

    og::SimpleSetup ss(space); 

    ss.setStateValidityChecker([&](const ob::State* s)
    {
        const auto* q = s->as<ob::RealVectorStateSpace::StateType>(); 

        KDL::JntArray jnts(dof); 
        for(int i = 0; i < dof; i++)
        {
            jnts(i) = q->values[i]; 
        }

        // if not in collision, state is valid
        return !mKinematicsHandler->checkCollisions(jnts); 
    });

    ob::ScopedState<> start(space), goal(space); 

    for(int i = 0; i < dof; i++)
    {
        start[i] = aStartState(i); 
        goal[i] = aGoalState(i); 
    }

    ss.setStartAndGoalStates(start, goal); 

    auto planner = std::make_shared<og::RRTConnect>(ss.getSpaceInformation()); 
    ss.setPlanner(planner); 

    // solve
    ob::PlannerStatus solved = ss.solve(2.0); 

    if(!solved)
    {
        LOGW << "Failed to solve trajectory"; 
        return false; 
    }

    LOGV << "Found feasible trajectory!"; 
    ss.simplifySolution(); 
    og::PathGeometric path = ss.getSolutionPath(); 
    path.interpolate(5); 

    aPathOut.clear(); 
    for(size_t i = 0; i < path.getStateCount(); i++)
    {
        const ob::State* s = path.getState(i); 
        const auto* q = s->as<ob::RealVectorStateSpace::StateType>(); 

        KDL::JntArray jnts(dof); 
        for(int j = 0; j < dof; j++)
        {
            jnts(j) = q->values[j]; 
        }

        aPathOut.push_back(jnts); 
    }

    return true; 
}