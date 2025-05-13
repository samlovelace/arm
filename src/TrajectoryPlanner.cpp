
#include "TrajectoryPlanner.h"
#include "plog/Log.h"

TrajectoryPlanner::TrajectoryPlanner(ConfigManager::Config& aConfig) : mPlanner(1/aConfig.manipControlRate)
{

}

TrajectoryPlanner::~TrajectoryPlanner()
{

}

KDL::JntArray TrajectoryPlanner::getNextWaypoint(JointPositionWaypoint aGoalWaypoint, KDL::JntArray aCurrentJointPos, KDL::JntArray aCurrentJointVel)
{
    LOGW << "getNextWaypointCalled()"; 
    std::array<double, 6> jntPos; 
    std::array<double, 6> jntVel; 
    std::array<double, 6> goalPos; 
    for(int i = 0; i < aCurrentJointPos.rows(); i++)
    {
        jntPos[i] = aCurrentJointPos(i);
        jntVel[i] = aCurrentJointVel(i);
        goalPos[i] = aGoalWaypoint.jointPositionGoal()(i);  
    }

    LOGW << "Assigned std::array(s)"; 

    ruckig::OutputParameter<6> output; 
    ruckig::InputParameter<6> input; 
    input.target_position = goalPos; 
    input.current_position = jntPos; 
    input.current_velocity = jntVel; 
    input.degrees_of_freedom = 6; 
    input.max_acceleration = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    input.max_jerk = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
    input.max_velocity = {0.25, 0.25, 0.25, 0.25, 0.25, 0.25};
    
    KDL::JntArray newJntPos; 
    if(mPlanner.validate_input(input))
    {
        LOGW << "rucking could not validate input"; 
        return newJntPos; 
    }
    
    mPlanner.update(input, output); 
    LOGW << "after mPLanner.update()";
        

    
    for(int i = 0; i < output.new_position.size(); i++)
    {
        newJntPos(i) = output.new_position[i]; 
    }

    LOGW << "returning newJntPos"; 
    return newJntPos; 
}