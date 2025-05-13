
#include "TrajectoryPlanner.h"
#include "plog/Log.h"

TrajectoryPlanner::TrajectoryPlanner(ConfigManager::Config& aConfig) : mPlanner(1/aConfig.manipControlRate)
{

}

TrajectoryPlanner::~TrajectoryPlanner()
{

}

KDL::JntArray TrajectoryPlanner::getNextWaypoint(std::shared_ptr<JointPositionWaypoint> aGoalWaypoint, KDL::JntArray aCurrentJointPos, KDL::JntArray aCurrentJointVel)
{
    std::array<double, 6> jntPos; 
    std::array<double, 6> jntVel; 
    std::array<double, 6> goalPos; 
    for(int i = 0; i < aCurrentJointPos.rows(); i++)
    {
        jntPos[i] = aCurrentJointPos(i);
        jntVel[i] = aCurrentJointVel(i);
        goalPos[i] = aGoalWaypoint->jointPositionGoal()(i);  
    }

    ruckig::OutputParameter<6> output; 
    ruckig::InputParameter<6> input; 
    // target
    input.target_position = goalPos; 
    input.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    input.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // current 
    input.current_position = jntPos; 
    input.current_velocity = jntVel; 
    
    // others
    input.degrees_of_freedom = 6; 
    input.max_acceleration = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    input.max_jerk = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01};
    input.max_velocity = {0.25, 0.25, 0.25, 0.25, 0.25, 0.25};
    
    KDL::JntArray newJntPos(6); 
    ruckig::Result result = mPlanner.update(input, output);

    if (result != ruckig::Result::Working && result != ruckig::Result::Finished) 
    {
        std::cerr << "Trajectory generation failed!" << std::endl;
    }
    
    for(int i = 0; i < output.new_position.size(); i++)
    {
        newJntPos(i) = output.new_position[i]; 
    }
    
    return newJntPos; 
}