
#include "TrajectoryPlanner.h"
#include "plog/Log.h"

TrajectoryPlanner::TrajectoryPlanner(ConfigManager::Config& aConfig) : mPlanner(1/(float)aConfig.manipControlRate), mInput(), mOutput()
{

}

TrajectoryPlanner::~TrajectoryPlanner()
{

}

void TrajectoryPlanner::initializePlanner(std::shared_ptr<JointPositionWaypoint> aGoalWaypoint, KDL::JntArray aCurrentJointPos, KDL::JntArray aCurrentJointVel)
{
    setInitialState(aCurrentJointPos, aCurrentJointVel); 
    setGoalState(aGoalWaypoint); 
}

void TrajectoryPlanner::setGoalState(std::shared_ptr<JointPositionWaypoint> aGoalWaypoint)
{
    std::array<double, 6> goalPos; 
    for(int i = 0; i < aGoalWaypoint->jointPositionGoal().rows(); i++)
    {
        goalPos[i] = aGoalWaypoint->jointPositionGoal()(i);  
    }

    // target
    mInput.target_position = goalPos; 
    mInput.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    mInput.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    LOGD << "Setting Goal Joint Pos: " << mInput.target_position; 
}

void TrajectoryPlanner::setInitialState(const KDL::JntArray& aPos, const KDL::JntArray& aVel)
{
    for(int i = 0; i < aPos.rows(); i++)
    {
        mInput.current_position[i] = aPos(i); 
        mInput.current_velocity[i] = aVel(i);

        // TODO: compute this in the IManipComms and populate here 
        mInput.current_acceleration[i] = 0.0; 
    }

    // TODO: get from config 
    mInput.degrees_of_freedom = 6; 
    mInput.max_velocity = {1.5, 1.5, 1.5, 1.5, 1.5, 1.5};
    mInput.max_acceleration = {3.14, 3.14, 3.14, 3.14, 3.14, 3.14};
    mInput.max_jerk = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0};
}

KDL::JntArray TrajectoryPlanner::getNextWaypoint()
{
    KDL::JntArray newJntPos(6);
    auto result = mPlanner.update(mInput, mOutput);

    if (result == ruckig::Result::Working || result == ruckig::Result::Finished) {
        for (size_t i = 0; i < 6; ++i) {
            newJntPos(i) = mOutput.new_position[i];
        }

        mOutput.pass_to_input(mInput);
    } else 
    {
        // Handle error
    }

    return newJntPos;
}