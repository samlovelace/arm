
#include "WaypointExecutor.h"
#include "plog/Log.h"

WaypointExecutor::WaypointExecutor(ConfigManager::Config& aConfig) : mExecutor(1/(float)aConfig.manipControlRate), mConfig(aConfig), mInput(), mOutput()
{

}

WaypointExecutor::~WaypointExecutor()
{

}

void WaypointExecutor::initializeExecutor(std::shared_ptr<JointPositionWaypoint> aGoalWaypoint, KDL::JntArray aCurrentJointPos, KDL::JntArray aCurrentJointVel)
{
    setInitialState(aCurrentJointPos, aCurrentJointVel); 
    setGoalState(aGoalWaypoint); 
}

void WaypointExecutor::setGoalState(std::shared_ptr<JointPositionWaypoint> aGoalWaypoint)
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

void WaypointExecutor::setInitialState(const KDL::JntArray& aPos, const KDL::JntArray& aVel)
{
    for(int i = 0; i < aPos.rows(); i++)
    {
        mInput.current_position[i] = aPos(i); 
        mInput.current_velocity[i] = aVel(i);

        // TODO: compute this in the IManipComms and populate here 
        mInput.current_acceleration[i] = 0.0; 
    }

    mInput.degrees_of_freedom = mConfig.initialPosition.size(); 

    // TODO: get from kinematics handler somehow
    mInput.max_velocity = {1.5, 1.5, 1.5, 1.5, 1.5, 1.5}; 

    std::copy_n(mConfig.accelLimit.begin(), (int)mInput.degrees_of_freedom, mInput.max_acceleration.begin()); 
    std::copy_n(mConfig.jerkLimit.begin(), (int)mInput.degrees_of_freedom, mInput.max_jerk.begin()); 
}

KDL::JntArray WaypointExecutor::getNextWaypoint()
{
    KDL::JntArray newJntPos(6);
    auto result = mExecutor.update(mInput, mOutput);

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