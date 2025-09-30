
#include "WaypointExecutor.h"
#include "plog/Log.h"

WaypointExecutor::WaypointExecutor(ConfigManager::Config& aConfig) : mConfig(aConfig)
{

}

WaypointExecutor::~WaypointExecutor()
{

}

void WaypointExecutor::init(int aNumDof, KinematicsHandler& aKinematicsHandler)
{
    mNumDof = aNumDof;
    KDL::JntArray velLimits = aKinematicsHandler.getJointLimits("velocity");
    
    mVelocityLimits.resize(mNumDof); 

    for(int i = 0; i < mNumDof; i++)
    {
        mVelocityLimits[i] = mConfig.velLimitFraction * velLimits(i); 
    }

    mExecutor = std::make_unique<ruckig::Ruckig<0>>(mNumDof, 1/(float)mConfig.manipControlRate); // pass timestep
    mInput = std::make_unique<ruckig::InputParameter<0>>(mNumDof);
    mOutput = std::make_unique<ruckig::OutputParameter<0>>(mNumDof);
}

bool WaypointExecutor::initializeExecutor(KDL::JntArray aGoalJointPos, KDL::JntArray aCurrentJointPos, KDL::JntArray aCurrentJointVel, ruckig::ControlInterface aControlType)
{
    setInitialState(aCurrentJointPos, aCurrentJointVel); 
    return setGoalState(aGoalJointPos, aControlType); 
}

bool WaypointExecutor::setGoalState(KDL::JntArray aGoalWaypoint, ruckig::ControlInterface aControlType)
{
    if(mNumDof != aGoalWaypoint.rows())
    {
        LOGW << "Goal state not equal to DOF size"; 
        return false; 
    }
    
    std::vector<double> goalPos(mNumDof); 
    std::vector<double> goalVel(mNumDof); 
    std::vector<double> goalAccel(mNumDof); 

    mInput->control_interface = aControlType;

    switch (aControlType)
    {
        case ruckig::ControlInterface::Position:
            { 
                for(int i = 0; i < mNumDof; i++)
                {
                    goalPos[i] = aGoalWaypoint(i); 
                    goalVel[i] = 0.0; 
                    goalAccel[i] = 0.0;  
                }

                // target
                mInput->target_position = goalPos; 
                mInput->target_velocity = goalVel; 
                mInput->target_acceleration = goalAccel; 

                LOGD << "Setting Goal Joint Pos: " << mInput->target_position;
            }
        break;
    case ruckig::ControlInterface::Velocity: 
        {
            for(int i = 0; i < mNumDof; i++)
            { 
                goalVel[i] = aGoalWaypoint(i); 
                goalAccel[i] = 0.0;  
            }

            // target
            mInput->target_velocity = goalVel; 
            mInput->target_acceleration = goalAccel; 

            LOGD << "Setting Goal Joint Vel: " << mInput->target_velocity;
        }
        break; 
    default:
        break;
    }

    return true; 
}

void WaypointExecutor::setInitialState(const KDL::JntArray& aPos, const KDL::JntArray& aVel)
{
    for(int i = 0; i < aPos.rows(); i++)
    {
        mInput->current_position[i] = aPos(i); 
        mInput->current_velocity[i] = aVel(i);

        // TODO: compute this in the IManipComms and populate here 
        mInput->current_acceleration[i] = 0.0; 
    }

    mInput->degrees_of_freedom = mNumDof;  
    std::copy_n(mVelocityLimits.begin(), (int)mInput->degrees_of_freedom, mInput->max_velocity.begin()); 
    std::copy_n(mConfig.accelLimit.begin(), (int)mInput->degrees_of_freedom, mInput->max_acceleration.begin()); 
    std::copy_n(mConfig.jerkLimit.begin(), (int)mInput->degrees_of_freedom, mInput->max_jerk.begin()); 
}

KDL::JntArray WaypointExecutor::getNextWaypoint()
{
    KDL::JntArray newJntPos(mNumDof);
    auto result = mExecutor->update(*mInput, *mOutput);

    if (result == ruckig::Result::Working || result == ruckig::Result::Finished) 
    {
        for (size_t i = 0; i < mNumDof; ++i) 
        {
            newJntPos(i) = mOutput->new_position[i];
        }

        mOutput->pass_to_input(*mInput);
    } 
    else 
    {
        // Handle error
    }

    return newJntPos;
}