
#include "TaskVelocityWaypoint.h"
#include "KinematicsHandler.h"
#include <stdexcept>
#include <sstream>
#include <cmath>
#include "plog/Log.h"
#include <iomanip>

TaskVelocityWaypoint::TaskVelocityWaypoint(const KDL::Twist& aGoal, const KDL::Twist& aTol) : mGoal(aGoal), mTol(aTol)
{
   // TODO: validate tolerance 
}

TaskVelocityWaypoint::~TaskVelocityWaypoint()
{

}

IWaypoint::Type TaskVelocityWaypoint::type() const noexcept 
{
    return Type::TaskVelocity;
}

bool TaskVelocityWaypoint::toJointGoal(const KDL::JntArray& q_seed,
                                       KinematicsHandler& aKinematicsHandler,
                                       KDL::JntArray& q_goal_out) const
{
    int numJoints = q_seed.rows(); 
    KDL::JntArray q_dot_goal(numJoints); 

    if(!aKinematicsHandler.solveIK(q_seed, mGoal, q_dot_goal))
    {
        LOGW << "Failed to generate joint vel goal from task vel"; 
        return false; 
    } 

    q_goal_out = q_dot_goal; 
    return true; 

    // TODO: add Velocity saturation 
    // aKinematicsHandler.getJointLimits("velocity"); 

    // for (unsigned int i = 0; i < n; ++i) {
    //     if (i < qdot_max.size() && qdot_max[i] > 0.0) {
    //         qdot(i) = std::max(-qdot_max[i], std::min(qdot_max[i], qdot(i)));
    //     }
    // }

    // KDL::JntArray qmin = aKinematicsHandler.getJointLimits("lower"); 
    // KDL::JntArray qmax = aKinematicsHandler.getJointLimits("upper"); 

    // Integrate
    // q_goal_out.resize(numJoints);

    // for (unsigned int i = 0; i < numJoints; ++i) 
    // {
    //     //double qi = q_seed(i) + dt * q_dot_goal(i);
        
    //     if (i < qmin.rows() && i < qmax.rows()) 
    //     {
    //         qi = std::max(qmin(i), std::min(qmax(i), qi));
    //     }

    //     q_goal_out(i) = qi;
    // }

}

bool TaskVelocityWaypoint::arrived(const ControlInputs& s) const
{
    // should a task vel waypoint ever arrive? 
    // with the current state machine, arrival would transition the state machine to IDLE
    return false; 
}

std::string TaskVelocityWaypoint::describe() const 
{
    std::ostringstream oss;
    oss << "TaskVelocityWaypoint";
    return oss.str();
}
