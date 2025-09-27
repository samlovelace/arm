
#include "JointVelocityWaypoint.h"
#include <stdexcept>
#include <sstream>

JointVelocityWaypoint::JointVelocityWaypoint(const KDL::JntArray& goal, const KDL::JntArray& tol) : mGoal(goal), mTol(tol)
{
    if (goal.rows() == 0 || tol.rows() != goal.rows()) 
    {
        throw std::invalid_argument("JointVelocityWaypoint: size mismatch");
    }

    for (unsigned i = 0; i < mTol.rows(); ++i) 
    {
        if (mTol(i) < 0.0) 
            throw std::invalid_argument("JointVelocityWaypoint: negative tolerance");
    }
}

JointVelocityWaypoint::~JointVelocityWaypoint()
{

}

IWaypoint::Type JointVelocityWaypoint::type() const noexcept
{
    return IWaypoint::Type::JointVelocity; 
}

bool JointVelocityWaypoint::toJointGoal(const KDL::JntArray& /*q_seed*/,
                                        KinematicsHandler& /*kin*/,
                                        KDL::JntArray& q_goal_out) const
{
    q_goal_out = mGoal; // trivial copy
    return true;
}

bool JointVelocityWaypoint::arrived(const ControlInputs& s) const 
{
    //
    return false; 
}

std::string JointVelocityWaypoint::describe() const
{
    std::ostringstream oss;
    oss << "JointVelocityWaypoint(n=" << mGoal.rows() << ")";
    return oss.str();
}