
#include "JointPositionWaypoint.h"
#include "common/KinematicsHandler.h"
#include <stdexcept>
#include <sstream>
#include <cmath>
#include "plog/Log.h"
#include <iomanip>

JointPositionWaypoint::JointPositionWaypoint(const KDL::JntArray& goal, const KDL::JntArray& tol) : mGoal(goal), mTol(tol)
{
    if (goal.rows() == 0 || tol.rows() != goal.rows()) 
    {
        throw std::invalid_argument("JointPositionWaypoint: size mismatch");
    }
    for (unsigned i = 0; i < mTol.rows(); ++i) 
    {
        if (mTol(i) < 0.0) 
            throw std::invalid_argument("JointPositionWaypoint: negative tolerance");
    }
}

IWaypoint::Type JointPositionWaypoint::type() const noexcept 
{
    return Type::JointPosition;
}

bool JointPositionWaypoint::toJointGoal(const KDL::JntArray& /*q_seed*/,
                                KinematicsHandler& /*kin*/,
                                KDL::JntArray& q_goal_out) const
{
    q_goal_out = mGoal; // trivial copy
    return true;
}

bool JointPositionWaypoint::arrived(const ControlInputs& s) const
{
    if (s.q.rows() != mGoal.rows()) 
    {
        return false; 
    }

    for (unsigned i = 0; i < mGoal.rows(); ++i) 
    {
        if (std::fabs(s.q(i) - mGoal(i)) > mTol(i)) 
        {
            return false;
        }
    }
    
    return true;
}

const KDL::JntArray& JointPositionWaypoint::goal() const noexcept { return mGoal; }
const KDL::JntArray& JointPositionWaypoint::tol()  const noexcept { return mTol;  }

std::string JointPositionWaypoint::describe() const 
{
    std::ostringstream oss;
    oss << "JointPositionWaypoint(n=" << mGoal.rows() << ")";
    return oss.str();
}
