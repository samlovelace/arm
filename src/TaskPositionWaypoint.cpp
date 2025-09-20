#include "TaskPositionWaypoint.h"
#include "KinematicsHandler.h"
#include <stdexcept>
#include <sstream>
#include <cmath>
#include "plog/Log.h"

TaskPositionWaypoint::TaskPositionWaypoint(const KDL::Frame& T_goal, const Tol6& tol) : mGoal(T_goal), mTol(tol)
{
    for (double v : mTol) 
    {
        if (v < 0.0) throw std::invalid_argument("TaskPositionWaypoint: negative tolerance");
    }
}

IWaypoint::Type TaskPositionWaypoint::type() const noexcept 
{
  return Type::TaskPosition;
}

bool TaskPositionWaypoint::toJointGoal(const KDL::JntArray& q_seed,
                                  KinematicsHandler& kin,
                                  KDL::JntArray& q_goal_out) const
{
    KDL::JntArray q_sol(q_seed.rows());
    
    if (!kin.solveIK(q_seed, mGoal, q_sol)) 
    {
        return false;
    }

    q_goal_out = q_sol;
    return true;
}

bool TaskPositionWaypoint::arrived(const ControlInputs& s) const
{
    // Linear error: err = goal * current^{-1}
    const KDL::Frame err = mGoal * s.T.Inverse();

    // Per-axis linear checks (meters)
    if (std::fabs(err.p.x()) > mTol[0]) return false;
    if (std::fabs(err.p.y()) > mTol[1]) return false;
    if (std::fabs(err.p.z()) > mTol[2]) return false;

    // Angular error via axis-angle of rotation part
    KDL::Vector axis;
    double angle = 0.0;
    err.M.GetRotAngle(axis, angle);
    const KDL::Vector rvec = axis * angle; // compare component-wise to [rx, ry, rz]

    if (std::fabs(rvec[0]) > mTol[3]) return false;
    if (std::fabs(rvec[1]) > mTol[4]) return false;
    if (std::fabs(rvec[2]) > mTol[5]) return false;

    // if here, arrived, so log task pos error
    // arrived so we can log final state
    std::stringstream ss;  
    ss << "Task Error: " << "\n";
    std::vector<std::string> axes = {"x: ", "y: ", "z: "}; 

    for(int i = 0; i < 3; i++)
    {
        ss << axes[i] << " (m): " << err.p.data[i] << "\n"; 
    }

    LOGD << ss.str();

    return true;
}

const KDL::Frame& TaskPositionWaypoint::goal() const noexcept { return mGoal; }
const TaskPositionWaypoint::Tol6& TaskPositionWaypoint::tol() const noexcept { return mTol; }

std::string TaskPositionWaypoint::describe() const 
{
    std::ostringstream oss;
    oss << "TaskPositionWaypoint";
    return oss.str();
}
