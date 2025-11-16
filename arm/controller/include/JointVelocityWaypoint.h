#ifndef JOINTVELOCITYWAYPOINT_H
#define JOINTVELOCITYWAYPOINT_H

#include "IWaypoint.hpp"

class JointVelocityWaypoint : public IWaypoint
{
public:
    JointVelocityWaypoint(const KDL::JntArray& goal, const KDL::JntArray& tol);
    ~JointVelocityWaypoint();

    Type type() const noexcept override; 
    bool toJointGoal(const KDL::JntArray& q_see, 
                    KinematicsHandler& kin, 
                    KDL::JntArray& q_goal_out) const override; 
    bool arrived(const ControlInputs& s) const override; 
    std::string describe() const override; 
    

private:
    KDL::JntArray mGoal; // radians/sec
    KDL::JntArray mTol;  // per-joint radians

};
#endif

