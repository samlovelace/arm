#ifndef JOINTPOSITIONWAYPOINT_H
#define JOINTPOSITIONWAYPOINT_H

#include "IWaypoint.hpp"

class JointPositionWaypoint final : public IWaypoint 
{

public:
    JointPositionWaypoint(const KDL::JntArray& goal, const KDL::JntArray& tol);

    Type type() const noexcept override;

    bool toJointGoal(const KDL::JntArray& q_seed,
                    KinematicsHandler& kin,
                    KDL::JntArray& q_goal_out) const override;

    bool arrived(const ControlInputs& s) const override;

    const KDL::JntArray& goal() const noexcept;
    const KDL::JntArray& tol()  const noexcept;

    std::string describe() const override;

private:
    KDL::JntArray mGoal; // radians
    KDL::JntArray mTol;  // per-joint radians
};

#endif 