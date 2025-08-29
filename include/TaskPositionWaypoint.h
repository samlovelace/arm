#ifndef TASKPOSITIONWAYPOINT_H
#define TASKPOSITIONWAYPOINT_H

#include "IWaypoint.hpp"
#include <array>

class TaskPositionWaypoint final : public IWaypoint {
public:
  // tol = [ tx, ty, tz, r_roll, r_pitch, r_yaw ]  (meters, radians)
  using Tol6 = std::array<float, 6>;

  TaskPositionWaypoint(const KDL::Frame& T_goal, const Tol6& tol);

  Kind kind() const noexcept override;

  bool toJointGoal(const KDL::JntArray& q_seed,
                   KinematicsHandler& kin,
                   KDL::JntArray& q_goal_out) const override;

  bool arrived(const ControlInputs& s) const override;

  const KDL::Frame& goal() const noexcept;
  const Tol6&       tol()  const noexcept;

  std::string describe() const override;

private:
  KDL::Frame mGoal;
  Tol6       mTol;   // per-axis linear (m) and angular (rad) tolerances
};

#endif //TASKPOSITIONWAYPOINT_H