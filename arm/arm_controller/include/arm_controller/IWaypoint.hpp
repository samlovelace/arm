#ifndef IWAYPOINT_HPP
#define IWAYPOINT_HPP

#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <string>

class KinematicsHandler; // forward-declare your existing class

class IWaypoint {
public:
	enum class Type 
	{
		JointPosition,
    JointVelocity, 
		TaskPosition, 
		TaskVelocity 
	};

  virtual ~IWaypoint() = default;

  struct ControlInputs {
    KDL::JntArray q;     // current joint positions
    KDL::JntArray qdot;  // current joint velocities (optional)
    KDL::Frame    T;     // current end-effector pose (fill when needed)
  };

  virtual Type type() const noexcept = 0;

  // Convert this waypoint into a joint-space goal.
  // For Joint: copy the goal. For Task: call kin.solveIK with q_seed.
  virtual bool toJointGoal(const KDL::JntArray& q_seed,
                           KinematicsHandler& kin,
                           KDL::JntArray& q_goal_out) const = 0;

  // Check arrival using this waypoint's tolerances.
  virtual bool arrived(const ControlInputs& s) const = 0;

  // For logs
  virtual std::string describe() const = 0;
};

#endif 
