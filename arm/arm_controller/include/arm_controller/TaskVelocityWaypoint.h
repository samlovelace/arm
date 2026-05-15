#ifndef TASKVELOCITYWAYPOINT_H
#define TASKVELOCITYWAYPOINT_H
 
#include "IWaypoint.hpp"
 
class TaskVelocityWaypoint : public IWaypoint
{ 
public:
    TaskVelocityWaypoint(const KDL::Twist& aGoal, const KDL::Twist& aTol);
    ~TaskVelocityWaypoint();

    Type type() const noexcept override;  

    bool toJointGoal(const KDL::JntArray& q_seed,
					 KinematicsHandler& kin,
					 KDL::JntArray& q_goal_out) const override; 

    bool arrived(const ControlInputs& s) const override; 

    // For logs
    std::string describe() const override; 

private:
	KDL::Twist mGoal; 
	KDL::Twist mTol; 
};
#endif //TASKVELOCITYWAYPOINT_H