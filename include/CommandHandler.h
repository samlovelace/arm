#ifndef COMMANDHANDLER_H
#define COMMANDHANDLER_H

#include <memory> 

#include "StateMachine.h" 
#include "Manipulator.h"
#include "IArmTaskPlanner.hpp"

#include "robot_idl/msg/joint_position_waypoint.hpp"
#include "robot_idl/msg/task_position_waypoint.hpp"
#include "robot_idl/msg/task_velocity_waypoint.hpp"
#include  "robot_idl/msg/joint_velocity_waypoint.hpp"
#include "robot_idl/msg/enable.hpp"
#include "robot_idl/msg/plan_command.hpp"
#include "robot_idl/msg/plan_response.hpp"

class CommandHandler 
{ 
public:
    CommandHandler(std::shared_ptr<StateMachine> msm, std::shared_ptr<Manipulator> manip, std::shared_ptr<IArmTaskPlanner> planner);
    ~CommandHandler();

private:

    void setNewActiveState(StateMachine::STATE aState); 

    void jointPosWaypointCallback(const robot_idl::msg::JointPositionWaypoint::SharedPtr aMsg);
    void taskPosWaypointCallback(const robot_idl::msg::TaskPositionWaypoint::SharedPtr aMsg); 
    void taskVelWaypointCallback(const robot_idl::msg::TaskVelocityWaypoint::SharedPtr aMsg); 
    void jointVelWaypointCallback(const robot_idl::msg::JointVelocityWaypoint::SharedPtr aMsg); 
    void enableCallback(const robot_idl::msg::Enable::SharedPtr anEnabledCmd); 
    void commandCallback(const robot_idl::msg::PlanCommand::SharedPtr aCmd); 

    std::shared_ptr<StateMachine> mStateMachine;
    std::shared_ptr<Manipulator> mManip;  
    std::shared_ptr<IArmTaskPlanner> mArmTaskPlanner; 

    bool mJointWaypointRcvd; 
   
};
#endif //COMMANDHANDLER_H