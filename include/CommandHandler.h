#ifndef COMMANDHANDLER_H
#define COMMANDHANDLER_H

#include <memory> 

#include "StateMachine.h" 
#include "Manipulator.h"
#include "IArmTaskPlanner.hpp"

#include "arm_idl/msg/joint_position_waypoint.hpp"
#include "arm_idl/msg/enable.hpp"
#include "arm_idl/msg/task_position_waypoint.hpp"
#include "arm_idl/msg/command.hpp"

class CommandHandler 
{ 
public:
    CommandHandler(std::shared_ptr<StateMachine> msm, std::shared_ptr<Manipulator> manip, std::shared_ptr<IArmTaskPlanner> planner);
    ~CommandHandler();

private:

    void setNewActiveState(StateMachine::STATE aState); 

    void jointPosWaypointCallback(const arm_idl::msg::JointPositionWaypoint::SharedPtr aMsg);
    void taskPosWaypointCallback(const arm_idl::msg::TaskPositionWaypoint::SharedPtr aMsg); 
    void enableCallback(const arm_idl::msg::Enable::SharedPtr anEnabledCmd); 
    void commandCallback(const arm_idl::msg::Command::SharedPtr aCmd); 

    std::shared_ptr<StateMachine> mStateMachine;
    std::shared_ptr<Manipulator> mManip;  
    std::shared_ptr<IArmTaskPlanner> mArmTaskPlanner; 

    bool mJointWaypointRcvd; 
   
};
#endif //COMMANDHANDLER_H