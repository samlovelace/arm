#ifndef COMMANDHANDLER_H
#define COMMANDHANDLER_H

#include <memory> 

#include "StateMachine.h" 
#include "Manipulator.h"
#include "arm_idl/msg/joint_position_waypoint.hpp"

class CommandHandler 
{ 
public:
    CommandHandler(std::shared_ptr<StateMachine> msm, std::shared_ptr<Manipulator> manip);
    ~CommandHandler();

private:

    void setNewActiveState(StateMachine::STATE aState); 

    void jointPosWaypointCallback(const arm_idl::msg::JointPositionWaypoint::SharedPtr aMsg);

    std::shared_ptr<StateMachine> mStateMachine;
    std::shared_ptr<Manipulator> mManip;  
   
};
#endif //COMMANDHANDLER_H