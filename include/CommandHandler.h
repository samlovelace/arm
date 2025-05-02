#ifndef COMMANDHANDLER_H
#define COMMANDHANDLER_H
 
#include "StateMachine.h" 
#include "arm_idl/msg/joint_position_waypoint.hpp"

class CommandHandler 
{ 
public:
    CommandHandler(StateMachine* msm);
    ~CommandHandler();

private:

    void jointPosWaypointCallback(const arm_idl::msg::JointPositionWaypoint::SharedPtr aMsg);

    StateMachine* mStateMachine; 
   
};
#endif //COMMANDHANDLER_H