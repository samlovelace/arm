#ifndef COMMANDHANDLER_H
#define COMMANDHANDLER_H

#include <memory> 

#include "StateMachine.h" 
#include "Manipulator.h"
#include "plog/Log.h"

#include "robot_idl/msg/joint_position_waypoint.hpp"
#include "robot_idl/msg/task_position_waypoint.hpp"
#include "robot_idl/msg/task_velocity_waypoint.hpp"
#include "robot_idl/msg/joint_velocity_waypoint.hpp"
#include "robot_idl/msg/enable.hpp"

class CommandHandler 
{ 
public:
    CommandHandler(std::shared_ptr<StateMachine> msm, std::shared_ptr<Manipulator> manip);
    ~CommandHandler();

private:

    void setNewActiveState(StateMachine::STATE aState); 

    void jointPosWaypointCallback(const robot_idl::msg::JointPositionWaypoint::SharedPtr aMsg);
    void taskPosWaypointCallback(const robot_idl::msg::TaskPositionWaypoint::SharedPtr aMsg); 
    void taskVelWaypointCallback(const robot_idl::msg::TaskVelocityWaypoint::SharedPtr aMsg); 
    void jointVelWaypointCallback(const robot_idl::msg::JointVelocityWaypoint::SharedPtr aMsg); 
    void enableCallback(const robot_idl::msg::Enable::SharedPtr anEnabledCmd); 

    std::shared_ptr<StateMachine> mStateMachine;
    std::shared_ptr<Manipulator> mManip;  

    bool mJointWaypointRcvd; 

    template<typename MsgType, typename WaypointType>
    void handleWaypoint(const typename MsgType::SharedPtr& msg,
                        const int aGoalSize, 
                        const int aProperSize,
                        std::function<WaypointType(const typename MsgType::SharedPtr&)> converter)
    {
        if (!mManip->isEnabled()) 
        {
            LOGW << "Manipulator not enabled. Cannot accept waypoint";
            return;
        }

        if(aGoalSize != aProperSize)
        {
            LOGW << "Rejecting waypoint with goal size (" << aGoalSize 
                 << ") not equal to proper size (" << aProperSize << ")."; 
            return; 
        }

        auto wp = converter(msg);
        if (!mManip->setGoalWaypoint(std::make_shared<WaypointType>(wp))) {
            LOGW << "Failed to set goal";
            return;
        }

        setNewActiveState(StateMachine::STATE::MOVING);
    }

   
};
#endif //COMMANDHANDLER_H