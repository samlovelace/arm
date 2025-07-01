
#include "CommandHandler.h"
#include "RosTopicManager.hpp"
#include "plog/Log.h"
#include <kdl/jntarray.hpp>
#include "PointCloudHandler.h"

CommandHandler::CommandHandler(std::shared_ptr<StateMachine> msm, std::shared_ptr<Manipulator> manip, std::shared_ptr<IArmTaskPlanner> planner) : 
    mStateMachine(msm), mManip(manip), mArmTaskPlanner(planner), mJointWaypointRcvd(false)
{
    auto topicManager = RosTopicManager::getInstance(); 
    topicManager->createSubscriber<arm_idl::msg::JointPositionWaypoint>("arm/joint_position_waypoint", 
                                                                        std::bind(&CommandHandler::jointPosWaypointCallback, 
                                                                                  this, 
                                                                                  std::placeholders::_1)); 

    topicManager->createSubscriber<arm_idl::msg::Enable>("arm/enable", 
                                                         std::bind(&CommandHandler::enableCallback, 
                                                                   this, 
                                                                   std::placeholders::_1)); 

    topicManager->createSubscriber<arm_idl::msg::TaskPositionWaypoint>("arm/task_position_waypoint", 
                                                                       std::bind(&CommandHandler::taskPosWaypointCallback, 
                                                                                this, 
                                                                                std::placeholders::_1)); 

    topicManager->createSubscriber<arm_idl::msg::PlanCommand>("arm/command", 
                                                          std::bind(&CommandHandler::commandCallback, 
                                                                    this, 
                                                                    std::placeholders::_1)); 

    topicManager->spinNode(); 

    while (!topicManager->isROSInitialized())
    {
    }
    
    LOGD << "ROS Comms Initialized";
}

CommandHandler::~CommandHandler()
{

}

void CommandHandler::setNewActiveState(StateMachine::STATE aNewState)
{
    auto currentState = mStateMachine->getActiveState(); 

    if(aNewState != currentState)
    { 
        mStateMachine->setActiveState(aNewState); 
    }
}

void CommandHandler::enableCallback(const arm_idl::msg::Enable::SharedPtr anEnabledCmd)
{ 
    // determine how to transition the state machine 
    if(anEnabledCmd->enabled && !mManip->isEnabled())
    {
        // send default stow jntPos so the arm doesnt move on first enable
        mManip->setEnabledState(anEnabledCmd->enabled); 
        mManip->startControl(); 
        setNewActiveState(StateMachine::STATE::IDLE); 
    }
    else if (!anEnabledCmd->enabled && mManip->isEnabled())
    {
        mManip->setEnabledState(anEnabledCmd->enabled);
        setNewActiveState(StateMachine::STATE::DISABLED); 
    }
}

void CommandHandler::jointPosWaypointCallback(const arm_idl::msg::JointPositionWaypoint::SharedPtr aMsg)
{
    if(!mManip->isEnabled())
    {
        LOGW << "Manipulator not enabled. Cannot accept joint position waypoint"; 
        return; 
    }
    
    KDL::JntArray jntPos(aMsg->positions.size()); 
    for(int i = 0; i < aMsg->positions.size(); i++)
    {
        jntPos(i) = aMsg->positions[i];
    }

    KDL::JntArray cmdTol(aMsg->tolerances.size()); 
    for(int i = 0; i < aMsg->positions.size(); i++)
    {
        cmdTol(i) = aMsg->tolerances[i];
    }

    JointPositionWaypoint wp; 
    wp.setJointPositionGoal(jntPos); 
    wp.setArrivalTolerance(cmdTol); 

    // TODO: better general purpose interface, eventually want mManip to operate on a IWaypoint* 
    // TODO: compare new goal waypoint to current goal waypoint so we arent spamming the setGoalWaypoint()
    // TODO: maybe some sort of check against current goal, whether new command is same/different, i want to send a goal waypoint
    //       and generate a smooth trajectory between current and goal, dont need constant waypoints from outsider? IDK 
    
    mManip->setGoalWaypoint(std::make_shared<JointPositionWaypoint>(wp)); 
    mStateMachine->setActiveState(StateMachine::STATE::MOVING); 
}

void CommandHandler::taskPosWaypointCallback(const arm_idl::msg::TaskPositionWaypoint::SharedPtr aMsg)
{
    if(!mManip->isEnabled())
    {
        LOGW << "Manipulator not enabled. Cannot accept task position waypoint"; 
        return; 
    }

    geometry_msgs::msg::Quaternion q = aMsg->pose.orientation;
    KDL::Rotation rot = KDL::Rotation::Quaternion(q.x, q.y, q.z, q.w);

    KDL::Vector position(aMsg->pose.position.x, aMsg->pose.position.y, aMsg->pose.position.z);
    KDL::Frame goalPose(rot, position);

    auto wp = std::make_shared<TaskPositionWaypoint>(goalPose, aMsg->tolerance, aMsg->command_frame.data);

    mManip->setTaskGoal(wp); 
    setNewActiveState(StateMachine::STATE::MOVING); 
}

void CommandHandler::commandCallback(const arm_idl::msg::PlanCommand::SharedPtr aCmd)
{
    LOGD << "Receieved plan request for task: " << aCmd->operation_type; 

    std::string saveFilePath = aCmd->object_type; 
    PointCloudHandler::toFile(aCmd->object_type, aCmd->pick_obj_point_cloud_gl); 
    
}

