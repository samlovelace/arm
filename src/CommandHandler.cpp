
#include "CommandHandler.h"
#include "RosTopicManager.hpp"
#include "plog/Log.h"
#include <kdl/jntarray.hpp>
#include "PointCloudHandler.h"
#include "Utils.h"

#include "JointPositionWaypoint.h"
#include "TaskPositionWaypoint.h"
#include "TaskVelocityWaypoint.h"
#include "JointVelocityWaypoint.h"

CommandHandler::CommandHandler(std::shared_ptr<StateMachine> msm, std::shared_ptr<Manipulator> manip, std::shared_ptr<IArmTaskPlanner> planner) : 
    mStateMachine(msm), mManip(manip), mArmTaskPlanner(planner)
{
    auto topicManager = RosTopicManager::getInstance(); 
    topicManager->createSubscriber<robot_idl::msg::JointPositionWaypoint>("arm/joint_position_waypoint", 
                                                                          std::bind(&CommandHandler::jointPosWaypointCallback, 
                                                                                    this, 
                                                                                    std::placeholders::_1)); 

    topicManager->createSubscriber<robot_idl::msg::Enable>("arm/enable", 
                                                           std::bind(&CommandHandler::enableCallback, 
                                                                     this, 
                                                                     std::placeholders::_1)); 

    topicManager->createSubscriber<robot_idl::msg::TaskPositionWaypoint>("arm/task_position_waypoint", 
                                                                         std::bind(&CommandHandler::taskPosWaypointCallback, 
                                                                                   this, 
                                                                                   std::placeholders::_1)); 

    topicManager->createSubscriber<robot_idl::msg::TaskVelocityWaypoint>("arm/task_velocity_waypoint", 
                                                                         std::bind(&CommandHandler::taskVelWaypointCallback, 
                                                                                  this, 
                                                                                  std::placeholders::_1));
    topicManager->createSubscriber<robot_idl::msg::JointVelocityWaypoint>("arm/joint_velocity_waypoint", 
                                                                         std::bind(&CommandHandler::jointVelWaypointCallback, 
                                                                                    this, 
                                                                                    std::placeholders::_1));

    topicManager->createSubscriber<robot_idl::msg::PlanCommand>("arm/command", 
                                                          std::bind(&CommandHandler::commandCallback, 
                                                                    this, 
                                                                    std::placeholders::_1)); 


    // TODO: is this the best place to put this? Idk where else to put it 
    topicManager->createPublisher<robot_idl::msg::PlanResponse>("arm/response"); 

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

void CommandHandler::enableCallback(const robot_idl::msg::Enable::SharedPtr anEnabledCmd)
{ 
    // determine how to transition the state machine 
    if(anEnabledCmd->enabled && !mManip->isEnabled())
    {
        LOGV << "Recieved enable command"; 
        mManip->setEnabledState(anEnabledCmd->enabled); 
        mManip->startControl(); 
        setNewActiveState(StateMachine::STATE::IDLE); 
    }
    else if (!anEnabledCmd->enabled && mManip->isEnabled())
    {
        LOGV << "Recieved disable command"; 
        mManip->setEnabledState(anEnabledCmd->enabled);
        setNewActiveState(StateMachine::STATE::DISABLED); 
    }
}

void CommandHandler::jointPosWaypointCallback(const robot_idl::msg::JointPositionWaypoint::SharedPtr aMsg)
{
    int numJoints = mManip->getKinematicsHandler()->getNrJoints(); 
    int goalSize = aMsg->positions.size(); 

    handleWaypoint<robot_idl::msg::JointPositionWaypoint, 
                   JointPositionWaypoint>(aMsg, goalSize, numJoints, [](const auto& m) 
    {
        return JointPositionWaypoint(utils::toJntArray(m->positions), utils::toJntArray(m->tolerances));
    });
}

void CommandHandler::jointVelWaypointCallback(const robot_idl::msg::JointVelocityWaypoint::SharedPtr aMsg)
{
    int numJoints = mManip->getKinematicsHandler()->getNrJoints(); 
    int goalSize = aMsg->velocities.size();

    handleWaypoint<robot_idl::msg::JointVelocityWaypoint, JointVelocityWaypoint>(aMsg, goalSize, numJoints,
         [](const auto& m) 
    {
        return JointVelocityWaypoint(utils::toJntArray(m->velocities), utils::toJntArray(m->tolerances));
    });
}

void CommandHandler::taskPosWaypointCallback(const robot_idl::msg::TaskPositionWaypoint::SharedPtr aMsg)
{

    handleWaypoint<robot_idl::msg::TaskPositionWaypoint, TaskPositionWaypoint>(aMsg, -1, -1, 
        [](const auto& wp)
    {
        return TaskPositionWaypoint(utils::toFrame(wp->pose), utils::toArray6(wp->tolerance));
    });
}

void CommandHandler::taskVelWaypointCallback(const robot_idl::msg::TaskVelocityWaypoint::SharedPtr aMsg)
{
    handleWaypoint<robot_idl::msg::TaskVelocityWaypoint, TaskVelocityWaypoint>(aMsg, -1, -1, 
        [](const auto& wp)
    {
        return TaskVelocityWaypoint(utils::toTwist(wp->goal), utils::toTwist(wp->tolerance));
    });
}

void CommandHandler::commandCallback(const robot_idl::msg::PlanCommand::SharedPtr aCmd)
{
    std::string task = aCmd->operation_type == robot_idl::msg::PlanCommand::PICK ? "pick" : "place"; 
    LOGD << "Receieved plan request for task: " << task; 

    if(!mManip->isEnabled())
    {
        LOGW << "Manipulator not enabled. Cannot accept planning command"; 
        return; 
    }

    if(!mArmTaskPlanner->init())
    {
        LOGE << "Failed to initiliaze task planner"; 
        return; 
    }
     
    setNewActiveState(StateMachine::STATE::PLANNING); 

    // TODO: improve location where these are saved
    std::string saveFilePath = "/home/sam/testing/test.ply"; 
    PointCloudHandler::toFile(saveFilePath, aCmd->pick_obj_point_cloud_gl); 

    geometry_msgs::msg::Point pt = aCmd->pick_obj_centroid_gl; 
    Eigen::Vector3d centroid_gl(pt.x, pt.y, pt.z);  

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudHandler::toPCL<pcl::PointXYZ>(aCmd->pick_obj_point_cloud_gl, *cloud); 

    // TODO: probably need to respond somehow 
    if(cloud->empty())
    {
        LOGE << "Cannot plan on empty object cloud"; 
        return; 
    }
 
    //TODO: dispatch to proper pick/place plan function
    mArmTaskPlanner->planPick(centroid_gl, cloud);
}

