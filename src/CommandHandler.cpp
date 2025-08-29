
#include "CommandHandler.h"
#include "RosTopicManager.hpp"
#include "plog/Log.h"
#include <kdl/jntarray.hpp>
#include "PointCloudHandler.h"
#include "Utils.h"

#include "JointPositionWaypoint.h"
#include "TaskPositionWaypoint.h"

CommandHandler::CommandHandler(std::shared_ptr<StateMachine> msm, std::shared_ptr<Manipulator> manip, std::shared_ptr<IArmTaskPlanner> planner) : 
    mStateMachine(msm), mManip(manip), mArmTaskPlanner(planner), mJointWaypointRcvd(false)
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

    JointPositionWaypoint wp(jntPos, cmdTol);

    // TODO: compare new goal waypoint to current goal waypoint so we arent spamming the setGoalWaypoint()
    // TODO: maybe some sort of check against current goal, whether new command is same/different, i want to send a goal waypoint
    //       and generate a smooth trajectory between current and goal, dont need constant waypoints from outsider? IDK 
    
    mManip->setGoalWaypoint(std::make_shared<JointPositionWaypoint>(wp)); 
    mStateMachine->setActiveState(StateMachine::STATE::MOVING); 
}

void CommandHandler::taskPosWaypointCallback(const robot_idl::msg::TaskPositionWaypoint::SharedPtr aMsg)
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

    //auto wp = TaskPositionWaypoint>(goalPose, utils::toArray6(aMsg->tolerance), aMsg->command_frame.data);
    auto wp = TaskPositionWaypoint(goalPose, utils::toArray6(aMsg->tolerance)); 

    mManip->setTaskGoal(std::make_shared<TaskPositionWaypoint>(wp)); 
    setNewActiveState(StateMachine::STATE::MOVING); 
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

