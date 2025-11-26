
#include "SendWaypointNode.h"
#include "plog/Log.h"
#include "common/RosTopicManager.hpp"
#include "robot_idl/msg/joint_position_waypoint.hpp"

SendWaypointNode::SendWaypointNode(std::shared_ptr<KDL::JntArray> aCurrentWp) : mCurrentWp(aCurrentWp)
{
    RosTopicManager::getInstance()->createPublisher<robot_idl::msg::JointPositionWaypoint>("arm/joint_position_waypoint");
}

SendWaypointNode::~SendWaypointNode()
{

}

INode::Status SendWaypointNode::tick()
{
    // convert to idl type 
    std::vector<double> pos(mCurrentWp->rows()); 
    std::vector<double> tol(mCurrentWp->rows()); 

    for(int i = 0; i < mCurrentWp->rows(); i++)
    {   
        pos[i] = (*mCurrentWp)(i); 
        tol[i] = 0.05; // hardcoded tolerance because im lazy rn  
    }

    robot_idl::msg::JointPositionWaypoint wp; 
    wp.set__positions(pos); 
    wp.set__tolerances(tol); 

    RosTopicManager::getInstance()->publishMessage("arm/joint_position_waypoint", wp); 
    sleep(2); // sleep briefly to allow arm to start moving
    return INode::Status::SUCCESS; 
}