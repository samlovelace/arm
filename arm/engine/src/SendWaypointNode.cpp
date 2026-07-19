
#include "SendWaypointNode.h"
#include "plog/Log.h"
#include "common/RosTopicManager.hpp"
#include "ptera_msgs/msg/joint_position_waypoint.hpp"

SendWaypointNode::SendWaypointNode(std::shared_ptr<KDL::JntArray> aCurrentWp) : mCurrentWp(aCurrentWp)
{
    RosTopicManager::getInstance()->createPublisher<ptera_msgs::msg::JointPositionWaypoint>("arm/joint_position_waypoint");
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

    ptera_msgs::msg::JointPositionWaypoint wp; 
    wp.set__positions(pos); 
    wp.set__tolerances(tol); 

    RosTopicManager::getInstance()->publishMessage("arm/joint_position_waypoint", wp); 
    sleep(1); // sleep briefly to allow arm to start moving
    return INode::Status::SUCCESS; 
}