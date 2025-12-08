
#include "MoveVehicleNode.h"
#include "common/RosTopicManager.hpp"
#include "robot_idl/msg/vehicle_waypoint.hpp"

MoveVehicleNode::MoveVehicleNode(std::shared_ptr<PickContext> aCtx) : mCtx(aCtx), mSent(false), mMax(100), mCount(0)
{
    RosTopicManager::getInstance()->createPublisher<robot_idl::msg::VehicleWaypoint>("vehicle/waypoint");
}

MoveVehicleNode::~MoveVehicleNode()
{

}

INode::Status MoveVehicleNode::tick()
{
    if(mCount < mMax)
    {
        if(!mSent)
        {
            auto p = mCtx->mT_G_R.p;
            double x, y, z, w;  
            mCtx->mT_G_R.M.GetQuaternion(x, y, z, w);  

            robot_idl::msg::Vec3 pos; 
            pos.set__x(p.x());
            pos.set__y(p.y()); 
            pos.set__z(p.z());   

            robot_idl::msg::Quaternion q; 
            q.set__w(w); 
            q.set__x(x); 
            q.set__y(y); 
            q.set__z(z); 

            robot_idl::msg::VehicleWaypoint wp; 
            wp.set__position(pos);
            wp.set__orientation(q); 

            RosTopicManager::getInstance()->publishMessage("vehicle/waypoint", wp); 
            sleep(1);
            mSent = true; 
        }

        mCount++; 
        return INode::Status::RUNNING;
    }
    else
    {
        return INode::Status::SUCCESS; 
    }

}