
#include "MovingNode.h"
#include "common/RosTopicManager.hpp"
#include "plog/Log.h"

MovingNode::MovingNode()
{
    mTopicBuffer = std::make_shared<TopicBuffer<ptera_msgs::msg::ManipulatorState>>();

    RosTopicManager::getInstance()->createSubscriber<ptera_msgs::msg::ManipulatorState>(
        "/arm/state",
        [this](ptera_msgs::msg::ManipulatorState::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(mTopicBuffer->mMtx);
            mTopicBuffer->mLastMsg = *msg;
            mTopicBuffer->mHasMsg = true;
        }
    );
}

MovingNode::~MovingNode()
{

}

INode::Status MovingNode::tick()
{
    if(mTopicBuffer->mHasMsg)
    {   
        auto arrived = ptera_msgs::msg::ManipulatorState::ARRIVED; 
        if(arrived == mTopicBuffer->mLastMsg.state)
        {
            LOGV << "Waypoint arrived..."; 
            return INode::Status::SUCCESS; 
        }

        // TODO: add timeout and fail if arrival not reached in time
    }
    
    return INode::Status::RUNNING; 
}