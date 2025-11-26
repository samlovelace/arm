
#include "MovingNode.h"
#include "common/RosTopicManager.hpp"

MovingNode::MovingNode()
{
    mTopicBuffer = std::make_shared<TopicBuffer<robot_idl::msg::ManipulatorState>>();

    RosTopicManager::getInstance()->createSubscriber<robot_idl::msg::ManipulatorState>(
        "/arm/state",
        [this](robot_idl::msg::ManipulatorState::SharedPtr msg)
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
        auto arrived = robot_idl::msg::ManipulatorState::ARRIVED; 
        if(arrived == mTopicBuffer->mLastMsg.state)
        {
            return INode::Status::SUCCESS; 
        }

        // TODO: add timeout and fail if arrival not reached in time
    }
    
    return INode::Status::RUNNING; 
}