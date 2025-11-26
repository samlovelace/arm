#ifndef MOVINGNODE_H
#define MOVINGNODE_H

#include "INode.hpp"
#include "common/TopicBuffer.hpp"
#include "robot_idl/msg/manipulator_state.hpp"

class MovingNode : public INode
{

public:
    MovingNode();
    ~MovingNode() override; 

    Status tick() override; 

private:
    std::shared_ptr<TopicBuffer<robot_idl::msg::ManipulatorState>> mTopicBuffer;
};
#endif

