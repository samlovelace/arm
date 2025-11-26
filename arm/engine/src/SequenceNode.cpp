
#include "SequenceNode.h"

SequenceNode::SequenceNode(std::vector<NodePtr>& aNodesVec) : mChildNodes(aNodesVec), mActiveIndex(0)
{

}

SequenceNode::~SequenceNode()
{

}

INode::Status SequenceNode::tick() 
{
    while (mActiveIndex < mChildNodes.size()) 
    {
        Status s = mChildNodes[mActiveIndex]->tick();
        
        if(Status::SUCCESS != s)
            return s; 
        
        // if here, current node succeeded, increment to next node
        mActiveIndex++;
    }
        
    // if here, all nodes succeeded
    return Status::SUCCESS;
}

void SequenceNode::reset()
{
    mActiveIndex = 0; 
}