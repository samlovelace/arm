
#include "ExecutePathNode.h"
#include "plog/Log.h"

ExecutePathNode::ExecutePathNode(std::shared_ptr<PickContext> aPickContext, 
                                 std::shared_ptr<KDL::JntArray> aCurrentWaypoint, 
                                 std::shared_ptr<SequenceNode> aChildNode) : 
    mPickContext(aPickContext), mIsWpSent(false), mCurrentWp(aCurrentWaypoint), mChildNode(aChildNode)
{

}

ExecutePathNode::~ExecutePathNode()
{

}

INode::Status ExecutePathNode::tick()
{
    if(mPickContext->mPath.empty())
    {
        LOGV << "Path empty, assuming success"; 
        return INode::Status::SUCCESS; 
    }

    if(!mIsWpSent)
    {   
        *mCurrentWp = mPickContext->mPath.front(); 
        LOGV << "[ExecutePathNode] Updating currentWp to " << mCurrentWp->data; 
        mIsWpSent = true; 
    }
    
    Status childStatus = mChildNode->tick();

    if(INode::Status::RUNNING == childStatus)
        return childStatus; 
    if(INode::Status::FAILURE == childStatus)
        return childStatus; 

    // pop waypoint from queue 
    mPickContext->mPath.pop_front();
    mIsWpSent = false; 
    mChildNode->reset(); 
    return INode::Status::RUNNING; 
}