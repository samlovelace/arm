
#include "PlanGraspNode.h"

PlanGraspNode::PlanGraspNode(std::shared_ptr<PickContext> aCtx, std::shared_ptr<IGraspPlanner> aGraspPlanner) : 
    mContext(aCtx), mGraspPlanner(aGraspPlanner)
{
    
}

INode::Status PlanGraspNode::tick()
{
    if(!mGraspPlanner->plan(mContext->mCloud, mContext->mT_G_ee))
    {
        return Status::FAILURE; 
    }
    
    return Status::SUCCESS; 
}


