
#include "PlanPickTaskNode.h"

PlanPickTaskNode::PlanPickTaskNode(std::shared_ptr<PickContext> aCtx, std::shared_ptr<IArmTaskPlanner> aPlanner) : 
    mContext(aCtx), mPlanner(aPlanner)
{

}

INode::Status PlanPickTaskNode::tick()
{
    if(!mPlanner->init())
    {
        return Status::FAILURE; 
    }

    if(!mPlanner->planPick(mContext))
    {
        return Status::FAILURE; 
    }

    return Status::SUCCESS; 
}