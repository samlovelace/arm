#ifndef PLANPICKTASKNODE_H
#define PLANPICKTASKNODE_H
 
#include "INode.hpp" 
#include "Context.hpp"
#include "IArmTaskPlanner.hpp"

class PlanPickTaskNode : public INode 
{ 
public:
    PlanPickTaskNode(std::shared_ptr<PickContext> aCtx, std::shared_ptr<IArmTaskPlanner> aPlanner);
    ~PlanPickTaskNode() override = default; 

    Status tick() override; 

private:

    std::shared_ptr<PickContext> mContext; 
    std::shared_ptr<IArmTaskPlanner> mPlanner; 
   
};
#endif //PLANPICKTASKNODE_H