#ifndef PLANGRASPNODE_H
#define PLANGRASPNODE_H
 
#include "INode.hpp"
#include "Context.hpp"
#include "IGraspPlanner.hpp"
 
class PlanGraspNode : public INode
{ 
public:
    PlanGraspNode(std::shared_ptr<PickContext> aCtx, std::shared_ptr<IGraspPlanner> aGraspPlanner);
    ~PlanGraspNode() override = default; 

    Status tick() override; 

private:

    std::shared_ptr<PickContext> mContext; 
    std::shared_ptr<IGraspPlanner> mGraspPlanner; 
   
};
#endif //PLANGRASPNODE_H