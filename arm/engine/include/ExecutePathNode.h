#ifndef EXECUTEPATHNODE_H
#define EXECUTEPATHNODE_H
 
#include <memory> 
#include "INode.hpp"
#include "SequenceNode.h"
#include "kdl/jntarray.hpp"
#include "Context.hpp"

class ExecutePathNode : public INode
{
public:
    ExecutePathNode(std::shared_ptr<PickContext> aPath, 
                    std::shared_ptr<KDL::JntArray> aCurrentWp, 
                    std::shared_ptr<SequenceNode> aChildNode);

    ~ExecutePathNode() override; 

    Status tick() override; 

private:

    std::shared_ptr<SequenceNode> mChildNode;  
    std::shared_ptr<PickContext> mPickContext; 
    std::shared_ptr<KDL::JntArray> mCurrentWp; 

    bool mIsWpSent; 
};
#endif