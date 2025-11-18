#ifndef SEQUENCENODE_H
#define SEQUENCENODE_H
 
#include "INode.hpp"
#include <vector> 
 
class SequenceNode : public INode
{ 
public:
    SequenceNode(std::vector<NodePtr>& aNodesVec);
    ~SequenceNode() override; 

    Status tick() override; 

private:
 
    size_t mActiveIndex; 
    std::vector<NodePtr> mChildNodes; 
   
};
#endif //SEQUENCENODE_H