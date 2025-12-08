#ifndef MOVEVEHICLENODE_H
#define MOVEVEHICLENODE_H

#include "INode.hpp"
#include "Context.hpp"

class MoveVehicleNode : public INode
{
public:
    MoveVehicleNode(std::shared_ptr<PickContext> aCtx);
    ~MoveVehicleNode() override; 

    Status tick() override; 

private: 

    std::shared_ptr<PickContext> mCtx; 

    int mMax; 
    bool mSent; 
    int mCount; 

};
#endif 
