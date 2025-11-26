#ifndef SENDWAYPOINTNODE_H
#define SENDWAYPOINTNODE_H

#include "INode.hpp"
#include <kdl/jntarray.hpp> 

class SendWaypointNode : public INode
{
public:
    SendWaypointNode(std::shared_ptr<KDL::JntArray> aCurrentWp);
    ~SendWaypointNode() override; 

    Status tick() override; 


private:
    std::shared_ptr<KDL::JntArray> mCurrentWp; 

};
#endif
