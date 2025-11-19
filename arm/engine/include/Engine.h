#ifndef ENGINE_H
#define ENGINE_H
 
#include "robot_idl/msg/manipulation_command.hpp"
#include "INode.hpp"

#include "IGraspPlanner.hpp"
#include "IArmTaskPlanner.hpp"
#include "common/KinematicsHandler.h"
class Engine 
{ 
public:
    Engine();
    ~Engine();

    bool init(); 
    void run(); 

private:
    void commandCallback(const robot_idl::msg::ManipulationCommand::SharedPtr aCmd);
    
private: 
    NodePtr mActiveTree; 
    INode::Status mStatus; 

    std::shared_ptr<IGraspPlanner> mGraspPlanner; 
    std::shared_ptr<IArmTaskPlanner> mTaskPlanner; 
    std::shared_ptr<KinematicsHandler> mKinematicsHandler; 
   
};
#endif //ENGINE_H