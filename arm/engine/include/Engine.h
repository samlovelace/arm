#ifndef ENGINE_H
#define ENGINE_H

#include <mutex>

#include "robot_idl/msg/manipulation_command.hpp"
#include "INode.hpp"

#include "IGraspPlanner.hpp"
#include "IArmTaskPlanner.hpp"
#include "TrajectoryPlanner.h"
#include "common/KinematicsHandler.h"

class Engine 
{ 
public:
    Engine();
    ~Engine();

    bool init(); 
    void run(); 

private:
    INode::Status tickActiveTree(); 
    void commandCallback(const robot_idl::msg::ManipulationCommand::SharedPtr aCmd);
    
private: 
    NodePtr mActiveTree; 
    std::mutex mTreeMutex; 
    INode::Status mStatus;

    std::shared_ptr<IGraspPlanner> mGraspPlanner; 
    std::shared_ptr<IArmTaskPlanner> mTaskPlanner; 
    std::shared_ptr<KinematicsHandler> mKinematicsHandler; 
    std::shared_ptr<TrajectoryPlanner> mTrajectoryPlanner; 
   
};
#endif //ENGINE_H