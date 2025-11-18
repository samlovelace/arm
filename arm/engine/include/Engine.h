#ifndef ENGINE_H
#define ENGINE_H
 
#include "robot_idl/msg/manipulation_command.hpp"
#include "INode.hpp"

#include "IGraspPlanner.hpp"
#include "IArmTaskPlanner.hpp"
class Engine 
{ 
public:
    Engine();
    ~Engine();

    bool init(); 
    void run(); 

private:

    void commandCallback(const robot_idl::msg::ManipulationCommand::SharedPtr aCmd);
    NodePtr mActiveTree; 
    INode::Status mStatus; 

    std::shared_ptr<IGraspPlanner> mGraspPlanner; 
    std::shared_ptr<IArmTaskPlanner> mTaskPlanner; 
   
};
#endif //ENGINE_H