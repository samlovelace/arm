#ifndef ENGINE_H
#define ENGINE_H
 
#include "robot_idl/msg/manipulation_command.hpp"
 
class Engine 
{ 
public:
    Engine();
    ~Engine();

    bool init(); 
    void run(); 

private:

    void commandCallback(const robot_idl::msg::ManipulationCommand::SharedPtr aCmd);
   
};
#endif //ENGINE_H