#ifndef COMMANDHANDLER_H
#define COMMANDHANDLER_H
 
#include "StateMachine.h" 
#include <std_msgs/msg/float64.hpp> 

class CommandHandler 
{ 
public:
    CommandHandler(StateMachine* msm);
    ~CommandHandler();

private:

    void commandCallback(const std_msgs::msg::Float64::SharedPtr aMsg);

    StateMachine* mStateMachine; 
   
};
#endif //COMMANDHANDLER_H