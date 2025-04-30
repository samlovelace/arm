#ifndef STATEMACHINE_H
#define STATEMACHINE_H
 
#include "RateController.h"
#include <memory>
#include <mutex> 
 
class StateMachine 
{ 
public:
    StateMachine();
    ~StateMachine();

    enum class STATE
    {
        IDLE, 
        MOVING,
        NUM_TYPES
    }; 

    void run(); 

    STATE getActiveState(); 
    void setActiveState(StateMachine::STATE aState);

private:

    STATE mActiveState;
    std::mutex mActiveStateMutex;  

    std::unique_ptr<RateController> mRate; 



    std::string toString(StateMachine::STATE aState);
   
};
#endif //STATEMACHINE_H