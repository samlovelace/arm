#ifndef STATEMACHINE_H
#define STATEMACHINE_H
 
#include "RateController.h"
#include "Manipulator.h"
#include <memory>
#include <mutex> 
 
class StateMachine 
{ 
public:
    StateMachine(std::shared_ptr<Manipulator> aManip);
    ~StateMachine();

    enum class STATE
    {
        DISABLED,
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

    std::shared_ptr<Manipulator> mManipulator; 

    std::unique_ptr<RateController> mRate; 



    std::string toString(StateMachine::STATE aState);
   
};
#endif //STATEMACHINE_H