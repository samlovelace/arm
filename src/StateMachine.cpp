
#include "StateMachine.h"
#include <chrono>
#include "plog/Log.h"

StateMachine::StateMachine() : mRate(std::make_unique<RateController>(50)), mActiveState(STATE::IDLE)
{

}

StateMachine::~StateMachine()
{

}

void StateMachine::run()
{
    LOGD << "State Machine starting in " << toString(mActiveState);

    while(true)
    {
        mRate->start(); 
        
        switch (mActiveState)
        {
            case StateMachine::STATE::IDLE:
                /* code */
                break;
            case StateMachine::STATE::MOVING: 
                //mManip->computeTrajectory(goalJntPos); 
                //mManip->executeTrajectory(); 
                break; 
            
            default:
                break;
        }

        mRate->block(); 
    }
}

StateMachine::STATE StateMachine::getActiveState()
{
    std::lock_guard<std::mutex> lock(mActiveStateMutex); 
    return mActiveState; 
}

void StateMachine::setActiveState(StateMachine::STATE aState)
{
    LOGD << "Switching state machine from " << toString(getActiveState()).c_str() << " to " 
                                                    << toString(aState).c_str();

    std::lock_guard<std::mutex> lock(mActiveStateMutex); 
    mActiveState = aState;    
}

std::string StateMachine::toString(StateMachine::STATE aState)
{
    std::string stringToReturn = ""; 
    switch (aState)
    {
    case StateMachine::STATE::IDLE:
        stringToReturn = "IDLE"; 
        break;
    case StateMachine::STATE::MOVING: 
        stringToReturn = "MOVING";
        break;
    default:
        break;
    }

    return stringToReturn; 
}
