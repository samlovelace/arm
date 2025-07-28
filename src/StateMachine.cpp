
#include "StateMachine.h"
#include <chrono>
#include "plog/Log.h"
#include "Utils.h"
#include "RosTopicManager.hpp"
#include "arm_idl/msg/plan_command.hpp"
#include "arm_idl/msg/plan_response.hpp"

StateMachine::StateMachine(std::shared_ptr<Manipulator> aManip, std::shared_ptr<IArmTaskPlanner> aPlanner) : 
            mRate(std::make_unique<RateController>(50)), mActiveState(STATE::DISABLED), 
            mManipulator(aManip), mPlanner(aPlanner)
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
            case StateMachine::STATE::DISABLED: 
                // arm is not controlled
                break; 
            case StateMachine::STATE::IDLE:
                // arm is active and trying to maintain last goal jnt pos
                break;
            case StateMachine::STATE::MOVING: 
                // arm is currently moving to new goal jnt pos
                if(mManipulator->isArrived())
                {
                    LOGD << "Waypoint arrived"; 
                    setActiveState(StateMachine::STATE::IDLE); 
                }
                break; 
            case StateMachine::STATE::PLANNING: 
                
                if(mPlanner->isPlanFound())
                {
                    LOGD << "Plan found!"; 
                    
                    // TODO: update ComandHandler to do this 
                    // get the plan and publish 
                    auto plans = mPlanner->getCurrentPlans(); 
                    auto planResponse = utils::toIdl(plans); 

                    RosTopicManager::getInstance()->publishMessage<arm_idl::msg::PlanResponse>("arm/response", planResponse);
                    setActiveState(StateMachine::STATE::IDLE); 
                }
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
    case StateMachine::STATE::DISABLED: 
        stringToReturn = "DISABLED"; 
        break; 
    case StateMachine::STATE::IDLE:
        stringToReturn = "IDLE"; 
        break;
    case StateMachine::STATE::MOVING: 
        stringToReturn = "MOVING";
        break;
    case StateMachine::STATE::PLANNING: 
        stringToReturn = "PLANNING"; 
        break; 
    default:
        stringToReturn = "UNKNOWN";
    }

    return stringToReturn; 
}
