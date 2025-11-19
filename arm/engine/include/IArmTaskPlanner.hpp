#ifndef IARMTASKPLANNER_HPP
#define IARMTASKPLANNER_HPP
 
#include "Context.hpp"
#include "common/KinematicsHandler.h"

class IArmTaskPlanner 
{ 
public:
    virtual ~IArmTaskPlanner() = default;  

    virtual bool init() = 0; 
    virtual bool planPick(std::shared_ptr<PickContext> aPickContext) = 0; 

protected:

    std::shared_ptr<KinematicsHandler> mKinematicsHandler; 
   
};
#endif //IARMTASKPLANNER_HPP