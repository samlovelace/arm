#ifndef PLANNERFACTORY_H
#define PLANNERFACTORY_H
 
#include "IArmTaskPlanner.hpp"
#include <memory>  

class PlannerFactory 
{ 
public:
    PlannerFactory();
    ~PlannerFactory();

    static std::shared_ptr<IArmTaskPlanner> createArmTaskPlanner(const std::string& aPlannerType); 
    static std::shared_ptr<IGraspPlanner> createGraspPlanner(const std::string& aPlannerType); 

private:
   
};
#endif //PLANNERFACTORY_H