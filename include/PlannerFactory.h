#ifndef PLANNERFACTORY_H
#define PLANNERFACTORY_H
 
#include "IArmTaskPlanner.hpp"
#include <memory>  

class PlannerFactory 
{ 
public:
    PlannerFactory();
    ~PlannerFactory();

    static std::shared_ptr<IArmTaskPlanner> create(const std::string& aPlannerType); 

private:
   
};
#endif //PLANNERFACTORY_H