#ifndef MOBILEARMTASKPLANNER_H
#define MOBILEARMTASKPLANNER_H
 
#include "IArmTaskPlanner.hpp" 

class MobileArmTaskPlanner : public IArmTaskPlanner
{ 
public:
    MobileArmTaskPlanner();
    ~MobileArmTaskPlanner() override; 

    bool plan(const std::string& aTaskType) override; 

private:
   
};
#endif //MOBILEARMTASKPLANNER_H