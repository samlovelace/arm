#ifndef IARMTASKPLANNER_HPP
#define IARMTASKPLANNER_HPP
 
#include <string>  

class IArmTaskPlanner 
{ 
public:
    virtual ~IArmTaskPlanner() = default; 
    virtual bool plan(const std::string& aTaskType) = 0; 

private:
   
};
#endif //IARMTASKPLANNER_HPP