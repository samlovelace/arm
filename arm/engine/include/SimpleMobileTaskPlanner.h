#ifndef SIMPLEMOBILETASKPLANNER_H
#define SIMPLEMOBILETASKPLANNER_H
 
#include "IArmTaskPlanner.hpp"
 
class SimpleMobileTaskPlanner : public IArmTaskPlanner
{ 
public:
    SimpleMobileTaskPlanner();
    ~SimpleMobileTaskPlanner() override = default; 

    bool init() override; 
    bool planPick(const Eigen::Affine3f& aT_G_ee, Eigen::Affine3f& aT_G_R) override; 

private:
   
};
#endif //SIMPLEMOBILETASKPLANNER_H