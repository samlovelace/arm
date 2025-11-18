#ifndef IARMTASKPLANNER_HPP
#define IARMTASKPLANNER_HPP
 
#include <eigen3/Eigen/Dense>

class IArmTaskPlanner 
{ 
public:
    virtual ~IArmTaskPlanner() = default;  

    virtual bool init() = 0; 
    virtual bool planPick(const Eigen::Affine3f& aT_G_ee, Eigen::Affine3f& aT_G_R) = 0; 

private:
   
};
#endif //IARMTASKPLANNER_HPP