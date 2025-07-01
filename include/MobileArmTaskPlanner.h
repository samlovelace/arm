#ifndef MOBILEARMTASKPLANNER_H
#define MOBILEARMTASKPLANNER_H
 
#include "IArmTaskPlanner.hpp" 

class MobileArmTaskPlanner : public IArmTaskPlanner
{ 
public:
    MobileArmTaskPlanner();
    ~MobileArmTaskPlanner() override; 

    bool planPick(const Eigen::Vector3d& aCentroid_G, pcl::PointCloud<pcl::PointXYZ>::Ptr aCloud) override; 
    bool planPlace(const Eigen::Vector3d& aPlacePos_G) override; 

private:
   
};
#endif //MOBILEARMTASKPLANNER_H