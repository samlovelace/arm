#ifndef GRASPPLANNER
#define GRASPPLANNER

#include "IGraspPlanner.hpp"
 
class PcaGraspPlanner : public IGraspPlanner
{ 
public:
    PcaGraspPlanner();
    ~PcaGraspPlanner();

    bool plan(pcl::PointCloud<pcl::PointXYZ>::Ptr aCloud, Eigen::Affine3d& aT_G_ee) override; 

private:

   
};
#endif //GRASPPLANNER