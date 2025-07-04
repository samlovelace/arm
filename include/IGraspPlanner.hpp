#ifndef IGRASPPLANNER_HPP
#define IGRASPPLANNER_HPP
 
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
 
class IGraspPlanner 
{ 
public:
    virtual ~IGraspPlanner() = default; 
    virtual bool plan(pcl::PointCloud<pcl::PointXYZ>::Ptr aCloud, Eigen::Affine3d& aT_G_ee) = 0; 

private:
   
};
#endif //IGRASPPLANNER_HPP