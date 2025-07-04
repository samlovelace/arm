#ifndef IARMTASKPLANNER_HPP
#define IARMTASKPLANNER_HPP
 
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "IGraspPlanner.hpp"

class IArmTaskPlanner 
{ 
public:
    virtual ~IArmTaskPlanner() = default; 
    virtual bool planPick(const Eigen::Vector3d& aCentroid_G, pcl::PointCloud<pcl::PointXYZ>::Ptr aCloud_G) = 0; 
    virtual bool planPlace(const Eigen::Vector3d& aPlacePos_G) = 0; 

    void setGraspPlanner(std::shared_ptr<IGraspPlanner> aGraspPlanner) { mGraspPlanner = aGraspPlanner; }

protected:
    std::shared_ptr<IGraspPlanner> mGraspPlanner; 
   
};
#endif //IARMTASKPLANNER_HPP