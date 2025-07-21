#ifndef IARMTASKPLANNER_HPP
#define IARMTASKPLANNER_HPP
 
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Manipulator.h"
#include "IGraspPlanner.hpp"

class IArmTaskPlanner 
{ 
public:
    virtual ~IArmTaskPlanner() = default; 
    virtual bool planPick(const Eigen::Vector3d& aCentroid_G, pcl::PointCloud<pcl::PointXYZ>::Ptr aCloud_G) = 0; 
    virtual bool planPlace(const Eigen::Vector3d& aPlacePos_G) = 0; 

    void setGraspPlanner(std::shared_ptr<IGraspPlanner> aGraspPlanner) { mGraspPlanner = aGraspPlanner; }
    void setManipulator(std::shared_ptr<Manipulator> aManip) {mManip = aManip; }

protected:
    std::shared_ptr<IGraspPlanner> mGraspPlanner; 
    std::shared_ptr<Manipulator> mManip; 
};
#endif //IARMTASKPLANNER_HPP