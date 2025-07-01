
#include "MobileArmTaskPlanner.h"
#include "plog/Log.h"

MobileArmTaskPlanner::MobileArmTaskPlanner()
{

}

MobileArmTaskPlanner::~MobileArmTaskPlanner()
{

}

bool MobileArmTaskPlanner::planPick(const Eigen::Vector3d& aCentroid_G, pcl::PointCloud<pcl::PointXYZ>::Ptr aCloud)
{
    LOGD << "MobileArmTaskPLanner planning for pick task"; 
    return true; 
}

bool MobileArmTaskPlanner::planPlace(const Eigen::Vector3d& aPlacePos_G)
{
    LOGD << "MobileArmTaskPlanner planning for place task"; 
    return true; 
} 


