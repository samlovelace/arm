#ifndef CONTEXT_HPP
#define CONTEXT_HPP

#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct PickContext
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr mCloud;
    Eigen::Affine3f mT_G_ee; 
    Eigen::Affine3f mT_G_R; 

    PickContext(pcl::PointCloud<pcl::PointXYZ>::Ptr aCloud) : 
        mCloud(aCloud), 
        mT_G_ee(Eigen::Affine3f::Identity()), 
        mT_G_R(Eigen::Affine3f::Identity()) 
    {
        // do nothing 
    }
};

#endif
