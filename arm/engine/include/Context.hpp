#ifndef CONTEXT_HPP
#define CONTEXT_HPP

#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <kdl/frames.hpp>

struct PickContext
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr mCloud;
    KDL::Frame mT_G_O; // global pose of pick object
    Eigen::Affine3f mT_G_ee; // planned global pose of end-effector at pick
    Eigen::Affine3f mT_G_R; // planning global pose of robot frame at pick 

    PickContext(pcl::PointCloud<pcl::PointXYZ>::Ptr aCloud) : 
        mCloud(aCloud), 
        mT_G_ee(Eigen::Affine3f::Identity()), 
        mT_G_R(Eigen::Affine3f::Identity()), 
        mT_G_O(KDL::Frame::Identity())
    {
        // do nothing 
    }
};

#endif
