#include "MobileArmTaskPlanner.h"
#include "plog/Log.h"

#include <thread>
#include <chrono>

MobileArmTaskPlanner::MobileArmTaskPlanner()
{

}

MobileArmTaskPlanner::~MobileArmTaskPlanner()
{

}

bool MobileArmTaskPlanner::planPick(const Eigen::Vector3d& /*aCentroid_G*/, pcl::PointCloud<pcl::PointXYZ>::Ptr aCloud)
{
    LOGD << "MobileArmTaskPlanner planning for pick task"; 

    if (!aCloud || aCloud->empty())
    {
        LOGE << "Input cloud is empty!";
        return false;
    }

    Eigen::Affine3f T_g_ee; 
    mGraspPlanner->plan(aCloud, T_g_ee); 
    
    // TODO: maybe not the best way to store? 
    std::vector<Eigen::Affine3f> mInverseReachabilityMap;

    // // Generate base pose candidates 
    // for(const auto& T_ee_base : mInverseReachabilityMap)
    // {
    //     float score = 0.0; 

    //     // base pose candidate
    //     Eigen::Affine3f T_g_base; 
    //     T_g_base = T_g_ee * T_ee_base; 

    //     // get ee pose in base frame 
    //     Eigen::Affine3f T_base_ee; 
    //     T_base_ee = T_g_base * T_g_ee.inverse(); 

    //     KDL::JntArray q; 
    //     if(!solveIK(T_base_ee, q))
    //         continue;

    //     // Compute manipulability
    //     score += manipulability(q); 
    //     score += collision(q); 

    //     // store in data structure convienient for sorting 
    //     // PickPlanCandidate: score, T_base_g, jointAngles, 
    
    // }

    // // sort data structure and find top score base pose 

    // // ultimately want to determine desired pose of robot
    // // using T_base_vehicle, fixed transform for rigidly mounted arm 

    // T_G_V = T_base_G_best * T_base_vehicle.inverse()

}

bool MobileArmTaskPlanner::planPlace(const Eigen::Vector3d& /*aPlacePos_G*/)
{
    LOGD << "MobileArmTaskPlanner planning for place task"; 
    return true; 
}
