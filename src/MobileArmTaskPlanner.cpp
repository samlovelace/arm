
#include "MobileArmTaskPlanner.h"
#include "plog/Log.h"
#include <thread>
#include <chrono>
#include "Utils.h"

MobileArmTaskPlanner::MobileArmTaskPlanner()
{
}

MobileArmTaskPlanner::~MobileArmTaskPlanner()
{

}

bool MobileArmTaskPlanner::init()
{
    mInverseReachMap = mManip->getInverseReachabilityMap(); 

    if(mInverseReachMap.empty())
    {
        LOGE << "Inverse reachability map not properly configured"; 
        return false; 
    }

    // TODO: get this from somewhere 
    mT_vehicle_base = KDL::Frame::Identity(); 
}

bool MobileArmTaskPlanner::planPick(const Eigen::Vector3d& /*aCentroid_G*/, pcl::PointCloud<pcl::PointXYZ>::Ptr aCloud)
{
    LOGD << "MobileArmTaskPlanner planning for pick task"; 
   
    if (!aCloud || aCloud->empty())
    {
        LOGE << "Input cloud is empty!";
        return false;
    }

    Eigen::Affine3f T_g_ee_temp; 
    mGraspPlanner->plan(aCloud, T_g_ee_temp);
    KDL::Frame T_g_ee = utils::affineToKDLFrame(T_g_ee_temp);
    utils::logFrame(T_g_ee);  

    size_t numCandidates = mInverseReachMap.size(); 
    int candidateNum = 0; 
    
    // TODO: parallelize this sucker
    for(int i = 0; i < 10; i++)
    {
        //LOGD << "Checking base candidate " << candidateNum++ << " of " << numCandidates; 
        IrmEntry entry = mInverseReachMap.at(i); 
        LOGD << "entry T_ee_base: "; 
        utils::logFrame(entry.T_ee_base); 
        float score = 0.0;

        // base candidate 
        KDL::Frame T_g_base; 
        T_g_base = T_g_ee * entry.T_ee_base; 
        LOGD << "T_g_base: "; 
        utils::logFrame(T_g_base); 

        KDL::Frame T_base_ee; 
        //T_base_ee = T_g_base * T_g_ee.Inverse(); 
        T_base_ee = T_g_base.Inverse() * T_g_ee; 

        LOGD << "T_base_ee:"; 
        utils::logFrame(T_base_ee); 

        KDL::Frame newT_g_ee; 
        newT_g_ee = T_g_base * T_base_ee; 
        LOGD << "newT_g_ee:"; 
        utils::logFrame(newT_g_ee); 

        // TODO: should i use the jntAngles from the IRM or current joint pos? 
        KDL::JntArray currentPos = entry.jntAngles;  
        KDL::JntArray resultPos; 
        resultPos.resize(currentPos.rows()); 

        if(!mManip->getKinematicsHandler()->solveIK(currentPos, T_base_ee, resultPos))
        {
            // bad Inverse Kinematics
            LOGV << "Could not solve Inverse Kinematics. Skipping..."; 
            continue; 
        }

        // TODO: add more scoring metrics 
        score += entry.manipulability; 

        BaseCandidate candidate; 
        candidate.T_g_base = T_g_base; 
        candidate.score = score; 
        candidate.jntAnglesFromIK = resultPos; 

        mBaseCandidates.push_back(candidate); 
        LOGV << "Added base candidate with score: " << score; 
    }

    std::sort(mBaseCandidates.begin(), mBaseCandidates.end(), 
              [](const BaseCandidate& a, const BaseCandidate& b)
    {   
        return a.score > b.score; 
    }); 

    // now sorted, take the first element
    BaseCandidate bestBaseCandidate = mBaseCandidates.front();  
    
    // // ultimately want to determine desired pose of robot
    // // using T_base_vehicle, fixed transform for rigidly mounted arm 
    KDL::Frame T_g_base = bestBaseCandidate.T_g_base; 
    KDL::Frame T_G_V = T_g_base * mT_vehicle_base.Inverse();

    // T_G_V represents the desired global pose of the robot to allow the manip to grasp at T_g_ee
}

bool MobileArmTaskPlanner::planPlace(const Eigen::Vector3d& /*aPlacePos_G*/)
{
    LOGD << "MobileArmTaskPlanner planning for place task"; 
    return true; 
}
