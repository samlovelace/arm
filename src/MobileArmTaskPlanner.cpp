
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

    mT_vehicle_base = mManip->getBaseInVehicleFrame(); 
    mPlanFound = false; 
    mPlans.clear(); // i may regret this in the future 

    return true; 
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

    size_t numCandidates = mInverseReachMap.size(); 
    int candidateNum = 0; 
    
    for(const auto& entry : mInverseReachMap)
    {
        float score = 0.0;

        // base candidate 
        KDL::Frame T_g_base; 
        T_g_base = T_g_ee * entry.T_ee_base; 

        KDL::Frame T_base_ee; 
        T_base_ee = T_g_base.Inverse() * T_g_ee; 

        KDL::Frame newT_g_ee; 
        newT_g_ee = T_g_base * T_base_ee; 

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
        //LOGV << "Added base candidate with score: " << score; 
    }

    LOGD << "Scored all candidates, sorting..."; 

    std::sort(mBaseCandidates.begin(), mBaseCandidates.end(), 
              [](const BaseCandidate& a, const BaseCandidate& b)
    {   
        return a.score > b.score; 
    }); 

    // now sorted, take the first element
    BaseCandidate bestBaseCandidate = mBaseCandidates.front();  
    
    LOGD << "Using candidate with highest score: " << bestBaseCandidate.score; 

    // // ultimately want to determine desired pose of robot
    // // using T_base_vehicle, fixed transform for rigidly mounted arm 
    KDL::Frame T_g_base = bestBaseCandidate.T_g_base; 
    KDL::Frame T_G_V = T_g_base * mT_vehicle_base.Inverse();

    // T_G_V represents the desired global pose of the robot to allow the manip to grasp at T_g_ee

    LOGD << "Planned base pose in global: "; 
    utils::logFrame(T_G_V); 
    LOGD << "Joint Pos: " << bestBaseCandidate.jntAnglesFromIK.data; 

    // Populate the plans vector 
    Plan plan; 
    plan.mPlanType = "pick"; 
    plan.mT_G_V = T_G_V; 
    plan.mGoalJointPos = bestBaseCandidate.jntAnglesFromIK; 
    mPlans.push_back(plan); 

    mPlanFound = true; 
}


bool MobileArmTaskPlanner::planPlace(const Eigen::Vector3d& /*aPlacePos_G*/)
{
    LOGD << "MobileArmTaskPlanner planning for place task"; 
    return true; 
}
