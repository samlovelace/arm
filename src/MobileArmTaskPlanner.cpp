
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
    //mInverseReachMap = mManip->getInverseReachabilityMap(); 

    // if(mInverseReachMap.empty())
    // {
    //     LOGE << "Inverse reachability map not properly configured"; 
    //     return false; 
    // }

    mT_vehicle_base = mManip->getBaseInVehicleFrame(); 
    mPlanFound = false; 
    mPlanningFailed = false; 
    mPlans.clear(); // i may regret this in the future 

    return true; 
}

bool MobileArmTaskPlanner::planPick(const Eigen::Vector3d& /*aCentroid_G*/,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr aCloud)
{
    LOGD << "MobileArmTaskPlanner planning for pick task";

    if (!aCloud || aCloud->empty())
    {
        LOGE << "Input cloud is empty!";
        return false;
    }

    // Desired grasp pose in global frame
    Eigen::Affine3f T_g_ee_aff;
    if (!mGraspPlanner->plan(aCloud, T_g_ee_aff)) {
        LOGE << "Grasp planner failed to produce T_g_ee";
        return false;
    }
    KDL::Frame T_g_ee = utils::affineToKDLFrame(T_g_ee_aff);

    const std::vector<IrmEntry>& inverseReachMap = mManip->getInverseReachabilityMap();
    if (inverseReachMap.empty()) {
        LOGE << "IRM is empty";
        return false;
    }

    // Build a valid IK seed (size must match DOF)
    auto kh = mManip->getKinematicsHandler();
    const int dof = kh->getNrJoints(); 
    KDL::JntArray q_min = kh->getJointLimits("lower");
    KDL::JntArray q_max = kh->getJointLimits("upper");

    auto makeMidSeed = [&](KDL::JntArray& q){
        q.resize(dof);
        for (int i = 0; i < dof; ++i)
            q(i) = 0.5 * (q_min(i) + q_max(i));
    };

    mBaseCandidates.clear();
    mBaseCandidates.reserve(std::min<size_t>(inverseReachMap.size(), 50000));

    int candidateNum = 0;
    for (const auto& entry : inverseReachMap)
    {
        // IK target in manip base frame: base->ee
        // IRM stores ee->base, so invert it
        KDL::Frame T_base_ee = entry.T_ee_base.Inverse();

        // Compose base pose candidate in global frame for scoring/output
        KDL::Frame T_g_base = T_g_ee * entry.T_ee_base;
        KDL::JntArray seed;
        makeMidSeed(seed);

        KDL::JntArray resultPos; resultPos.resize(dof);
        const bool ik_ok = kh->solveIK(seed, T_base_ee, resultPos);
        if (!ik_ok) {
            // Could also try a second seed (e.g., q_min or q_max) before skipping
            continue;
        }

        float score = static_cast<float>(entry.manipulability); // add more terms if you want
        BaseCandidate candidate;
        candidate.T_g_base = T_g_base;
        candidate.score = score;
        candidate.jntAnglesFromIK = resultPos;
        mBaseCandidates.push_back(std::move(candidate));

        ++candidateNum;
    }

    if (mBaseCandidates.empty())
    {
        LOGW << "No feasible base candidates found"; 
        mPlanningFailed = true;
        return false;
    }

    std::sort(mBaseCandidates.begin(), mBaseCandidates.end(),
              [](const BaseCandidate& a, const BaseCandidate& b){ return a.score > b.score; });

    const BaseCandidate& best = mBaseCandidates.front();

    // Convert base candidate to desired global vehicle pose:
    // T_G_V = T_G_base * (T_V_base)^-1
    KDL::Frame T_G_V = best.T_g_base * mT_vehicle_base.Inverse();

    LOGD << "Best score: " << best.score;
    LOGD << "Desired vehicle pose (global):";
    utils::logFrame(T_G_V);
    LOGD << "Goal joints: " << best.jntAnglesFromIK.data;

    Plan plan;
    plan.mPlanType = "pick";
    plan.mT_G_V = T_G_V;
    plan.mGoalJointPos = best.jntAnglesFromIK;
    mPlans.push_back(std::move(plan));

    mPlanFound = true;
    return true;
}

bool MobileArmTaskPlanner::planPlace(const Eigen::Vector3d& /*aPlacePos_G*/)
{
    LOGD << "MobileArmTaskPlanner planning for place task"; 
    return true; 
}
