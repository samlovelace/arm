#ifndef MOBILEARMTASKPLANNER_H
#define MOBILEARMTASKPLANNER_H
 
#include "IArmTaskPlanner.hpp" 
#include <vector> 

class MobileArmTaskPlanner : public IArmTaskPlanner
{ 
public:
    MobileArmTaskPlanner();
    ~MobileArmTaskPlanner() override; 

    bool init() override; 
    bool planPick(const Eigen::Vector3d& aCentroid_G, pcl::PointCloud<pcl::PointXYZ>::Ptr aCloud) override; 
    bool planPlace(const Eigen::Vector3d& aPlacePos_G) override; 

private:

    std::vector<IrmEntry> mInverseReachMap; 

    struct BaseCandidate
    {
        KDL::Frame T_g_base; 
        KDL::JntArray jntAnglesFromIK; // will this be needed later on? 
        float score; 
    };

    std::vector<BaseCandidate> mBaseCandidates; 
    KDL::Frame mT_vehicle_base;
   
};
#endif //MOBILEARMTASKPLANNER_H