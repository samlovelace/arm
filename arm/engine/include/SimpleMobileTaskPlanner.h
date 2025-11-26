#ifndef SIMPLEMOBILETASKPLANNER_H
#define SIMPLEMOBILETASKPLANNER_H
 
#include "IArmTaskPlanner.hpp"
#include "TrajectoryPlanner.h"
 
class SimpleMobileTaskPlanner : public IArmTaskPlanner
{ 
public:
    SimpleMobileTaskPlanner(std::shared_ptr<KinematicsHandler> aKine, 
                            const KDL::Frame& aT_V_B, 
                            std::shared_ptr<TrajectoryPlanner> aTrajPlanner);

    ~SimpleMobileTaskPlanner() override = default; 

    bool init() override; 
    bool planPick(std::shared_ptr<PickContext> aPickContext) override;
    
    struct BaseCandidate
    {
        KDL::Frame T_g_base; 
        KDL::JntArray jntAnglesFromIK; // will this be needed later on? 
        double manipulability; 
        std::vector<KDL::JntArray> mPath; 
    };

private:
    KDL::Frame mT_R_B;
    std::shared_ptr<TrajectoryPlanner> mTrajectoryPlanner; 
   
};
#endif //SIMPLEMOBILETASKPLANNER_H