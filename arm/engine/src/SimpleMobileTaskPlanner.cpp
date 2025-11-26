
#include "SimpleMobileTaskPlanner.h"
#include "plog/Log.h"
#include "common/Utils.hpp"

SimpleMobileTaskPlanner::SimpleMobileTaskPlanner(std::shared_ptr<KinematicsHandler> aKine, const KDL::Frame& aT_R_B, std::shared_ptr<TrajectoryPlanner> aTrajPlanner) : 
    mT_R_B(aT_R_B), mTrajectoryPlanner(aTrajPlanner) 
{
    mKinematicsHandler = aKine; 
}

bool SimpleMobileTaskPlanner::init()
{
    LOGV << "Simple init!"; 
    return true; 
}

bool SimpleMobileTaskPlanner::planPick(std::shared_ptr<PickContext> aPickContext)
{
    LOGV << "Planning pick task with simple mobile planner"; 
    
    int numCandidates = 15; 
    int numDistances = 10; 
    double distStep = 0.05; 
    double maxAngle = M_PI; // max polar angle 
    double angleStep_rad = maxAngle / numCandidates; 
    LOGV << "Generating " << numCandidates*numDistances << " candidates with step size " << angleStep_rad << " rad"; 
    double r = 0.25; // radius of circle centered at pick object centroid, TODO: get from config or make arm specific 

    std::vector<BaseCandidate> candidates; 
    
    // TODO: get from somewhere else 
    KDL::JntArray initJnts(mKinematicsHandler->getNrJoints()); 
    for(int i = 0; i < mKinematicsHandler->getNrJoints(); i++)
    {
        initJnts(i) = 1; // radians 
    }

    KDL::Frame T_G_O = aPickContext->mT_G_O;      
    KDL::Frame T_G_ee = utils::affineToKDLFrame(aPickContext->mT_G_ee); 

    LOGV << "Object global pose (x, y, z): " << T_G_O.p.x() << "," << T_G_O.p.y() << "," << T_G_O.p.z();  
    int numCandidate = 0; 

    for(int j = 0; j < numDistances; j++)
    {
        // check further distances from object 
        r += distStep; 

        for (int i = 0; i < numCandidates; i++)
        {
            numCandidate++; 
            double theta = -maxAngle/2 + i * angleStep_rad; // theta = 0 means robot is directly “in front” of object

            double x = r * cos(theta);
            double y = r * sin(theta);

            // Robot pose relative to object frame 
            auto p = KDL::Vector(x, y, 0); // TODO: what to set for z axis?? 

            // Robot orientation: face the object
            float yaw = theta + M_PI;  // +π makes the robot point inward toward the object

            KDL::Vector axis(0, 0, 1); 
            KDL::Rotation R = KDL::Rotation::Rot(axis, yaw); 
            KDL::Frame T_O_R(R, p); 

            // Now convert to global frame
            KDL::Frame T_G_R = T_G_O * T_O_R;

            LOGV << "Candidate " << numCandidate << " = " << T_G_R.p.x() << "," << T_G_R.p.y() << "," << T_G_R.p.z();
        
            // ee pose relative to robot
            KDL::Frame T_R_ee = T_G_R.Inverse() * T_G_ee;

            // ee pose relative to arm base for IK solving 
            KDL::Frame T_B_ee = mT_R_B.Inverse() * T_R_ee; 
            KDL::JntArray finalJnts(mKinematicsHandler->getNrJoints()); 

            if(!mKinematicsHandler->solveIK(initJnts, T_B_ee, finalJnts))
            {
                LOGV << "Failed to solve IK for candidate..."; 
                continue;
            }

            std::vector<KDL::JntArray> path; 
            if(!mTrajectoryPlanner->plan(initJnts, finalJnts, path))
            {
                LOGV << "Rejecting candidate for infeasible trajectory"; 
                continue; 
            }

            LOGV << "Found feasible trajectory with " << path.size() << " waypoints"; 
            double manipulability = mKinematicsHandler->computeManipulability(finalJnts); 

            BaseCandidate cand; 
            cand.T_g_base = T_G_R; 
            cand.jntAnglesFromIK = finalJnts; 
            cand.manipulability = manipulability; 
            cand.mPath = path;

            candidates.push_back(cand); 
            LOGI << "Found feasible base pose candidate!"; 
        }
    }

    if (candidates.empty())
    {
        LOGW << "No feasible base candidates found"; 
        return false;
    }

    LOGD << "Found " << candidates.size() << " mobile base pose candidates!"; 

    std::sort(candidates.begin(), candidates.end(),
              [](const BaseCandidate& a, const BaseCandidate& b){ return a.manipulability > b.manipulability; });

    const BaseCandidate& best = candidates.front();
    LOGD << "Best score: " << best.manipulability;
    LOGD << "Desired vehicle pose (global):";
    utils::logFrame(best.T_g_base);
    LOGV << "Goal State: " << best.jntAnglesFromIK.data; 

    std::deque<KDL::JntArray> path; 
    for(int i = 0; i < best.mPath.size(); i++)
    {
        LOGV << "Wp " << i << ": " << best.mPath[i].data;
        path.push_back(best.mPath[i]); 
    }

    aPickContext->mPath = path; 
    aPickContext->mT_G_R = best.T_g_base;
    
    return true;
}
