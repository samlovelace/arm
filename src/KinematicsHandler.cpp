
#include "KinematicsHandler.h"
#include "plog/Log.h"
#include "Utils.h"

KinematicsHandler::KinematicsHandler()
{

}

KinematicsHandler::~KinematicsHandler()
{

}

bool KinematicsHandler::init(const std::string& anUrdfPath)
{ 
    if(!kdl_parser::treeFromFile(anUrdfPath, mTree))
    {
        LOGE << "Failed to parse urdf to KDL::Tree"; 
        return false; 
    }

    LOGD << "Successfully parsed urdf with " << mTree.getNrOfJoints() << " joints!"; 

    if (!mTree.getChain("base_link", "tool0", mChain)) 
    {
        throw std::runtime_error("Failed to extract KDL chain from tree");
    }

    KDL::JntArray q_min(mChain.getNrOfJoints()); 
    KDL::JntArray q_max(mChain.getNrOfJoints()); 

    urdf::Model mModel; 
    if(!mModel.initFile(anUrdfPath))
    {
        LOGE << "Could not parse model from urdf"; 
        return false; 
    }

    int numJnt = 0; 
    for (const auto& joint : mModel.joints_) 
    {
        if(joint.second->limits)
        {
            q_min(numJnt); 
            q_max(numJnt); 
            numJnt++; 
        }
    }

    mFkSolver = std::make_shared<KDL::ChainFkSolverPos_recursive>(mChain);
    mVelSolver = std::make_shared<KDL::ChainIkSolverVel_pinv>(mChain);
    mIkSolver = std::make_shared<KDL::ChainIkSolverPos_NR_JL>(mChain, q_min, q_max, *mFkSolver, *mVelSolver, 200, 1e-4);

    LOGD << "KinematicsHandler initialized successfully"; 
    return true; 
}

bool KinematicsHandler::solveIK(const KDL::JntArray& anInitPos, const KDL::Frame& aGoalPose, KDL::JntArray& aResultOut)
{
    utils::logFrame(aGoalPose); 

    // TODO: error checking on JntArray sizes 
    int result = mIkSolver->CartToJnt(anInitPos, aGoalPose, aResultOut);

    KDL::Frame fkPose;
    int fkResult = mFkSolver->JntToCart(aResultOut, fkPose);

    KDL::Frame err = aGoalPose * fkPose.Inverse(); 

    LOGW << "Task Error: ";
    utils::logFrame(err); 

}

