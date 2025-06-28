
#include "KinematicsHandler.h"
#include "plog/Log.h"
#include "Utils.h"

KinematicsHandler::KinematicsHandler() : mModel(std::make_shared<urdf::Model>())
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

    mModel->clear(); 
    if(!mModel->initFile(anUrdfPath))
    {
        LOGE << "Could not parse model from urdf"; 
        return false; 
    }

    KDL::JntArray q_min = getJointLimits("lower"); 
    KDL::JntArray q_max = getJointLimits("upper"); 

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

KDL::JntArray KinematicsHandler::getJointLimits(const std::string& aLimitType)
{
    int jntNum = 0; 
    KDL::JntArray limits(mChain.getNrOfJoints()); 

    for(const auto& joint : mModel->joints_)
    {
        if(joint.second->limits)
        {
            if("effort" == aLimitType)
            {
                limits(jntNum) = joint.second->limits->effort; 
            }
            else if("velocity" == aLimitType)
            {
                limits(jntNum) = joint.second->limits->velocity; 
            }
            else if("upper" == aLimitType)
            {
                limits(jntNum) = joint.second->limits->upper; 
            }
            else if("lower" == aLimitType)
            {
                limits(jntNum) = joint.second->limits->lower; 
            }
        
            jntNum++; 
        }

    }

    return limits; 
}

