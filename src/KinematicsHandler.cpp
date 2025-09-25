
#include "KinematicsHandler.h"
#include "plog/Log.h"
#include <iostream> 
#include <eigen3/Eigen/Dense>

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

    // TODO: this will break with any other arm that doesnt have base_link or tool0
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

    KDL::JntArray lower(mChain.getNrOfJoints()); 
    KDL::JntArray upper(mChain.getNrOfJoints()); 
    int idx = 0; 

    // TODO: add parsing and saving of effort and velocity limits 
    for(size_t idx = 0; idx < mChain.getNrOfJoints(); idx++)
    {
        KDL::Segment segment = mChain.getSegment(idx); 
        std::string jointName = segment.getJoint().getName(); 
        mJointNames.push_back(jointName); 
        
        auto urdfJoint = mModel->getJoint(jointName); 

        lower(idx) = urdfJoint->limits->lower; 
        upper(idx) = urdfJoint->limits->upper;  
    }

    mLimitsMap.insert({"lower", lower}); 
    mLimitsMap.insert({"upper", upper}); 

    KDL::JntArray q_min = getJointLimits("lower"); 
    KDL::JntArray q_max = getJointLimits("upper"); 

    mFkSolver = std::make_shared<KDL::ChainFkSolverPos_recursive>(mChain);
    mVelSolver = std::make_shared<KDL::ChainIkSolverVel_pinv>(mChain);
    mIkSolver = std::make_shared<KDL::ChainIkSolverPos_NR_JL>(mChain, q_min, q_max, *mFkSolver, *mVelSolver, 200, 1e-1);
    mJacobianSolver = std::make_shared<KDL::ChainJntToJacSolver>(mChain); 

    LOGD << "KinematicsHandler initialized successfully"; 
    return true; 
}

bool KinematicsHandler::solveIK(const KDL::JntArray& anInitPos, const KDL::Frame& aGoalPose, KDL::JntArray& aResultOut)
{
    LOGW << "Chain size: " << mChain.getNrOfJoints() << " anInitPos size: " << anInitPos.rows(); 
    // TODO: error checking on JntArray sizes 
    int result = mIkSolver->CartToJnt(anInitPos, aGoalPose, aResultOut);
    
    // KDL::Frame fkPose;
    // int fkResult = mFkSolver->JntToCart(aResultOut, fkPose);
    // KDL::Frame err = aGoalPose * fkPose.Inverse(); 

    // // Position
    // double x = err.p.x();
    // double y = err.p.y();
    // double z = err.p.z();

    // // Orientation (Quaternion)
    // double qx, qy, qz, qw;
    // err.M.GetQuaternion(qx, qy, qz, qw);

    // LOGD << "Error Frame Position: x=" << x << ", y=" << y << ", z=" << z;
    // //LOGD << "Frame Orientation (quaternion): qw=" << qw
    // //    << ", qx=" << qx << ", qy=" << qy << ", qz=" << qz;

    if(result != 0)
    {
        LOGW << "Failed to solve IK, error code: " << result; 
        return false; 
    }

    return true; 
}

bool KinematicsHandler::solveIK(const KDL::JntArray& anInitPos, const KDL::Twist& aGoalVel, KDL::JntArray& aResultOut)
{
   int result = mVelSolver->CartToJnt(anInitPos, aGoalVel, aResultOut); 

   if(result != 0)
   {    
        LOGW << "Failed to solve IK for goal vel. Error code: " << result; 
        return false; 
   }
   
   return true; 
}

bool KinematicsHandler::solveFk(const KDL::JntArray& anInitPos, KDL::Frame& aFrameOut)
{
    int fkResult = mFkSolver->JntToCart(anInitPos, aFrameOut);
    
    // // Position
    // double x = aFrameOut.p.x();
    // double y = aFrameOut.p.y();
    // double z = aFrameOut.p.z();

    // // Orientation (Quaternion)
    // double qx, qy, qz, qw;
    // aFrameOut.M.GetQuaternion(qx, qy, qz, qw);

    // LOGD << "Frame Position: x=" << x << ", y=" << y << ", z=" << z;
    // LOGD << "Frame Orientation (quaternion): qw=" << qw
    //     << ", qx=" << qx << ", qy=" << qy << ", qz=" << qz; 

    if(0 > fkResult)
    {
        return false; 
    }

    return true; 
}

KDL::JntArray KinematicsHandler::getJointLimits(const std::string& aLimitType)
{
    KDL::JntArray limits(mChain.getNrOfJoints()); 

    if(mLimitsMap.find(aLimitType) != mLimitsMap.end())
    {
        return mLimitsMap.at(aLimitType); 
    }

    return limits; 
}

double KinematicsHandler::computeManipulability(const KDL::JntArray& aJntCfg)
{
    KDL::Jacobian jac(aJntCfg.rows()); 
    
    if(mJacobianSolver->JntToJac(aJntCfg, jac) < 0)
    {
        LOGW << "Failed to compute jacobian"; 
        return 0.0; 
    }

    Eigen::MatrixXd J = jac.data; 
    Eigen::MatrixXd JJt = J * J.transpose(); 
    return std::sqrt(JJt.determinant()); 
}

