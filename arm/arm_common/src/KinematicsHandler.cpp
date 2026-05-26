
#include "arm_common/KinematicsHandler.h"
#include "plog/Log.h"
#include <iostream>
#include <eigen3/Eigen/Dense>

KinematicsHandler::KinematicsHandler() :
    mModel(std::make_shared<urdf::Model>()),
    mRobotModel(std::make_shared<urdf::Model>())
{

}

KinematicsHandler::~KinematicsHandler()
{

}

bool KinematicsHandler::init(const ConfigManager::Config& aConfig)
{
    if(!kdl_parser::treeFromFile(aConfig.robotUrdfPath, mTree))
    {
        LOGE << "Failed to parse robot urdf file to tree"; 
        return false; 
    }

    KDL::Tree manip; 
    if(!kdl_parser::treeFromFile(aConfig.urdfPath, manip))
    {
        LOGE << "Failed to parse manipulator urdf to KDL::Tree";
        return false;
    }

    if(!mTree.addTree(manip, aConfig.manipAttachLink))
    {
        LOGE << "Failed to attach manip tree to base at link: " << aConfig.manipAttachLink; 
        return false; 
    }

    mModel->clear();
    if(!mModel->initFile(aConfig.urdfPath))
    {
        LOGE << "Could not parse model from urdf";
        return false;
    }

    mRobotModel->clear(); 
    if(!mRobotModel->initFile(aConfig.robotUrdfPath))
    {
        LOGE << "Failed to parse robot urdf"; 
        return false; 
    }

    const unsigned int nj = mTree.getNrOfJoints();
    LOGV << "Kinematics tree has " << nj << " joints and " 
         << mTree.getNrOfSegments() << " segments";

    KDL::JntArray lower(nj);
    KDL::JntArray upper(nj);
    KDL::JntArray velocity(nj);
    mJointNames.resize(nj);

    for(const auto& [segName, elem] : mTree.getSegments())
    {    
        const KDL::Joint& joint = elem.segment.getJoint();

        // Fixed joints don't consume a q_nr index — skip them
        if(joint.getType() == KDL::Joint::Fixed)
        {
            LOGV << "Skipping fixed joint for segment: " << segName;
            continue;
        }

        const unsigned int idx = elem.q_nr;
        const std::string& jointName = joint.getName();

        if(idx >= nj)
        {
            LOGE << "Joint index " << idx << " out of range for joint: " << jointName;
            return false;
        }

        auto urdfJoint = mModel->getJoint(jointName);
        if(nullptr == urdfJoint)
            urdfJoint = mRobotModel->getJoint(jointName); 

        if(nullptr == urdfJoint)
        {
            LOGW << "Failed to find urdf joint for: " << jointName << " — skipping limits";
            continue;
        }

        if(urdfJoint->type != urdf::Joint::REVOLUTE 
            && urdfJoint->type != urdf::Joint::PRISMATIC)
        {
            LOGV << "Skipping non-revolute/prismatic joint: " << jointName;
            continue;
        }

        if(!urdfJoint->limits)
        {
            LOGW << "Joint " << jointName << " has no limits defined — defaulting to zero";
            continue;
        }

        mJointNames[idx] = jointName;
        lower(idx)   = urdfJoint->limits->lower;
        upper(idx)   = urdfJoint->limits->upper;
        velocity(idx) = urdfJoint->limits->velocity;

        LOGD << "Joint [" << idx << "] " << jointName 
             << "  lower=" << lower(idx) 
             << "  upper=" << upper(idx) 
             << "  vel=" << velocity(idx);
    }

    // Verify no unnamed joints (indicates an index was missed)
    for(unsigned int i = 0; i < nj; ++i)
    {
        if(mJointNames[i].empty())
        {
            LOGW << "Joint index " << i << " has no name — limits will be [0, 0]";
        }
    }

    mLimitsMap.insert({"lower",    lower});
    mLimitsMap.insert({"upper",    upper});
    mLimitsMap.insert({"velocity", velocity});

    KDL::JntArray q_min = getJointLimits("lower");
    KDL::JntArray q_max = getJointLimits("upper");

    mManipEndLink = aConfig.manipEndLink; 
    std::vector<std::string> endpoints = {mManipEndLink}; 

    mFkSolver  = std::make_shared<KDL::TreeFkSolverPos_recursive>(mTree);
    mVelSolver = std::make_shared<KDL::TreeIkSolverVel_wdls>(mTree, endpoints);
    mIkSolver  = std::make_shared<KDL::TreeIkSolverPos_NR_JL>(
        mTree, endpoints, q_min, q_max, *mFkSolver, *mVelSolver, 200, 1e-1);
    mJacobianSolver = std::make_shared<KDL::TreeJntToJacSolver>(mTree);

    Eigen::MatrixXd Wq = Eigen::MatrixXd::Identity(nj, nj);
    mVelSolver->setWeightJS(Wq);
    mVelSolver->setWeightTS(Eigen::MatrixXd::Identity(6 * endpoints.size(), 6 * endpoints.size()));

    LOGD << "KinematicsHandler initialized successfully";
    return true;
}

bool KinematicsHandler::solveIK(const KDL::JntArray& anInitPos, const KDL::Frame& aGoalPose, KDL::JntArray& aResultOut)
{
    LOGW << "Tree size: " << mTree.getNrOfJoints() << " anInitPos size: " << anInitPos.rows();
    // TODO: error checking on JntArray sizes
    KDL::Frames goalFrames; 
    goalFrames[mManipEndLink] = aGoalPose; 

    if(aResultOut.rows() != anInitPos.rows())
    {
        LOGW << "Resizing result array from " << aResultOut.rows() << " to " << anInitPos.rows();
        aResultOut.resize(anInitPos.rows());
    }

    float result = mIkSolver->CartToJnt(anInitPos, goalFrames, aResultOut);

    if(result < 0)
    {
        LOGW << "Failed to solve IK, error code: " << result;
        return false;
    }

    return true;
}

bool KinematicsHandler::solveIK(const KDL::JntArray& anInitPos, const KDL::Twist& aGoalVel, KDL::JntArray& aResultOut)
{
    KDL::Twists goalVels; 
    goalVels[mManipEndLink] = aGoalVel; 

    int result = mVelSolver->CartToJnt(anInitPos, goalVels, aResultOut);

    if(result != 0)
    {
            LOGW << "Failed to solve IK for goal vel. Error code: " << result;
            return false;
    }

    return true;
}

bool KinematicsHandler::solveFk(const KDL::JntArray& anInitPos, KDL::Frame& aFrameOut, const std::string& aSegmentName)
{
    std::string segName = aSegmentName;  
    if(segName.empty())
    {
        segName = mManipEndLink; 
    }

    int fkResult = mFkSolver->JntToCart(anInitPos, aFrameOut, segName);

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
    KDL::JntArray limits(mTree.getNrOfJoints());

    if(mLimitsMap.find(aLimitType) != mLimitsMap.end())
    {
        return mLimitsMap.at(aLimitType);
    }

    return limits;
}

double KinematicsHandler::computeManipulability(const KDL::JntArray& aJntCnfg)
{
    KDL::Jacobian jac(mTree.getNrOfJoints());

    if (mJacobianSolver->JntToJac(aJntCnfg, jac, mManipEndLink) < 0)
    {
        std::cerr << "Error computing Jacobian" << std::endl;
        return -1;
    }

    const int rows = 6;
    const int cols = jac.columns();

    // SAFE COPY
    Eigen::MatrixXd J(rows, cols);
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            J(r, c) = jac(r, c);

    // Compute manipulability
    Eigen::MatrixXd JJt = J * J.transpose();
    double det = JJt.determinant();

    if (det <= 0.0)
        return 0.0;

    return std::sqrt(det);
}