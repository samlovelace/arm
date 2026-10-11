
#include "arm_common/KinematicsHandler.h"
#include "plog/Log.h"
#include <iostream>
#include <set>
#include <functional>
#include <limits>
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
    if(aConfig.mobileBase)
    {
        if(!kdl_parser::treeFromFile(aConfig.mobileBase->urdfPath, mTree))
        {
            LOGE << "Failed to parse mobile base urdf file to tree";
            return false;
        }

        KDL::Tree manip;
        if(!kdl_parser::treeFromFile(aConfig.manipulator.urdfPath, manip))
        {
            LOGE << "Failed to parse manipulator urdf to KDL::Tree";
            return false;
        }

        // addTree hangs the children of manip's root onto the hook and drops
        // the root itself, so first insert the manip base link as a fixed
        // segment carrying the mount transform, then hang the arm off it
        const std::string& manipBaseLink = aConfig.manipulator.baseLink;
        if(manip.getRootSegment()->first != manipBaseLink)
        {
            LOGE << "Manipulator urdf root link '" << manip.getRootSegment()->first
                 << "' does not match configured base_link '" << manipBaseLink << "'";
            return false;
        }

        const KDL::Segment mount(manipBaseLink, KDL::Joint(manipBaseLink + "_mount_joint", KDL::Joint::Fixed), aConfig.T_V_B);
        if(!mTree.addSegment(mount, aConfig.mobileBase->manipAttachLink))
        {
            LOGE << "Failed to attach manip mount to base at link: " << aConfig.mobileBase->manipAttachLink;
            return false;
        }

        if(!mTree.addTree(manip, manipBaseLink))
        {
            LOGE << "Failed to attach manip tree at link: " << manipBaseLink;
            return false;
        }

        mRobotModel->clear();
        if(!mRobotModel->initFile(aConfig.mobileBase->urdfPath))
        {
            LOGE << "Failed to parse mobile base urdf";
            return false;
        }
    }
    else
    {
        if(!kdl_parser::treeFromFile(aConfig.manipulator.urdfPath, mTree))
        {
            LOGE << "Failed to parse manipulator urdf to KDL::Tree";
            return false;
        }

        mRobotModel.reset();
    }

    mModel->clear();
    if(!mModel->initFile(aConfig.manipulator.urdfPath))
    {
        LOGE << "Could not parse model from urdf";
        return false;
    }

    if(aConfig.gripper)
    {
        KDL::Tree pruned;
        if(!pruneJoints(mTree, aConfig.gripper->jointNames, pruned))
        {
            LOGE << "Failed to prune gripper joints from kinematics tree";
            return false;
        }
        mTree = pruned;
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
        if(nullptr == urdfJoint && mRobotModel)
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

    // A non-controllable base stays in the tree (its measured pose still
    // places the EE), but is locked out of IK: a 0 joint-space weight keeps
    // the velocity solver from moving it
    mLockedJoints.clear();
    if(aConfig.mobileBase && !aConfig.mobileBase->controllable)
    {
        const std::set<std::string> baseJoints(aConfig.mobileBase->jointNames.begin(),
                                               aConfig.mobileBase->jointNames.end());
        for(const auto& [segName, elem] : mTree.getSegments())
        {
            const KDL::Joint& joint = elem.segment.getJoint();
            if(joint.getType() != KDL::Joint::Fixed && baseJoints.count(joint.getName()))
            {
                mLockedJoints.push_back(elem.q_nr);
                LOGD << "Locking non-controllable base joint [" << elem.q_nr << "] " << joint.getName() << " in IK";
            }
        }
    }

    // NR_JL clamps every joint to its limits each iteration — unbound the
    // locked joints so a measured base pose outside its urdf limits survives
    KDL::JntArray q_min = getJointLimits("lower");
    KDL::JntArray q_max = getJointLimits("upper");
    for(unsigned int idx : mLockedJoints)
    {
        q_min(idx) = -std::numeric_limits<double>::max();
        q_max(idx) =  std::numeric_limits<double>::max();
    }

    mManipEndLink = aConfig.manipulator.endLink; 
    std::vector<std::string> endpoints = {mManipEndLink}; 

    mFkSolver  = std::make_shared<KDL::TreeFkSolverPos_recursive>(mTree);
    mVelSolver = std::make_shared<KDL::TreeIkSolverVel_wdls>(mTree, endpoints);
    mIkSolver  = std::make_shared<KDL::TreeIkSolverPos_NR_JL>(
        mTree, endpoints, q_min, q_max, *mFkSolver, *mVelSolver, 200, 1e-3);
    mJacobianSolver = std::make_shared<KDL::TreeJntToJacSolver>(mTree);

    Eigen::MatrixXd Wq = Eigen::MatrixXd::Identity(nj, nj);
    for(unsigned int idx : mLockedJoints)
    {
        Wq(idx, idx) = 0.0;
    }
    mVelSolver->setWeightJS(Wq);
    mVelSolver->setWeightTS(Eigen::MatrixXd::Identity(6 * endpoints.size(), 6 * endpoints.size()));
    // Default lambda is 0, which yields 0/0 = NaN for zero singular values
    // (always present when nj > 6, or near singularities)
    mVelSolver->setLambda(0.01);

    LOGD << "KinematicsHandler initialized successfully";
    return true;
}

bool KinematicsHandler::solveIK(const KDL::JntArray& anInitPos, const KDL::Frame& aGoalPose, KDL::JntArray& aResultOut)
{
    // TODO: error checking on JntArray sizes
    KDL::Frames goalFrames; 
    goalFrames[mManipEndLink] = aGoalPose; 

    if(aResultOut.rows() != anInitPos.rows())
    {
        LOGV << "Resizing result array from " << aResultOut.rows() << " to " << anInitPos.rows();
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
    return solveIK(anInitPos, KDL::JntArray(), aGoalVel, aResultOut);
}

bool KinematicsHandler::solveIK(const KDL::JntArray& anInitPos, const KDL::JntArray& aMeasuredVel,
                                const KDL::Twist& aGoalVel, KDL::JntArray& aResultOut)
{
    // Locked joints (an uncontrolled base) still move the EE: subtract their
    // measured contribution J_locked * qdot_locked so the solved joints make
    // up the difference. Jacobian and twist are both in the root frame with
    // the ref point at the end link.
    KDL::Twist goalVel = aGoalVel;
    if(!mLockedJoints.empty() && aMeasuredVel.rows() == mTree.getNrOfJoints())
    {
        KDL::Jacobian jac(mTree.getNrOfJoints());
        if(mJacobianSolver->JntToJac(anInitPos, jac, mManipEndLink) < 0)
        {
            LOGW << "Failed to compute Jacobian for locked joint feedforward";
            return false;
        }

        for(unsigned int idx : mLockedJoints)
        {
            goalVel = goalVel - jac.getColumn(idx) * aMeasuredVel(idx);
        }
    }

    KDL::Twists goalVels; 
    goalVels[mManipEndLink] = goalVel; 

    double result = mVelSolver->CartToJnt(anInitPos, goalVels, aResultOut);

    // Returns the norm of the weighted input twist on success, negative on error
    if(result < 0)
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

bool KinematicsHandler::pruneJoints(const KDL::Tree& aTree,
                                     const std::vector<std::string>& aJointNamesToRemove,
                                     KDL::Tree& aTreeOut)
{
    const std::set<std::string> toRemove(aJointNamesToRemove.begin(), aJointNamesToRemove.end());
    std::set<std::string> found;

    KDL::SegmentMap::const_iterator rootIt = aTree.getRootSegment();
    aTreeOut = KDL::Tree(rootIt->first);

    std::function<void(const KDL::SegmentMap::const_iterator&, const std::string&)> addChildren =
        [&](const KDL::SegmentMap::const_iterator& aParentIt, const std::string& aParentName)
    {
        for(const auto& childIt : aParentIt->second.children)
        {
            const std::string& jointName = childIt->second.segment.getJoint().getName();

            if(toRemove.count(jointName))
            {
                found.insert(jointName);
                LOGV << "Pruning gripper joint (and its subtree) from kinematics tree: " << jointName;
                continue;
            }

            aTreeOut.addSegment(childIt->second.segment, aParentName);
            addChildren(childIt, childIt->first);
        }
    };

    addChildren(rootIt, rootIt->first);

    for(const auto& jointName : aJointNamesToRemove)
    {
        if(!found.count(jointName))
        {
            LOGW << "Configured gripper joint not found in kinematics tree: " << jointName;
        }
    }

    return true;
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

bool KinematicsHandler::solveTaskVelocity(const KDL::JntArray& aJntCfg, const KDL::JntArray& aJntVel, KDL::Twist& aTwistOut)
{
    KDL::Jacobian jac(mTree.getNrOfJoints());

    if(mJacobianSolver->JntToJac(aJntCfg, jac, mManipEndLink) < 0)
    {
        LOGW << "Failed to compute Jacobian for task velocity";
        return false;
    }

    if(aJntVel.rows() != jac.columns())
    {
        LOGW << "Failed to compute task velocity: joint velocity size mismatch";
        return false;
    }

    KDL::MultiplyJacobian(jac, aJntVel, aTwistOut);
    return true;
}
