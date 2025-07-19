#ifndef KINEMATICSHANDLER_H
#define KINEMATICSHANDLER_H
 
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <memory> 
 
class KinematicsHandler 
{ 
public:
    KinematicsHandler();
    ~KinematicsHandler();

    bool init(const std::string& anUrdfPath); 
    bool solveIK(const KDL::JntArray& anInitPos, const KDL::Frame& aGoalPose, KDL::JntArray& aResultOut);
    bool solveFk(const KDL::JntArray& anInitPos, KDL::Frame& aFrameOut); 

    KDL::JntArray getJointLimits(const std::string& aLimitType); 
private:

    KDL::Tree mTree; 
    KDL::Chain mChain; 
    std::shared_ptr<urdf::Model> mModel; 

    std::vector<std::string> mJointNames; 
    
    std::map<std::string, KDL::JntArray> mLimitsMap; 

    std::shared_ptr<KDL::ChainFkSolverPos_recursive> mFkSolver; 
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> mVelSolver; 
    std::shared_ptr<KDL::ChainIkSolverPos_NR_JL> mIkSolver; 

};
#endif //KINEMATICSHANDLER_H