#ifndef KINEMATICSHANDLER_H
#define KINEMATICSHANDLER_H
 
#include <memory>
#include <unordered_map>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <urdf/model.h>

struct CollisionShape 
{
  enum Type { Box, Cylinder, Sphere, Mesh } type;

  // Box
  double bx=0, by=0, bz=0;
  // Cylinder
  double radius=0, length=0;
  // Sphere
  double sr=0;
  // Mesh
  std::string mesh_filename;
  double mesh_scale_x=1, mesh_scale_y=1, mesh_scale_z=1;
};

struct CollisionShell
{
    KDL::Frame T_link_shell; // shell in link frame
    KDL::Frame T_base_shell;   // shell in manipulator base frame 

    CollisionShape shape; 
};
 
class KinematicsHandler 
{ 
public:
    KinematicsHandler();
    ~KinematicsHandler();

    bool init(const std::string& anUrdfPath, const std::string& aRobotUrdfPath); 
    bool solveIK(const KDL::JntArray& anInitPos, const KDL::Frame& aGoalPose, KDL::JntArray& aResultOut);
    bool solveIK(const KDL::JntArray& anInitPos, const KDL::Twist& aGoalVel, KDL::JntArray& aResultOut);
    bool solveFk(const KDL::JntArray& anInitPos, KDL::Frame& aFrameOut, int aSegmentNr = -1); 

    void updateCollisionShells(const KDL::JntArray& aCurrentJointState);
    bool checkCollisions(const KDL::JntArray& aJntConfig);

    double computeManipulability(const KDL::JntArray& aJntCfg);

    KDL::JntArray getJointLimits(const std::string& aLimitType); 
    int getNrJoints() {return mChain.getNrOfJoints(); }

private: 
    bool parseManipCollisionGeometry();
    bool parseRobotCollisionGeometry();     
    void parseLinkCollisions(urdf::LinkConstSharedPtr& aLink, std::vector<CollisionShell>& aLinkShellsOut);

    bool collides(const CollisionShell& aFirstShell, const CollisionShell& aSecondShell); 
    void shellToPoints(const CollisionShell& aShell, std::vector<KDL::Vector>& aPoints);
    double segmentDistance(const KDL::Vector& p1, const KDL::Vector& q1, const KDL::Vector& p2, const KDL::Vector& q2); 

private:

    KDL::Tree mTree; 
    KDL::Chain mChain; 
    std::shared_ptr<urdf::Model> mModel; 
    std::shared_ptr<urdf::Model> mRobotModel; 

    std::vector<std::string> mJointNames; 
    
    std::map<std::string, KDL::JntArray> mLimitsMap; 
    std::map<std::string, unsigned int> mSegmentNameMap; 
    std::vector<std::vector<CollisionShell>> mCollisionShells; 
    std::vector<std::vector<CollisionShell>> mRobotShells; 

    std::shared_ptr<KDL::ChainFkSolverPos_recursive> mFkSolver; 
    std::shared_ptr<KDL::ChainIkSolverVel_pinv> mVelSolver; 
    std::shared_ptr<KDL::ChainIkSolverPos_NR_JL> mIkSolver; 
    std::shared_ptr<KDL::ChainJntToJacSolver> mJacobianSolver;  

};
#endif //KINEMATICSHANDLER_H