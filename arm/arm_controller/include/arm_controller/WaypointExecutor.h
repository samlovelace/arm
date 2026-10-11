#ifndef WAYPOINTEXECUTOR_H
#define WAYPOINTEXECUTOR_H

#include <kdl/jntarray.hpp>
#include <memory>

#include "ruckig/ruckig.hpp"

#include "arm_common/ConfigManager.h"
#include "arm_common/KinematicsHandler.h"

#include "arm_controller/JointPositionWaypoint.h"

class WaypointExecutor
{
public:
    WaypointExecutor(ConfigManager::Config& aConfig);
    ~WaypointExecutor();

    // aNumDof controlled joints, starting at aOffset in the kinematics' joint indexing
    void init(int aNumDof, int aOffset, KinematicsHandler& aKinematicsHandler);
    bool initializeExecutor(KDL::JntArray aGoalJointPos, KDL::JntArray aCurrentJointPos, KDL::JntArray aCurrentJointVel, ruckig::ControlInterface aControlType);
    KDL::JntArray getNextWaypoint();

    // Retarget without touching current state (Ruckig carries it via pass_to_input)
    bool setGoalState(KDL::JntArray aGoalJointPos, ruckig::ControlInterface aControlType);

private:
    std::unique_ptr<ruckig::Ruckig<0>> mExecutor; // 0 indicates dynamic
    std::unique_ptr<ruckig::InputParameter<0>> mInput;
    std::unique_ptr<ruckig::OutputParameter<0>> mOutput;

    ConfigManager::Config mConfig;
    std::vector<double> mVelocityLimits;
    std::vector<double> mLowerPosLimits;
    std::vector<double> mUpperPosLimits;

    bool mInitialSet;

    int mNumDof;

    void setInitialState(const KDL::JntArray& aPos, const KDL::JntArray& aVel);

};
#endif //WAYPOINTEXECUTOR_H
