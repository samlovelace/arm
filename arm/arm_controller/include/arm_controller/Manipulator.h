#ifndef MANIPULATOR_H
#define MANIPULATOR_H

#include <memory>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <mutex>
#include <map>
#include <thread>

#include "HardwareInterface.h"
#include "arm_common/ConfigManager.h"
#include "arm_common/RateController.hpp"
#include "arm_common/KinematicsHandler.h"

#include "arm_controller/WaypointExecutor.h"
#include "arm_controller/IWaypoint.hpp"
#include "arm_controller/TaskPositionWaypoint.h"
#include "arm_controller/JointPositionWaypoint.h"
#include "arm_controller/HardwareInterface.h"
#include "arm_controller/MobileBaseInterface.h"

class Manipulator
{
public:
    Manipulator(const ConfigManager::Config aConfig);
    ~Manipulator();

    enum class POSE
    {
        STOW,
        DEPLOY,
        NUM_TYPES
    };

    void setJointPositionGoal(const KDL::JntArray &aNewJntPos);
    // aResetState=false only retargets the executor for the current waypoint
    // (used by the control loop to re-solve task waypoints each tick)
    bool setGoalWaypoint(std::shared_ptr<IWaypoint> aWp, bool aResetState = true);

    // Gripper joints bypass WaypointExecutor entirely — sent directly to
    // hardware, no Ruckig trajectory smoothing or arrival tracking.
    int getGripperNrJoints() const { return mConfig.gripper ? (int)mConfig.gripper->jointNames.size() : 0; }
    void setGripperGoal(const KDL::JntArray& aGripperGoal);

    std::shared_ptr<IWaypoint> getGoalWaypoint();
    bool isArrived();
    void printArrivedState();
    bool sendToPose(Manipulator::POSE aPose);
    void setEnabledState(bool anEnabledFlag);

    std::shared_ptr<KinematicsHandler> getKinematicsHandler() {return mKinematicsHandler; }

    // Joints driven by trajectory generation and hardware commands — excludes
    // the mobile base joints when the base isn't controllable
    int getNrCtrlJoints() const { return (int)mConfig.ctrlJointNames.size(); }
    KDL::JntArray getCurrentJointPos() {return getFullJointPositions();}

    // Full kinematic joint state [base..., arm...] (no gripper), composed from
    // the mobile base and arm hardware. Empty until both have reported.
    KDL::JntArray getFullJointPositions();
    KDL::JntArray getFullJointVelocities();
    KDL::Frame getBaseInVehicleFrame() {return mConfig.T_V_B;}

    void startControl();

    bool isEnabled();

private:
    std::shared_ptr<HardwareInterface> mHardware;
    std::shared_ptr<MobileBaseInterface> mBase; // null without a mobile base
    ConfigManager::Config mConfig;
    std::unique_ptr<WaypointExecutor> mWaypointExecutor;
    std::shared_ptr<KinematicsHandler> mKinematicsHandler;

    std::mutex mGoalJntPosMutex;
    KDL::JntArray mGoalJntPos;

    std::mutex mEnabledMutex;
    bool mEnabled;

    std::atomic<bool> mVelocityMode;

    std::shared_ptr<IWaypoint> mInitialGoalWp;
    std::shared_ptr<IWaypoint> mGoalWaypoint;
    std::mutex mGoalMutex; // guards mGoalWaypoint and mWaypointExecutor
    KDL::JntArray mLastTaskSolution; // full-size IK seed for task position refreshes (guarded by mGoalMutex)
    bool mExecutorReady; // executor initialized for mGoalWaypoint (guarded by mGoalMutex)

    std::map<POSE, KDL::JntArray> mArmPoseMap;

    std::thread mControlThread;
    std::thread mArrivalThread;
    std::thread mPublishThread;

    // Publishes arm/hardware/state (joint + global task-space state) for the
    // node's lifetime, independent of enable state
    std::atomic<bool> mRunning;
    std::thread mHardwareStateThread;

    void controlLoop();
    void arrivalLoop();
    void publishStateLoop();
    void publishHardwareStateLoop();

    bool checkArrival();

    // Slice a full [base..., arm...] joint array down to the controlled joints
    KDL::JntArray toCtrl(const KDL::JntArray& aFull) const;

    // Concatenate [base..., arm...], or empty if either source hasn't reported
    KDL::JntArray composeFull(const KDL::JntArray& aBase, const KDL::JntArray& aArm) const;

    // Route a controlled-joint command to the base (when controllable) and arm
    void sendCommand(const KDL::JntArray& aCtrlCmd);

    void logWaypointError();

    void setArrivalState(bool aFlag);

    std::mutex mArrivalMutex;
    bool mIsArrived;

};
#endif //MANIPULATOR_H
