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
    bool setGoalWaypoint(std::shared_ptr<IWaypoint> aWp);

    std::shared_ptr<IWaypoint> getGoalWaypoint();
    bool isArrived();
    void printArrivedState();
    bool sendToPose(Manipulator::POSE aPose);
    void setEnabledState(bool anEnabledFlag);

    std::shared_ptr<KinematicsHandler> getKinematicsHandler() {return mKinematicsHandler; }
    KDL::JntArray getCurrentJointPos() {return mHardware->getJointPositions();}
    KDL::Frame getBaseInVehicleFrame() {return mConfig.T_V_B;}

    void startControl();

    bool isEnabled();

private:
    std::shared_ptr<HardwareInterface> mHardware;
    ConfigManager::Config mConfig;
    std::unique_ptr<WaypointExecutor> mWaypointExecutor;
    std::shared_ptr<KinematicsHandler> mKinematicsHandler;

    std::mutex mGoalJntPosMutex;
    KDL::JntArray mGoalJntPos;
    std::unique_ptr<RateController> mArmControlRate;

    std::mutex mEnabledMutex;
    bool mEnabled;

    std::atomic<bool> mVelocityMode;

    std::shared_ptr<IWaypoint> mInitialGoalWp;
    std::shared_ptr<IWaypoint> mGoalWaypoint;

    std::map<POSE, KDL::JntArray> mArmPoseMap;

    std::thread mControlThread;
    std::thread mArrivalThread;
    std::thread mPublishThread;

    void controlLoop();
    void arrivalLoop();
    void publishStateLoop();

    bool checkArrival();

    void logWaypointError();

    void setArrivalState(bool aFlag);

    std::mutex mArrivalMutex;
    bool mIsArrived;

};
#endif //MANIPULATOR_H
