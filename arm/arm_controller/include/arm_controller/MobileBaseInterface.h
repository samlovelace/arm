#ifndef MOBILEBASEINTERFACE_H
#define MOBILEBASEINTERFACE_H

#include <mutex>
#include <string>
#include <vector>
#include <kdl/jntarray.hpp>

#include "arm_common/BaseState.hpp"
#include "arm_common/ConfigManager.h"

#include "ptera_msgs/msg/robot_state.hpp"

// Generic 6-DOF mobile base interface. State arrives as a rigid-body pose +
// world-frame twist on robot/state; commands go out as 6-DOF VehicleWaypoints.
// The base's virtual joints in the kinematic tree are a subset of those DOFs,
// selected by MobileBaseConfig::jointAxes.
class MobileBaseInterface
{
public:
    MobileBaseInterface(const ConfigManager::MobileBaseConfig& aConfig, int aControlRateHz);
    ~MobileBaseInterface();

    bool init();

    bool hasState();
    BaseState getState();

    // Base virtual joint state, ordered like MobileBaseConfig::jointNames
    KDL::JntArray getJointPositions();
    KDL::JntArray getJointVelocities();
    int getNrJoints() const { return (int)mJointAxes.size(); }

    // Mapped DOFs come from aCmd, unmapped DOFs hold their latest measured value
    void sendCommand(const KDL::JntArray& aCmd);

private:
    void stateCallback(ptera_msgs::msg::RobotState::SharedPtr aState);

    std::vector<std::string> mJointAxes;
    std::string mCommandTopic;
    bool mControllable;
    double mControlPeriod; // seconds, used as the waypoint execution duration

    std::mutex mStateMutex;
    BaseState mLatestState;
    bool mHasState;
};
#endif
