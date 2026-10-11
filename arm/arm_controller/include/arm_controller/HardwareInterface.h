#ifndef HARDWAREINTERFACE_H
#define HARDWAREINTERFACE_H

#include <mutex>
#include <kdl/jntarray.hpp>

#include "arm_msgs/msg/hardware_telemetry.hpp"

class HardwareInterface
{
public:
    HardwareInterface();
    ~HardwareInterface();

    bool init();
    KDL::JntArray getJointPositions();
    KDL::JntArray getJointVelocities();
    void sendJointCommand(const KDL::JntArray &aCmd);

    KDL::JntArray getGripperPositions();
    KDL::JntArray getGripperVelocities();
    void sendGripperCommand(const KDL::JntArray &aCmd);

private:
    void stateCallback(arm_msgs::msg::HardwareTelemetry::SharedPtr aState);
    void gripperStateCallback(arm_msgs::msg::HardwareTelemetry::SharedPtr aState);

    std::mutex mStateMutex;
    KDL::JntArray mLatestPosition;
    KDL::JntArray mLatestVelocity;

    std::mutex mGripperStateMutex;
    KDL::JntArray mLatestGripperPosition;
    KDL::JntArray mLatestGripperVelocity;

};
#endif
