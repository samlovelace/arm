#ifndef ROSMANIPCOMMS_H
#define ROSMANIPCOMMS_H

#include <mutex>
#include <chrono>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "ptera_msgs/msg/robot_state.hpp"
#include "arm_hardware/IManipComms.hpp"
#include "arm_common/ConfigManager.h"

class RosManipComms : public IManipComms
{
public:
    RosManipComms(const ConfigManager::Config& aConfig);
    ~RosManipComms() override;

    bool init() override;
    KDL::JntArray getJointPositions() override;
    KDL::JntArray getJointVelocities() override;
    void sendJointCommand(const KDL::JntArray &aCmd) override;

    KDL::JntArray getGripperPositions() override;
    KDL::JntArray getGripperVelocities() override;
    void sendGripperCommand(const KDL::JntArray &aCmd) override;

private:
    void publishCombinedCommand();

private:

    void armStateCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void vehicleStateCallback(ptera_msgs::msg::RobotState::SharedPtr msg);

    // Mobile-base DOF count (0 for a standalone manipulator). Every mobile
    // base in this repo is modeled as exactly [x_joint, y_joint, yaw_joint],
    // so /robot/state's position.x/position.y/euler.yaw map onto indices 0-2.
    int mNumBaseJoints;
    int mNumArmJoints;
    int mNumGripperJoints;

    std::mutex mJointPosMutex;
    KDL::JntArray mJointPositions;

    std::mutex mJointVelMutex;
    KDL::JntArray mJointVelocities;

    // Arm and gripper share one combined command array on the wire
    // (UR/joint_commands) but arrive independently via sendJointCommand /
    // sendGripperCommand — cache both and republish the merge each time.
    std::mutex mCmdMutex;
    std::vector<double> mLastArmCmd;
    std::vector<double> mLastGripperCmd;

    // Arm velocity finite-differencing (from UR/joint_positions)
    KDL::JntArray mPrevJointPos;
    std::chrono::time_point<std::chrono::steady_clock> mPrevTime;

    // Vehicle velocity finite-differencing (from /robot/state) — [x, y, yaw]
    KDL::JntArray mPrevVehiclePos;
    std::chrono::time_point<std::chrono::steady_clock> mPrevVehicleTime;

    bool mFirstRcvd; // guards the first /robot/state message so it doesn't produce a velocity spike

};
#endif //ROSMANIPCOMMS_H
