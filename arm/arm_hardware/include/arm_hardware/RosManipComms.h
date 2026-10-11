#ifndef ROSMANIPCOMMS_H
#define ROSMANIPCOMMS_H

#include <mutex>
#include <chrono>

#include "std_msgs/msg/float64_multi_array.hpp"
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

    // Arm only — the mobile base is owned by arm_controller.
    // State arrays are laid out [arm..., gripper...]
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

};
#endif //ROSMANIPCOMMS_H
