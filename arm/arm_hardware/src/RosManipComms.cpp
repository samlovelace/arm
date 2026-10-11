
#include "std_msgs/msg/float64_multi_array.hpp"
#include "plog/Log.h"

#include "arm_common/RosTopicManager.hpp"
#include "arm_hardware/RosManipComms.h"

RosManipComms::RosManipComms(const ConfigManager::Config& aConfig) :
        mPrevJointPos(0), mPrevTime(std::chrono::steady_clock::now())
{
    mNumArmJoints     = static_cast<int>(aConfig.manipulator.jointNames.size());
    mNumGripperJoints = aConfig.gripper ? static_cast<int>(aConfig.gripper->jointNames.size()) : 0;

    const int totalJoints = mNumArmJoints + mNumGripperJoints;

    mPrevJointPos.resize(mNumArmJoints + mNumGripperJoints);
    mJointPositions.resize(totalJoints);
    mJointVelocities.resize(totalJoints);

    mLastArmCmd.assign(mNumArmJoints, 0.0);
    mLastGripperCmd.assign(mNumGripperJoints, 0.0);

    for(int i = 0; i < mPrevJointPos.rows(); i++)
    {
        mPrevJointPos(i) = 0.0;
    }

    for(int i = 0; i < mJointPositions.rows(); i++)
    {
        mJointPositions(i) = 0.0;
        mJointVelocities(i) = 0.0;
    }
}

RosManipComms::~RosManipComms()
{

}

bool RosManipComms::init()
{
    RosTopicManager::getInstance()->
        createSubscriber<std_msgs::msg::Float64MultiArray>("gazebo/joint_positions",
                                                            std::bind(&RosManipComms::armStateCallback,
                                                                    this,
                                                                    std::placeholders::_1));

    RosTopicManager::getInstance()->
        createPublisher<std_msgs::msg::Float64MultiArray>("gazebo/joint_commands");

    return true;
}

KDL::JntArray RosManipComms::getJointPositions()
{
    std::lock_guard<std::mutex> lock(mJointPosMutex);
    KDL::JntArray armPos(mNumArmJoints);
    for(int i = 0; i < armPos.rows(); i++)
    {
        armPos(i) = mJointPositions(i);
    }
    return armPos;
}

KDL::JntArray RosManipComms::getJointVelocities()
{
    std::lock_guard<std::mutex> lock(mJointVelMutex);
    KDL::JntArray armVel(mNumArmJoints);
    for(int i = 0; i < armVel.rows(); i++)
    {
        armVel(i) = mJointVelocities(i);
    }
    return armVel;
}

KDL::JntArray RosManipComms::getGripperPositions()
{
    std::lock_guard<std::mutex> lock(mJointPosMutex);
    KDL::JntArray gripperPos(mNumGripperJoints);
    for(int i = 0; i < mNumGripperJoints; i++)
    {
        gripperPos(i) = mJointPositions(mNumArmJoints + i);
    }
    return gripperPos;
}

KDL::JntArray RosManipComms::getGripperVelocities()
{
    std::lock_guard<std::mutex> lock(mJointVelMutex);
    KDL::JntArray gripperVel(mNumGripperJoints);
    for(int i = 0; i < mNumGripperJoints; i++)
    {
        gripperVel(i) = mJointVelocities(mNumArmJoints + i);
    }
    return gripperVel;
}

void RosManipComms::sendJointCommand(const KDL::JntArray &aCmd)
{
    // Arm joints only — the mobile base is owned by arm_controller
    std::lock_guard<std::mutex> lock(mCmdMutex);
    for(int i = 0; i < static_cast<int>(aCmd.rows()) && i < mNumArmJoints; i++)
    {
        mLastArmCmd[i] = aCmd(i);
    }
    publishCombinedCommand();
}

void RosManipComms::sendGripperCommand(const KDL::JntArray &aCmd)
{
    std::lock_guard<std::mutex> lock(mCmdMutex);
    for(int i = 0; i < static_cast<int>(aCmd.rows()) && i < mNumGripperJoints; i++)
    {
        mLastGripperCmd[i] = aCmd(i);
    }
    publishCombinedCommand();
}

// Arm and gripper share one combined Float64MultiArray on the wire
// (UR/joint_commands) — publish the merge of whichever last changed.
// Caller must hold mCmdMutex.
void RosManipComms::publishCombinedCommand()
{
    std::vector<double> posCmd;
    posCmd.reserve(mLastArmCmd.size() + mLastGripperCmd.size());
    posCmd.insert(posCmd.end(), mLastArmCmd.begin(), mLastArmCmd.end());
    posCmd.insert(posCmd.end(), mLastGripperCmd.begin(), mLastGripperCmd.end());

    std_msgs::msg::Float64MultiArray cmd;
    cmd.set__data(posCmd);
    RosTopicManager::getInstance()->publishMessage<std_msgs::msg::Float64MultiArray>("gazebo/joint_commands", cmd);
}

void RosManipComms::armStateCallback(std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    auto now = std::chrono::steady_clock::now();
    const std::size_t n = msg->data.size();

    {
        std::lock_guard<std::mutex> lock(mJointPosMutex);
        for(std::size_t i = 0; i < n; i++)
        {
            mJointPositions(i) = msg->data[i];
        }
    }

    const double dt = std::chrono::duration<double>(now - mPrevTime).count(); // seconds
    if(dt > 0.0)
    {
        std::lock_guard<std::mutex> lock(mJointVelMutex);
        for(std::size_t i = 0; i < n; i++)
        {
            mJointVelocities(i) = (msg->data[i] - mPrevJointPos(i)) / dt;
        }
    }

    for(std::size_t i = 0; i < n; i++)
    {
        mPrevJointPos(i) = msg->data[i];
    }
    mPrevTime = now;
}
