#ifndef SIMMANIPCOMMS_H
#define SIMMANIPCOMMS_H

#include <kdl/jntarray.hpp>
#include <string>
#include <vector>

#include "plog/Log.h"

#include <sensor_msgs/msg/joint_state.hpp>

#include "arm_hardware/IManipComms.hpp"
#include "arm_common/RosTopicManager.hpp"

static constexpr const char* JOINT_STATES_TOPIC = "joint_states";

class SimManipComms : public IManipComms
{
public:
    SimManipComms(std::vector<std::string> aJointNames,
                   std::vector<std::string> aGripperJointNames = {},
                   double aSmoothing = 0.05)
        : mJointNames(aJointNames)
        , mPositions(aJointNames.size())
        , mVelocities(aJointNames.size())
        , mGoal(aJointNames.size())
        , mGripperJointNames(aGripperJointNames)
        , mGripperPositions(aGripperJointNames.size())
        , mGripperVelocities(aGripperJointNames.size())
        , mGripperGoal(aGripperJointNames.size())
        , mSmoothing(aSmoothing)
    {
        KDL::SetToZero(mPositions);
        KDL::SetToZero(mVelocities);
        KDL::SetToZero(mGoal);
        KDL::SetToZero(mGripperPositions);
        KDL::SetToZero(mGripperVelocities);
        KDL::SetToZero(mGripperGoal);
    }

    bool init() override
    {
        auto* ros = RosTopicManager::getInstance();

        ros->createPublisher<sensor_msgs::msg::JointState>(JOINT_STATES_TOPIC);

        mTimer = ros->create_wall_timer(
            std::chrono::milliseconds(20),
            [this]() { step(); }
        );

        RCLCPP_INFO(ros->get_logger(), "SimManipComms initialized with %zu joints", mJointNames.size());
        return true;
    }

    KDL::JntArray getJointPositions() override { return mPositions; }
    KDL::JntArray getJointVelocities() override { return mVelocities; }

    void sendJointCommand(const KDL::JntArray &aCmd) override
    {
        if (aCmd.rows() != mGoal.rows())
        {
            RCLCPP_WARN(RosTopicManager::getInstance()->get_logger(),
                "Command size mismatch: got %u, expected %u", aCmd.rows(), mGoal.rows());
            return;
        }

        LOGV << "Received joint command: ";
        for(int i = 0; i < aCmd.rows(); i++)        {
            LOGV << "aCmd(" << i << "): " << aCmd(i);
        }
        mGoal = aCmd;
    }

    KDL::JntArray getGripperPositions() override { return mGripperPositions; }
    KDL::JntArray getGripperVelocities() override { return mGripperVelocities; }

    void sendGripperCommand(const KDL::JntArray &aCmd) override
    {
        if (aCmd.rows() != mGripperGoal.rows())
        {
            RCLCPP_WARN(RosTopicManager::getInstance()->get_logger(),
                "Gripper command size mismatch: got %u, expected %u", aCmd.rows(), mGripperGoal.rows());
            return;
        }

        mGripperGoal = aCmd;
    }

private:
    void step()
    {
        constexpr double dt = 0.02;

        for (unsigned int i = 0; i < mPositions.rows(); ++i)
        {
            double prev = mPositions(i);
            mPositions(i) = mPositions(i) + (1.0 - mSmoothing) * (mGoal(i) - mPositions(i));
            mVelocities(i) = (mPositions(i) - prev) / dt;
        }

        for (unsigned int i = 0; i < mGripperPositions.rows(); ++i)
        {
            double prev = mGripperPositions(i);
            mGripperPositions(i) = mGripperPositions(i) + (1.0 - mSmoothing) * (mGripperGoal(i) - mGripperPositions(i));
            mGripperVelocities(i) = (mGripperPositions(i) - prev) / dt;
        }

        publishJointStates();
    }

    void publishJointStates()
    {
        sensor_msgs::msg::JointState msg;
        msg.header.stamp = RosTopicManager::getInstance()->now();
        msg.header.frame_id = "world";
        msg.name = mJointNames;
        msg.name.insert(msg.name.end(), mGripperJointNames.begin(), mGripperJointNames.end());

        for (unsigned int i = 0; i < mPositions.rows(); ++i)
        {
            msg.position.push_back(mPositions(i));
            msg.velocity.push_back(mVelocities(i));
            msg.effort.push_back(0.0);
        }

        for (unsigned int i = 0; i < mGripperPositions.rows(); ++i)
        {
            msg.position.push_back(mGripperPositions(i));
            msg.velocity.push_back(mGripperVelocities(i));
            msg.effort.push_back(0.0);
        }

        RosTopicManager::getInstance()->publishMessage(JOINT_STATES_TOPIC, msg);
    }

    std::vector<std::string> mJointNames;
    KDL::JntArray mPositions;
    KDL::JntArray mVelocities;
    KDL::JntArray mGoal;

    std::vector<std::string> mGripperJointNames;
    KDL::JntArray mGripperPositions;
    KDL::JntArray mGripperVelocities;
    KDL::JntArray mGripperGoal;

    double mSmoothing;
    rclcpp::TimerBase::SharedPtr mTimer;
};

#endif
