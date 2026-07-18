
#include <functional>
#include <kdl/jntarray.hpp>
#include <string>
#include <plog/Log.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "arm_common/RosTopicManager.hpp"
#include "arm_common/ConfigManager.h"
#include "arm_common/RateController.hpp"

#include "arm_msgs/msg/hardware_state.hpp"

#include "arm_hardware/HardwareManager.h"
#include "arm_hardware/ManipulatorFactory.h"

HardwareManager::HardwareManager()
{

}

HardwareManager::~HardwareManager()
{

}

bool HardwareManager::init()
{
    RosTopicManager::getInstance()->createPublisher<arm_msgs::msg::HardwareState>("arm/hardware/state");
    RosTopicManager::getInstance()->createSubscriber<arm_msgs::msg::HardwareGoal>("arm/hardware/goal",
        std::bind(&HardwareManager::goalCallback, this, std::placeholders::_1));

    mHasGripper = ConfigManager::getInstance()->getConfig().gripper.has_value();

    if(mHasGripper)
    {
        RosTopicManager::getInstance()->createPublisher<arm_msgs::msg::HardwareState>("arm/hardware/gripper_state");
        RosTopicManager::getInstance()->createSubscriber<arm_msgs::msg::HardwareGoal>("arm/hardware/gripper_goal",
            std::bind(&HardwareManager::gripperGoalCallback, this, std::placeholders::_1));
    }

    mComms = ManipulatorFactory::create();
    mComms->init();

    if(nullptr == mComms)
    {
        LOGE << "Failed to initialize hardware comms";
        return false;
    }

    return true;
}

void HardwareManager::run()
{
    RateController rate(20); // TODO: make config
    mRunning = true;

    KDL::JntArray pos, vel;
    std::vector<double> posV, velV;

    arm_msgs::msg::HardwareState state;
    arm_msgs::msg::HardwareState gripperState;
    // TODO: set name from config

    while(mRunning)
    {
        rate.start();

        pos = mComms->getJointPositions();
        vel = mComms->getJointVelocities();
        posV.clear(); velV.clear();
        posV.resize(pos.rows()); velV.resize(vel.rows());

        for(int i = 0; i < (int)pos.rows(); i++)
        {
            posV[i] = pos(i);
            velV[i] = vel(i);
        }

        state.set__position(posV);
        state.set__velocity(velV);
        RosTopicManager::getInstance()->publishMessage("arm/hardware/state", state);

        if(mHasGripper)
        {
            KDL::JntArray gripperPos = mComms->getGripperPositions();
            KDL::JntArray gripperVel = mComms->getGripperVelocities();

            std::vector<double> gripperPosV(gripperPos.rows());
            std::vector<double> gripperVelV(gripperVel.rows());

            for(int i = 0; i < (int)gripperPos.rows(); i++)
            {
                gripperPosV[i] = gripperPos(i);
                gripperVelV[i] = gripperVel(i);
            }

            gripperState.set__position(gripperPosV);
            gripperState.set__velocity(gripperVelV);
            RosTopicManager::getInstance()->publishMessage("arm/hardware/gripper_state", gripperState);
        }

        rate.block();
    }
}

void HardwareManager::goalCallback(arm_msgs::msg::HardwareGoal::SharedPtr aGoal)
{
    KDL::JntArray goal(aGoal->position.size());
    for(int i = 0; i < (int) goal.rows(); i++)
    {
        goal(i) = aGoal->position[i];
    }

    mComms->sendJointCommand(goal);
}

void HardwareManager::gripperGoalCallback(arm_msgs::msg::HardwareGoal::SharedPtr aGoal)
{
    KDL::JntArray goal(aGoal->position.size());
    for(int i = 0; i < (int) goal.rows(); i++)
    {
        goal(i) = aGoal->position[i];
    }

    mComms->sendGripperCommand(goal);
}
