
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <arm_common/RosTopicManager.hpp>
#include <arm_msgs/msg/detail/hardware_state__struct.hpp>
#include <kdl/jntarray.hpp>
#include <string>
#include <plog/Log.h>

#include "arm_msgs/msg/hardware_state.hpp"

#include "arm_hardware/HardwareManager.h"
#include "arm_hardware/ManipulatorFactory.h"
#include <arm_common/Config.hpp>
#include <arm_common/RateController.hpp>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>

HardwareManager::HardwareManager()
{

}

HardwareManager::~HardwareManager()
{

}

bool HardwareManager::init()
{
    RosTopicManager::getInstance()->createPublisher<arm_msgs::msg::HardwareState>("arm/hardware_state");

    mComms = ManipulatorFactory::create(ConfigManager::getInstance()->getManipConfig());
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
    // TODO: set name from config

    while(mRunning)
    {
        rate.start();

        pos = mComms->getJointPositions();
        vel = mComms->getJointPositions();
        posV.clear(); velV.clear();
        posV.resize(pos.rows()); velV.resize(vel.rows());

        for(int i = 0; i < (int)pos.rows(); i++)
        {
            posV[i] = pos(i);
            velV[i] = vel(i);
        }

        state.set__position(posV);
        state.set__velocity(velV);
        RosTopicManager::getInstance()->publishMessage("arm/hardware_state", state);

        rate.block();
    }
}
