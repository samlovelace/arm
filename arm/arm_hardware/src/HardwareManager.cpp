
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <plog/Log.h>

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

    while(mRunning)
    {
        rate.start();

        rate.block();
    }
}
