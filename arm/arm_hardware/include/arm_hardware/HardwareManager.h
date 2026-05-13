#ifndef HARDWAREMANAGER_H
#define HARDWAREMANAGER_H

#include "arm_hardware/IManipComms.hpp"
#include "arm_common/ConfigManager.h"

#include <atomic>
#include <memory>

class HardwareManager
{
public:
    HardwareManager();
    ~HardwareManager();

    bool init();
    void run();

private:
    std::shared_ptr<IManipComms> mComms;

    std::atomic_bool mRunning;
};
#endif
