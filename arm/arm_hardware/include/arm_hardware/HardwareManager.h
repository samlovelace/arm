#ifndef HARDWAREMANAGER_H
#define HARDWAREMANAGER_H

#include "arm_msgs/msg/hardware_goal.hpp"

#include "arm_hardware/IManipComms.hpp"

#include <arm_msgs/msg/detail/hardware_goal__struct.hpp>
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
    void goalCallback(arm_msgs::msg::HardwareGoal::SharedPtr aGoal);

private:
    std::shared_ptr<IManipComms> mComms;
    std::atomic_bool mRunning;
};
#endif
