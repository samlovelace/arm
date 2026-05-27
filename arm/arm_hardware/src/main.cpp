
#include <rclcpp/utilities.hpp>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "arm_common/ConfigManager.h"
#include "arm_common/RosTopicManager.hpp"

#include "arm_hardware/HardwareManager.h"

int main()
{
    rclcpp::init(0, nullptr);
    RosTopicManager::getInstance("arm_hardware")->spinNode();

    ConfigManager::getInstance()->loadConfig();

    HardwareManager hwMan;
    if(!hwMan.init())
    {
        std::cerr << "Failed to initialize hardware manager" << std::endl;
        return 1;
    }

    hwMan.run();

    rclcpp::shutdown();
}
