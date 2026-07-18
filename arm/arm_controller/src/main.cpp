
#include <arm_common/RosTopicManager.hpp>
#include <cstdio>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "arm_common/Logger.h"
#include "arm_common/ConfigManager.h"

#include "arm_controller/Manipulator.h"
#include "arm_controller/StateMachine.h"
#include "arm_controller/CommandHandler.h"

void signalHandler(int signal)
{
    RCLCPP_INFO(rclcpp::get_logger("arm_controller"), "Received signal %d", signal);
    rclcpp::shutdown();
    exit(0); 
}

int main()
{
    std::signal(SIGINT, signalHandler);

    rclcpp::init(0, nullptr);
    Logger::get().createMainLog("arm_controller");
    RosTopicManager::getInstance("arm_controller")->spinNode();

    ConfigManager::getInstance()->loadConfig();

    auto manip = std::make_shared<Manipulator>(ConfigManager::getInstance()->getConfig());
    auto sm = std::make_shared<StateMachine>(manip);
    CommandHandler* cm = new CommandHandler(sm, manip);

    sm->run();

    return 0;
}
