cmake_minimum_required(VERSION 3.8)
project(arm_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/HardwareGoal.msg"
    "msg/HardwareState.msg"
)

ament_package()
