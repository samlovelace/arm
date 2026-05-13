cmake_minimum_required(VERSION 3.8)
project(arm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(arm_common REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)

set(DYNAMIXEL_LIB /usr/local/lib/libdxl_x64_cpp.so)

set(SOURCES
    src/DynamixelManipComms.cpp
    src/GazeboManipComms.cpp
    src/HardwareManager.cpp
    src/main.cpp
    src/ManipulatorFactory.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
    /usr/local/include # for dynamixel lib install
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    std_msgs
    sensor_msgs
    arm_common
    Eigen3
    kdl_parser
)

target_link_libraries(${PROJECT_NAME} ${DYNAMIXEL_LIB})

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
