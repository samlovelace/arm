cmake_minimum_required(VERSION 3.8)
project(arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_idl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

set(PCL_DIR "/usr/local/pcl-1.15/lib/cmake/pcl")
find_package(PCL REQUIRED)

set(DYNAMIXEL_LIB /usr/local/lib/libdxl_x64_cpp.so)

# disable Ruckig examples and tests
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../dependencies/ruckig ruckig)

set(COMMON_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/src/ConfigManager.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/KinematicsHandler.cpp
)

set(SOURCES 
  ${CMAKE_CURRENT_LIST_DIR}/src/CommandHandler.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/DynamixelManipComms.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/GazeboManipComms.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/JointPositionWaypoint.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/JointVelocityWaypoint.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/Manipulator.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/ManipulatorFactory.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/MobileArmTaskPlanner.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/PcaGraspPlanner.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/PlannerFactory.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/PointCloudHandler.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/SimpleGripperVisualizer.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/StateMachine.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/TaskPositionWaypoint.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/TaskVelocityWaypoint.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/WaypointExecutor.cpp
  ${COMMON_SOURCES}
)

add_executable(${PROJECT_NAME} ${SOURCES})

include_directories(${PROJECT_NAME}
    ${CMAKE_CURRENT_LIST_DIR}/../common/include
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/dependencies/plog/plog-master/include
    ${CMAKE_CURRENT_LIST_DIR}/dependencies/ruckig/include
    ${PCL_INCLUDE_DIRS}
    /usr/local/include
    /opt/ros/humble/include/kdl_parser
    /opt/ros/humble/include/urdf
)

set(COMMON_LIBS
  kdl_parser
  urdf
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  robot_idl
  pcl_msgs
  pcl_conversions
  ${COMMON_LIBS}
)

target_link_libraries(${PROJECT_NAME} yaml-cpp ruckig ${PCL_LIBRARIES} ${DYNAMIXEL_LIB})

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
ament_package()
