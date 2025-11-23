cmake_minimum_required(VERSION 3.8)
project(arm_engine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# fix intermeittent segfault in manipulability computation
add_compile_options(-mno-avx512f)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_idl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED) 
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(plog REQUIRED)

find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

set(PCL_DIR "/usr/local/pcl-1.15/lib/cmake/pcl")
find_package(PCL REQUIRED)

set(COMMON_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/../common/src/ConfigManager.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../common/src/KinematicsHandler.cpp
)

set(SOURCES
    ${COMMON_SOURCES}
    ${CMAKE_CURRENT_LIST_DIR}/src/Engine.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/PcaGraspPlanner.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/PlanGraspNode.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/PlannerFactory.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/PlanPickTaskNode.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/PointCloudHandler.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/SequenceNode.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/SimpleGripperVisualizer.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/SimpleMobileTaskPlanner.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}/../common/include
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    robot_idl
    kdl_parser
    urdf
    pcl_msgs
    pcl_conversions
)

target_link_libraries(${PROJECT_NAME} yaml-cpp ${PCL_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
ament_package()
