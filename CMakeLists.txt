cmake_minimum_required(VERSION 3.8)
project(arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(BUILD_ARM_TOOLS "build tools for a.r.m module" ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(arm_idl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

set(PCL_DIR "/usr/local/pcl-1.15/lib/cmake/pcl")
find_package(PCL REQUIRED)

# Optional: disable Ruckig examples and tests
set(RUCKIG_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(RUCKIG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(dependencies/ruckig)

include(FetchContent)

FetchContent_Declare(
  simdjson
  GIT_REPOSITORY https://github.com/simdjson/simdjson.git
  GIT_TAG        master  # or a specific release tag
)

FetchContent_MakeAvailable(simdjson)

set(COMMON_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/src/ConfigManager.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/KinematicsHandler.cpp
)

set(SOURCES 
  ${CMAKE_CURRENT_LIST_DIR}/src/CommandHandler.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/GazeboManipComms.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/JointPositionWaypoint.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/Manipulator.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/ManipulatorFactory.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/MobileArmTaskPlanner.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/PcaGraspPlanner.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/PlannerFactory.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/PointCloudHandler.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/RateController.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/SimpleGripperVisualizer.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/StateMachine.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/TaskPositionWaypoint.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/WaypointExecutor.cpp
  ${COMMON_SOURCES}
)

add_executable(${PROJECT_NAME} ${SOURCES})

set(COMMON_INCLUDE_DIRS
  ${CMAKE_CURRENT_LIST_DIR}/include
  /opt/ros/humble/include/kdl_parser
  /opt/ros/humble/include/urdf
)

include_directories(${PROJECT_NAME} 
  ${COMMON_INCLUDE_DIRS}
  ${CMAKE_CURRENT_LIST_DIR}/dependencies/plog/plog-master/include
  ${CMAKE_CURRENT_LIST_DIR}/dependencies/ruckig/include
  ${PCL_INCLUDE_DIRS}
)

set(COMMON_LIBS
  kdl_parser
  urdf
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  arm_idl
  pcl_msgs
  pcl_conversions
  ${COMMON_LIBS}
)

target_link_libraries(${PROJECT_NAME} yaml-cpp simdjson ruckig ${PCL_LIBRARIES})

if(BUILD_ARM_TOOLS)
    message(STATUS "Building test tool executable")

    find_package(ament_index_cpp REQUIRED)
    find_package(nlohmann_json REQUIRED)

    set(TOOL_SOURCES
      ${CMAKE_CURRENT_LIST_DIR}/tools/src/IrmGenerator.cpp
      ${CMAKE_CURRENT_LIST_DIR}/tools/src/irm_generator.cpp
      ${COMMON_SOURCES}
    )

    set(TOOL_INCLUDE_DIRS
      tools/include
      /opt/ros/humble/include/ament_index_cpp
      ${COMMON_INCLUDE_DIRS}
    )

    set(TOOL_LIBS
      ${COMMON_LIBS}
    )

    add_executable(irm_generator ${TOOL_SOURCES})
    ament_target_dependencies(irm_generator ${TOOL_LIBS})
    target_link_libraries(irm_generator yaml-cpp nlohmann_json::nlohmann_json)
    target_include_directories(irm_generator PRIVATE ${TOOL_INCLUDE_DIRS})
else()
    message(STATUS "Not building test tool executable")
endif()


install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

if(BUILD_ARM_TOOLS)
  install(TARGETS irm_generator DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
