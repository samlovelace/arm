cmake_minimum_required(VERSION 3.8)
project(arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(arm_idl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# Optional: disable Ruckig examples and tests
set(RUCKIG_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(RUCKIG_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(dependencies/ruckig)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(SOURCES 
  ${CMAKE_CURRENT_LIST_DIR}/src/CommandHandler.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/ConfigManager.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/GazeboManipComms.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/JointPositionWaypoint.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/KinematicsHandler.cpp
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
  )

add_executable(${PROJECT_NAME} ${SOURCES})

include_directories(${PROJECT_NAME} 
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${CMAKE_CURRENT_LIST_DIR}/dependencies/plog/plog-master/include
  ${CMAKE_CURRENT_LIST_DIR}/dependencies/ruckig/include
  /opt/ros/humble/include/kdl_parser
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  kdl_parser
  urdf
  std_msgs
  arm_idl
  pcl_msgs
  pcl_conversions
)

target_link_libraries(${PROJECT_NAME} yaml-cpp ruckig ${PCL_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_package()
