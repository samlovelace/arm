cmake_minimum_required(VERSION 3.8)
project(arm_engine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_idl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED) 
find_package(kdl_parser REQUIRED)
find_package(plog REQUIRED)

set(COMMON_SOURCES
  ${CMAKE_CURRENT_LIST_DIR}/../common/src/ConfigManager.cpp
)

set(SOURCES
    ${COMMON_SOURCES}
    ${CMAKE_CURRENT_LIST_DIR}/src/Engine.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}/../common/include
    ${CMAKE_CURRENT_LIST_DIR}/include
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    robot_idl
    kdl_parser
)

target_link_libraries(${PROJECT_NAME} yaml-cpp)

install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
ament_package()
