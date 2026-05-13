cmake_minimum_required(VERSION 3.8)
project(arm_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/ConfigManager.cpp
  src/Logger.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} PUBLIC
  yaml-cpp
  rclcpp::rclcpp
)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}_targets
        LIBRARY DESTINATION lib)

install(DIRECTORY include/
        DESTINATION include)

ament_export_targets(${PROJECT_NAME}_targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp yaml-cpp)
ament_package()
