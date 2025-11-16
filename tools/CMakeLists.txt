cmake_minimum_required(VERSION 3.1)

message(STATUS "Building test tool executable")

find_package(ament_index_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

set(TOOL_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/IrmGenerator.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/irm_generator.cpp
    ${COMMON_SOURCES}
)

set(TOOL_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/include
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

install(TARGETS irm_generator DESTINATION lib/${PROJECT_NAME})
