
find_package(GTest REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(yaml-cpp REQUIRED)

set(CORE_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/arm/controller/src/DynamixelManipComms.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/arm/common/RateController.cpp
)

set(TEST_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/test/DynamixelManipComms_Test.cpp
    ${CMAKE_CURRENT_LIST_DIR}/test/RunAll.cpp
)

add_executable(test_arm ${TEST_SOURCES} ${CORE_SOURCES})
target_link_libraries(test_arm 
    gtest 
    ${DYNAMIXEL_LIB}
    kdl_parser::kdl_parser
    yaml-cpp
)

target_include_directories(test_arm PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
    /usr/local/include # installed location of dynamixel_sdk
    ${CMAKE_CURRENT_LIST_DIR}/dependencies/plog/plog-master/include
)
