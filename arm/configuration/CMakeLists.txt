cmake_minimum_required(VERSION 3.8)
project(arm_configs)

# find dependencies
find_package(ament_cmake REQUIRED)

install(FILES config.yaml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY manipulators/ DESTINATION share/${PROJECT_NAME}/manipulators)

ament_package()
