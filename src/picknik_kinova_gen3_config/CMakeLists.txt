cmake_minimum_required(VERSION 3.16.3)
project(picknik_kinova_gen3_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    calibration
    config
    description
    launch
    objectives
    waypoints
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
