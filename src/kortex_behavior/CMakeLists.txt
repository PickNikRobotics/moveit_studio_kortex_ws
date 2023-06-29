cmake_minimum_required(VERSION 3.16)
project(kortex_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Needed to pass Studio CI clang-tidy checks
find_package(moveit_common REQUIRED)
moveit_package()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(fp REQUIRED)
find_package(moveit_studio_behavior_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

# Set includes
set(THIS_PACKAGE_INCLUDE_DEPENDS
    behaviortree_cpp
    fp
    moveit_studio_behavior_interface
    pluginlib
    rclcpp
    geometry_msgs
    control_msgs
    controller_manager_msgs
)

###########
## Build ##
###########

# Add libraries
add_library(kortex_behavior SHARED
  src/deactivate_controllers.cpp
  src/teleoperate_non_servo.cpp)

target_include_directories(kortex_behavior
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>
  PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(kortex_behavior ${PCL_LIBRARIES})
ament_target_dependencies(kortex_behavior ${THIS_PACKAGE_INCLUDE_DEPENDS})

###########
# Install #
###########

# Install Libraries
install(
  TARGETS kortex_behavior
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

###########
# Testing(TODO) #
###########


##########
# Export #
##########

pluginlib_export_plugin_description_file(moveit_studio_behavior_interface kortex_behavior_plugin_description.xml)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
