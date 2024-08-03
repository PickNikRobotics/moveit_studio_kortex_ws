cmake_minimum_required(VERSION 3.22)
project(set_dynamixel_velocity CXX)

find_package(moveit_studio_common REQUIRED)
moveit_studio_package()

find_package(dynamixel_control_msgs REQUIRED)
find_package(moveit_studio_behavior_interface REQUIRED)
find_package(pluginlib REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS dynamixel_control_msgs moveit_studio_behavior_interface pluginlib)

add_library(
  set_dynamixel_velocity
  SHARED
  src/set_dynamixel_velocity.cpp
  src/register_behaviors.cpp)
target_include_directories(
  set_dynamixel_velocity
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(set_dynamixel_velocity
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install Libraries
install(
  TARGETS set_dynamixel_velocity
  EXPORT set_dynamixel_velocityTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Export the behavior plugins defined in this package so they are available to
# plugin loaders that load the behavior base class library from the
# moveit_studio_behavior package.
pluginlib_export_plugin_description_file(
  moveit_studio_behavior_interface set_dynamixel_velocity_plugin_description.xml)

ament_export_targets(set_dynamixel_velocityTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
