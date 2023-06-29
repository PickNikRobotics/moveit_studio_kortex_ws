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
find_package(std_msgs REQUIRED)

# Set includes
set(THIS_PACKAGE_INCLUDE_DEPENDS
    behaviortree_cpp
    fp
    moveit_studio_behavior_interface
    pluginlib
    rclcpp
    std_msgs
)

###########
## Build ##
###########

# Add libraries
add_library(gibson_behavior SHARED
  src/detect_objects.cpp
  src/get_object_3d_from_detections.cpp
  src/get_cabinet_door_from_detections.cpp
  src/get_cuboid_from_detections.cpp
  src/get_lever_handle_from_detections.cpp
  src/get_button_from_detections.cpp
  src/register_behaviors.cpp)
target_include_directories(gibson_behavior
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>
  PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(gibson_behavior ${PCL_LIBRARIES})
ament_target_dependencies(gibson_behavior ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Add executables
add_executable(kortex_action_client src/kortex_action_client.cpp)
ament_target_dependencies(kortex_action_client ${THIS_PACKAGE_INCLUDE_DEPENDS})

###########
# Install #
###########

# Install Libraries
install(
  TARGETS gibson_behavior
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

# Install Executables
install(TARGETS
  kortex_action_client
  DESTINATION lib/${PROJECT_NAME})

###########
# Testing #
###########

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  add_subdirectory(test)
  #ament_lint_auto_find_test_dependencies()
endif()

##########
# Export #
##########

pluginlib_export_plugin_description_file(moveit_studio_behavior_interface kortex_behavior_plugin_description.xml)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
