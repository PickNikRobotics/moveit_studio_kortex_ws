find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(
  test_load_behavior_plugins
  test_load_behavior_plugins.cpp
)
ament_target_dependencies(test_load_behavior_plugins ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_add_gtest(
  test_deactivate_controllers
  test_deactivate_controllers.cpp
)
target_link_libraries(test_deactivate_controllers kortex_behavior)
target_include_directories(test_deactivate_controllers
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>)

