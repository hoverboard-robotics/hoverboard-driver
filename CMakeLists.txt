cmake_minimum_required(VERSION 3.16)
project(hoverboard_driver  LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  control_toolbox
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  hoverboard_driver 
  SHARED
  hardware/hoverboard_driver.cpp
  hardware/pid.cpp
)
target_compile_features(hoverboard_driver  PUBLIC cxx_std_17)
target_include_directories(hoverboard_driver  PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/hoverboard_driver>
)

ament_target_dependencies(
  hoverboard_driver PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "HOVERBOARD_DRIVER_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface hoverboard_driver.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/hoverboard_driver 
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/hoverboard_driver 
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/hoverboard_driver 
)
install(TARGETS hoverboard_driver 
  EXPORT export_hoverboard_driver 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_hoverboard_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()