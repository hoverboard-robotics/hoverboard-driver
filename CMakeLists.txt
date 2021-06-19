cmake_minimum_required(VERSION 2.8.3)
project(hoverboard_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  hardware_interface
  controller_manager
  dynamic_reconfigure
  control_toolbox
  rosparam_shortcuts
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(
  config/Hoverboard.cfg
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
 include
 include/hoverboard_driver
  ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} 
  src/hoverboard.cpp 
  src/pid.cpp
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(hoverboard_driver ${PROJECT_NAME}_gencfg)

