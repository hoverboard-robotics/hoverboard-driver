cmake_minimum_required(VERSION 2.8.3)
project(hoverboard_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  hardware_interface
  controller_manager
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

generate_dynamic_reconfigure_options(
  config/Hoverboard.cfg
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES hoverboard_driver
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
 include/hoverboard_driver
 src/bipropellant/src
 src/bipropellant/src/hbprotocol
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} 
  src/hoverboard.cpp 
  src/main.cpp
  src/bipropellant/src/HoverboardAPI.cpp
  src/bipropellant/src/protocolFunctions.c
  src/bipropellant/src/hbprotocol/ascii_protocol.c
  src/bipropellant/src/hbprotocol/protocol.c
  src/bipropellant/src/hbprotocol/machine_protocol.c
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(hoverboard_driver ${PROJECT_NAME}_gencfg)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/hoverboard_driver.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

