cmake_minimum_required(VERSION 3.0.2)
project(bug_power)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  base_mcu_interface
  rospy
  sensor_msgs
)

catkin_python_setup()

catkin_package(
 INCLUDE_DIRS 
 LIBRARIES bug_power
 CATKIN_DEPENDS rospy sensor_msgs
 DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  nodes/bug_power_node.py
  nodes/power_driver_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

