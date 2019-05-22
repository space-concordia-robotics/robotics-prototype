cmake_minimum_required(VERSION 2.8.3)
project(arm_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ArmRequest.srv
)

add_message_files(
  FILES
  IkCommand.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mux_selector
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
