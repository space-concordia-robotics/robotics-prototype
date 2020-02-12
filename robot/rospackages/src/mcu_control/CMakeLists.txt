cmake_minimum_required(VERSION 2.8.3)
project(mcu_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ## all
  rospy
  std_msgs
  message_generation
  sensor_msgs
  geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ## arm
  ArmRequest.srv
  ScienceRequest.srv
)

add_message_files(
  FILES
  ## arm
  IkCommand.msg
  ## rover
  RoverPosition.msg
  AntennaGoal.msg
  RoverGoal.msg
  BatteryTemps.msg
  BatteryVoltage.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  ## all
  std_msgs
  sensor_msgs
  geometry_msgs
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
