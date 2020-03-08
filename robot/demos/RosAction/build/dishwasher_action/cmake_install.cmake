# Install script for directory: /home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/src/dishwasher_action

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dishwasher_action/action" TYPE FILE FILES "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/src/dishwasher_action/action/Dishwasher.action")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dishwasher_action/msg" TYPE FILE FILES
    "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/share/dishwasher_action/msg/DishwasherAction.msg"
    "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/share/dishwasher_action/msg/DishwasherActionGoal.msg"
    "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/share/dishwasher_action/msg/DishwasherActionResult.msg"
    "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/share/dishwasher_action/msg/DishwasherActionFeedback.msg"
    "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/share/dishwasher_action/msg/DishwasherGoal.msg"
    "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/share/dishwasher_action/msg/DishwasherResult.msg"
    "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/share/dishwasher_action/msg/DishwasherFeedback.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dishwasher_action/cmake" TYPE FILE FILES "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/build/dishwasher_action/catkin_generated/installspace/dishwasher_action-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/include/dishwasher_action")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/share/roseus/ros/dishwasher_action")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/share/common-lisp/ros/dishwasher_action")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/share/gennodejs/ros/dishwasher_action")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/lib/python2.7/dist-packages/dishwasher_action")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/lib/python2.7/dist-packages/dishwasher_action")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/build/dishwasher_action/catkin_generated/installspace/dishwasher_action.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dishwasher_action/cmake" TYPE FILE FILES "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/build/dishwasher_action/catkin_generated/installspace/dishwasher_action-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dishwasher_action/cmake" TYPE FILE FILES
    "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/build/dishwasher_action/catkin_generated/installspace/dishwasher_actionConfig.cmake"
    "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/build/dishwasher_action/catkin_generated/installspace/dishwasher_actionConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dishwasher_action" TYPE FILE FILES "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/src/dishwasher_action/package.xml")
endif()

