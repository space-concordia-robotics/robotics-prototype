# Install script for directory: /home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/src/actionlib_tutorials

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_tutorials/action" TYPE FILE FILES "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/src/actionlib_tutorials/action/Fibonacci.action")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_tutorials/msg" TYPE FILE FILES
    "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/share/actionlib_tutorials/msg/FibonacciAction.msg"
    "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/share/actionlib_tutorials/msg/FibonacciActionGoal.msg"
    "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/share/actionlib_tutorials/msg/FibonacciActionResult.msg"
    "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/share/actionlib_tutorials/msg/FibonacciActionFeedback.msg"
    "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/share/actionlib_tutorials/msg/FibonacciGoal.msg"
    "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/share/actionlib_tutorials/msg/FibonacciResult.msg"
    "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/share/actionlib_tutorials/msg/FibonacciFeedback.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_tutorials/cmake" TYPE FILE FILES "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/build/actionlib_tutorials/catkin_generated/installspace/actionlib_tutorials-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/include/actionlib_tutorials")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/share/roseus/ros/actionlib_tutorials")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/share/common-lisp/ros/actionlib_tutorials")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/share/gennodejs/ros/actionlib_tutorials")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/lib/python2.7/dist-packages/actionlib_tutorials")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/devel/lib/python2.7/dist-packages/actionlib_tutorials")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/build/actionlib_tutorials/catkin_generated/installspace/actionlib_tutorials.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_tutorials/cmake" TYPE FILE FILES "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/build/actionlib_tutorials/catkin_generated/installspace/actionlib_tutorials-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_tutorials/cmake" TYPE FILE FILES
    "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/build/actionlib_tutorials/catkin_generated/installspace/actionlib_tutorialsConfig.cmake"
    "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/build/actionlib_tutorials/catkin_generated/installspace/actionlib_tutorialsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/actionlib_tutorials" TYPE FILE FILES "/home/koa/Desktop/ConcordiaRobotics/robotics-prototype/robot/demos/RosAction/src/actionlib_tutorials/package.xml")
endif()

