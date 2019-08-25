# Install script for directory: /home/ali/Programming/robotics-prototype/robot/rospackages/src/tf2_web_republisher

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ali/Programming/robotics-prototype/robot/rospackages/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tf2_web_republisher/action" TYPE FILE FILES "/home/ali/Programming/robotics-prototype/robot/rospackages/src/tf2_web_republisher/action/TFSubscription.action")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tf2_web_republisher/msg" TYPE FILE FILES
    "/home/ali/Programming/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionAction.msg"
    "/home/ali/Programming/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionGoal.msg"
    "/home/ali/Programming/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionResult.msg"
    "/home/ali/Programming/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionFeedback.msg"
    "/home/ali/Programming/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg"
    "/home/ali/Programming/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg"
    "/home/ali/Programming/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tf2_web_republisher/services" TYPE FILE FILES "/home/ali/Programming/robotics-prototype/robot/rospackages/src/tf2_web_republisher/services/RepublishTFs.srv")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tf2_web_republisher/msg" TYPE FILE FILES "/home/ali/Programming/robotics-prototype/robot/rospackages/src/tf2_web_republisher/msg/TFArray.msg")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tf2_web_republisher/cmake" TYPE FILE FILES "/home/ali/Programming/robotics-prototype/robot/rospackages/build/tf2_web_republisher/catkin_generated/installspace/tf2_web_republisher-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ali/Programming/robotics-prototype/robot/rospackages/devel/include/tf2_web_republisher")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/ali/Programming/robotics-prototype/robot/rospackages/devel/share/roseus/ros/tf2_web_republisher")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ali/Programming/robotics-prototype/robot/rospackages/devel/share/common-lisp/ros/tf2_web_republisher")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/ali/Programming/robotics-prototype/robot/rospackages/devel/share/gennodejs/ros/tf2_web_republisher")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/ali/Programming/robotics-prototype/robot/rospackages/devel/lib/python2.7/dist-packages/tf2_web_republisher")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/ali/Programming/robotics-prototype/robot/rospackages/devel/lib/python2.7/dist-packages/tf2_web_republisher")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ali/Programming/robotics-prototype/robot/rospackages/build/tf2_web_republisher/catkin_generated/installspace/tf2_web_republisher.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tf2_web_republisher/cmake" TYPE FILE FILES "/home/ali/Programming/robotics-prototype/robot/rospackages/build/tf2_web_republisher/catkin_generated/installspace/tf2_web_republisher-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tf2_web_republisher/cmake" TYPE FILE FILES
    "/home/ali/Programming/robotics-prototype/robot/rospackages/build/tf2_web_republisher/catkin_generated/installspace/tf2_web_republisherConfig.cmake"
    "/home/ali/Programming/robotics-prototype/robot/rospackages/build/tf2_web_republisher/catkin_generated/installspace/tf2_web_republisherConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/tf2_web_republisher" TYPE FILE FILES "/home/ali/Programming/robotics-prototype/robot/rospackages/src/tf2_web_republisher/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tf2_web_republisher/tf2_web_republisher" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tf2_web_republisher/tf2_web_republisher")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tf2_web_republisher/tf2_web_republisher"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/tf2_web_republisher" TYPE EXECUTABLE FILES "/home/ali/Programming/robotics-prototype/robot/rospackages/devel/lib/tf2_web_republisher/tf2_web_republisher")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tf2_web_republisher/tf2_web_republisher" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tf2_web_republisher/tf2_web_republisher")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tf2_web_republisher/tf2_web_republisher"
         OLD_RPATH "/opt/ros/kinetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/tf2_web_republisher/tf2_web_republisher")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/tf2_web_republisher" TYPE DIRECTORY FILES "/home/ali/Programming/robotics-prototype/robot/rospackages/src/tf2_web_republisher/include/" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.hpp$")
endif()

