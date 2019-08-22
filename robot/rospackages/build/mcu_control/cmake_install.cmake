# Install script for directory: /home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/mcu_control

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mcu_control/srv" TYPE FILE FILES
    "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/mcu_control/srv/ArmRequest.srv"
    "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/mcu_control/srv/ScienceRequest.srv"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mcu_control/msg" TYPE FILE FILES
    "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/mcu_control/msg/IkCommand.msg"
    "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/mcu_control/msg/AntennaGoal.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mcu_control/cmake" TYPE FILE FILES "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/build/mcu_control/catkin_generated/installspace/mcu_control-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/include/mcu_control")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/roseus/ros/mcu_control")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/common-lisp/ros/mcu_control")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/gennodejs/ros/mcu_control")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/home/vashmata/Programming/git/robotics-prototype/venv/bin/python" -m compileall "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/lib/python3/dist-packages/mcu_control")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/lib/python3/dist-packages/mcu_control")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/build/mcu_control/catkin_generated/installspace/mcu_control.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mcu_control/cmake" TYPE FILE FILES "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/build/mcu_control/catkin_generated/installspace/mcu_control-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mcu_control/cmake" TYPE FILE FILES
    "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/build/mcu_control/catkin_generated/installspace/mcu_controlConfig.cmake"
    "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/build/mcu_control/catkin_generated/installspace/mcu_controlConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mcu_control" TYPE FILE FILES "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/mcu_control/package.xml")
endif()

