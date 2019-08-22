# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "arm_control: 2 messages, 2 services")

set(MSG_I_FLAGS "-Iarm_control:/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(arm_control_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ArmRequest.srv" NAME_WE)
add_custom_target(_arm_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arm_control" "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ArmRequest.srv" ""
)

get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ScienceRequest.srv" NAME_WE)
add_custom_target(_arm_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arm_control" "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ScienceRequest.srv" ""
)

get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/IkCommand.msg" NAME_WE)
add_custom_target(_arm_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arm_control" "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/IkCommand.msg" ""
)

get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/AntennaGoal.msg" NAME_WE)
add_custom_target(_arm_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "arm_control" "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/AntennaGoal.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(arm_control
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/IkCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_control
)
_generate_msg_cpp(arm_control
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/AntennaGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_control
)

### Generating Services
_generate_srv_cpp(arm_control
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ArmRequest.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_control
)
_generate_srv_cpp(arm_control
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ScienceRequest.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_control
)

### Generating Module File
_generate_module_cpp(arm_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(arm_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(arm_control_generate_messages arm_control_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ArmRequest.srv" NAME_WE)
add_dependencies(arm_control_generate_messages_cpp _arm_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ScienceRequest.srv" NAME_WE)
add_dependencies(arm_control_generate_messages_cpp _arm_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/IkCommand.msg" NAME_WE)
add_dependencies(arm_control_generate_messages_cpp _arm_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/AntennaGoal.msg" NAME_WE)
add_dependencies(arm_control_generate_messages_cpp _arm_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arm_control_gencpp)
add_dependencies(arm_control_gencpp arm_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arm_control_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(arm_control
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/IkCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_control
)
_generate_msg_eus(arm_control
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/AntennaGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_control
)

### Generating Services
_generate_srv_eus(arm_control
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ArmRequest.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_control
)
_generate_srv_eus(arm_control
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ScienceRequest.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_control
)

### Generating Module File
_generate_module_eus(arm_control
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_control
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(arm_control_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(arm_control_generate_messages arm_control_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ArmRequest.srv" NAME_WE)
add_dependencies(arm_control_generate_messages_eus _arm_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ScienceRequest.srv" NAME_WE)
add_dependencies(arm_control_generate_messages_eus _arm_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/IkCommand.msg" NAME_WE)
add_dependencies(arm_control_generate_messages_eus _arm_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/AntennaGoal.msg" NAME_WE)
add_dependencies(arm_control_generate_messages_eus _arm_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arm_control_geneus)
add_dependencies(arm_control_geneus arm_control_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arm_control_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(arm_control
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/IkCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_control
)
_generate_msg_lisp(arm_control
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/AntennaGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_control
)

### Generating Services
_generate_srv_lisp(arm_control
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ArmRequest.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_control
)
_generate_srv_lisp(arm_control
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ScienceRequest.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_control
)

### Generating Module File
_generate_module_lisp(arm_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(arm_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(arm_control_generate_messages arm_control_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ArmRequest.srv" NAME_WE)
add_dependencies(arm_control_generate_messages_lisp _arm_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ScienceRequest.srv" NAME_WE)
add_dependencies(arm_control_generate_messages_lisp _arm_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/IkCommand.msg" NAME_WE)
add_dependencies(arm_control_generate_messages_lisp _arm_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/AntennaGoal.msg" NAME_WE)
add_dependencies(arm_control_generate_messages_lisp _arm_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arm_control_genlisp)
add_dependencies(arm_control_genlisp arm_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arm_control_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(arm_control
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/IkCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_control
)
_generate_msg_nodejs(arm_control
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/AntennaGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_control
)

### Generating Services
_generate_srv_nodejs(arm_control
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ArmRequest.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_control
)
_generate_srv_nodejs(arm_control
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ScienceRequest.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_control
)

### Generating Module File
_generate_module_nodejs(arm_control
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_control
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(arm_control_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(arm_control_generate_messages arm_control_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ArmRequest.srv" NAME_WE)
add_dependencies(arm_control_generate_messages_nodejs _arm_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ScienceRequest.srv" NAME_WE)
add_dependencies(arm_control_generate_messages_nodejs _arm_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/IkCommand.msg" NAME_WE)
add_dependencies(arm_control_generate_messages_nodejs _arm_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/AntennaGoal.msg" NAME_WE)
add_dependencies(arm_control_generate_messages_nodejs _arm_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arm_control_gennodejs)
add_dependencies(arm_control_gennodejs arm_control_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arm_control_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(arm_control
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/IkCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_control
)
_generate_msg_py(arm_control
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/AntennaGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_control
)

### Generating Services
_generate_srv_py(arm_control
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ArmRequest.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_control
)
_generate_srv_py(arm_control
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ScienceRequest.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_control
)

### Generating Module File
_generate_module_py(arm_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(arm_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(arm_control_generate_messages arm_control_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ArmRequest.srv" NAME_WE)
add_dependencies(arm_control_generate_messages_py _arm_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/srv/ScienceRequest.srv" NAME_WE)
add_dependencies(arm_control_generate_messages_py _arm_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/IkCommand.msg" NAME_WE)
add_dependencies(arm_control_generate_messages_py _arm_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/arm_control/msg/AntennaGoal.msg" NAME_WE)
add_dependencies(arm_control_generate_messages_py _arm_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(arm_control_genpy)
add_dependencies(arm_control_genpy arm_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS arm_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/arm_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(arm_control_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(arm_control_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(arm_control_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/arm_control
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(arm_control_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(arm_control_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(arm_control_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/arm_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(arm_control_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(arm_control_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(arm_control_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/arm_control
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(arm_control_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(arm_control_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(arm_control_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_control)
  install(CODE "execute_process(COMMAND \"/home/vashmata/Programming/git/robotics-prototype/venv/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/arm_control
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(arm_control_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(arm_control_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(arm_control_generate_messages_py geometry_msgs_generate_messages_py)
endif()
