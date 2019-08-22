# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tf2_web_republisher: 8 messages, 1 services")

set(MSG_I_FLAGS "-Itf2_web_republisher:/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg;-Itf2_web_republisher:/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Iroscpp:/opt/ros/kinetic/share/roscpp/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tf2_web_republisher_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionResult.msg" NAME_WE)
add_custom_target(_tf2_web_republisher_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tf2_web_republisher" "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionResult.msg" "tf2_web_republisher/TFSubscriptionResult:std_msgs/Header:actionlib_msgs/GoalStatus:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg" NAME_WE)
add_custom_target(_tf2_web_republisher_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tf2_web_republisher" "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg" ""
)

get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/msg/TFArray.msg" NAME_WE)
add_custom_target(_tf2_web_republisher_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tf2_web_republisher" "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/msg/TFArray.msg" "geometry_msgs/Vector3:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Transform:geometry_msgs/TransformStamped"
)

get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionAction.msg" NAME_WE)
add_custom_target(_tf2_web_republisher_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tf2_web_republisher" "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionAction.msg" "geometry_msgs/Vector3:std_msgs/Header:tf2_web_republisher/TFSubscriptionResult:geometry_msgs/Quaternion:actionlib_msgs/GoalStatus:geometry_msgs/TransformStamped:tf2_web_republisher/TFSubscriptionActionResult:tf2_web_republisher/TFSubscriptionActionFeedback:tf2_web_republisher/TFSubscriptionFeedback:tf2_web_republisher/TFSubscriptionGoal:geometry_msgs/Transform:tf2_web_republisher/TFSubscriptionActionGoal:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/services/RepublishTFs.srv" NAME_WE)
add_custom_target(_tf2_web_republisher_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tf2_web_republisher" "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/services/RepublishTFs.srv" ""
)

get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionGoal.msg" NAME_WE)
add_custom_target(_tf2_web_republisher_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tf2_web_republisher" "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionGoal.msg" "std_msgs/Header:tf2_web_republisher/TFSubscriptionGoal:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg" NAME_WE)
add_custom_target(_tf2_web_republisher_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tf2_web_republisher" "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg" "geometry_msgs/Vector3:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Transform:geometry_msgs/TransformStamped"
)

get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg" NAME_WE)
add_custom_target(_tf2_web_republisher_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tf2_web_republisher" "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg" ""
)

get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionFeedback.msg" NAME_WE)
add_custom_target(_tf2_web_republisher_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tf2_web_republisher" "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionFeedback.msg" "geometry_msgs/Vector3:std_msgs/Header:geometry_msgs/Quaternion:actionlib_msgs/GoalStatus:geometry_msgs/TransformStamped:tf2_web_republisher/TFSubscriptionFeedback:geometry_msgs/Transform:actionlib_msgs/GoalID"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_cpp(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_cpp(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/msg/TFArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_cpp(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionResult.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionFeedback.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_cpp(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_cpp(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_cpp(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_cpp(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tf2_web_republisher
)

### Generating Services
_generate_srv_cpp(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/services/RepublishTFs.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tf2_web_republisher
)

### Generating Module File
_generate_module_cpp(tf2_web_republisher
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tf2_web_republisher
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tf2_web_republisher_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tf2_web_republisher_generate_messages tf2_web_republisher_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionResult.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_cpp _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_cpp _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/msg/TFArray.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_cpp _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionAction.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_cpp _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/services/RepublishTFs.srv" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_cpp _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionGoal.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_cpp _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_cpp _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_cpp _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionFeedback.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_cpp _tf2_web_republisher_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tf2_web_republisher_gencpp)
add_dependencies(tf2_web_republisher_gencpp tf2_web_republisher_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tf2_web_republisher_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_eus(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_eus(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/msg/TFArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_eus(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionResult.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionFeedback.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_eus(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_eus(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_eus(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_eus(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tf2_web_republisher
)

### Generating Services
_generate_srv_eus(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/services/RepublishTFs.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tf2_web_republisher
)

### Generating Module File
_generate_module_eus(tf2_web_republisher
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tf2_web_republisher
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tf2_web_republisher_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tf2_web_republisher_generate_messages tf2_web_republisher_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionResult.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_eus _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_eus _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/msg/TFArray.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_eus _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionAction.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_eus _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/services/RepublishTFs.srv" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_eus _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionGoal.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_eus _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_eus _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_eus _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionFeedback.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_eus _tf2_web_republisher_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tf2_web_republisher_geneus)
add_dependencies(tf2_web_republisher_geneus tf2_web_republisher_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tf2_web_republisher_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_lisp(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_lisp(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/msg/TFArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_lisp(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionResult.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionFeedback.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_lisp(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_lisp(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_lisp(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_lisp(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tf2_web_republisher
)

### Generating Services
_generate_srv_lisp(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/services/RepublishTFs.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tf2_web_republisher
)

### Generating Module File
_generate_module_lisp(tf2_web_republisher
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tf2_web_republisher
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tf2_web_republisher_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tf2_web_republisher_generate_messages tf2_web_republisher_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionResult.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_lisp _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_lisp _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/msg/TFArray.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_lisp _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionAction.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_lisp _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/services/RepublishTFs.srv" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_lisp _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionGoal.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_lisp _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_lisp _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_lisp _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionFeedback.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_lisp _tf2_web_republisher_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tf2_web_republisher_genlisp)
add_dependencies(tf2_web_republisher_genlisp tf2_web_republisher_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tf2_web_republisher_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_nodejs(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_nodejs(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/msg/TFArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_nodejs(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionResult.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionFeedback.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_nodejs(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_nodejs(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_nodejs(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_nodejs(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tf2_web_republisher
)

### Generating Services
_generate_srv_nodejs(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/services/RepublishTFs.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tf2_web_republisher
)

### Generating Module File
_generate_module_nodejs(tf2_web_republisher
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tf2_web_republisher
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tf2_web_republisher_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tf2_web_republisher_generate_messages tf2_web_republisher_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionResult.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_nodejs _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_nodejs _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/msg/TFArray.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_nodejs _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionAction.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_nodejs _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/services/RepublishTFs.srv" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_nodejs _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionGoal.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_nodejs _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_nodejs _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_nodejs _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionFeedback.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_nodejs _tf2_web_republisher_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tf2_web_republisher_gennodejs)
add_dependencies(tf2_web_republisher_gennodejs tf2_web_republisher_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tf2_web_republisher_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionResult.msg"
  "${MSG_I_FLAGS}"
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_py(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_py(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/msg/TFArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_py(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionResult.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionFeedback.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_py(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_py(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_py(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tf2_web_republisher
)
_generate_msg_py(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/TransformStamped.msg;/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tf2_web_republisher
)

### Generating Services
_generate_srv_py(tf2_web_republisher
  "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/services/RepublishTFs.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tf2_web_republisher
)

### Generating Module File
_generate_module_py(tf2_web_republisher
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tf2_web_republisher
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tf2_web_republisher_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tf2_web_republisher_generate_messages tf2_web_republisher_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionResult.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_py _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionGoal.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_py _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/msg/TFArray.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_py _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionAction.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_py _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/src/tf2_web_republisher/services/RepublishTFs.srv" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_py _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionGoal.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_py _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionFeedback.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_py _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionResult.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_py _tf2_web_republisher_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vashmata/Programming/git/robotics-prototype/robot/rospackages/devel/share/tf2_web_republisher/msg/TFSubscriptionActionFeedback.msg" NAME_WE)
add_dependencies(tf2_web_republisher_generate_messages_py _tf2_web_republisher_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tf2_web_republisher_genpy)
add_dependencies(tf2_web_republisher_genpy tf2_web_republisher_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tf2_web_republisher_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tf2_web_republisher)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tf2_web_republisher
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(tf2_web_republisher_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tf2_web_republisher_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(tf2_web_republisher_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET roscpp_generate_messages_cpp)
  add_dependencies(tf2_web_republisher_generate_messages_cpp roscpp_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tf2_web_republisher)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tf2_web_republisher
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(tf2_web_republisher_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tf2_web_republisher_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(tf2_web_republisher_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET roscpp_generate_messages_eus)
  add_dependencies(tf2_web_republisher_generate_messages_eus roscpp_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tf2_web_republisher)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tf2_web_republisher
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(tf2_web_republisher_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tf2_web_republisher_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(tf2_web_republisher_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET roscpp_generate_messages_lisp)
  add_dependencies(tf2_web_republisher_generate_messages_lisp roscpp_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tf2_web_republisher)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tf2_web_republisher
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(tf2_web_republisher_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tf2_web_republisher_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(tf2_web_republisher_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET roscpp_generate_messages_nodejs)
  add_dependencies(tf2_web_republisher_generate_messages_nodejs roscpp_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tf2_web_republisher)
  install(CODE "execute_process(COMMAND \"/home/vashmata/Programming/git/robotics-prototype/venv/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tf2_web_republisher\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tf2_web_republisher
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(tf2_web_republisher_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tf2_web_republisher_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(tf2_web_republisher_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET roscpp_generate_messages_py)
  add_dependencies(tf2_web_republisher_generate_messages_py roscpp_generate_messages_py)
endif()
