
cmake_minimum_required(VERSION 3.5)
project(wheels_controller)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(builtin_interfaces REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#rosidl_generate_interfaces(${PROJECT_NAME}
##        "msg/ArmMotors.msg"
#        DEPENDENCIES std_msgs
#)
include_directories(include)

add_executable(wheels_controller_node 
    src/wheels_controller_node.cpp
    src/rev_motor_controller.cpp
    src/can_controller.cpp
    src/can_controller.h
    src/rev_motor_controller.h
    src/wheels_controller_node.h
)

#rosidl_target_interfaces(wheels_controller_node
#        ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(wheels_controller_node rclcpp geometry_msgs nav_msgs nav2_msgs sensor_msgs builtin_interfaces action_msgs std_msgs lifecycle_msgs rclcpp_lifecycle)

install(TARGETS
        wheels_controller_node
        DESTINATION lib/${PROJECT_NAME})
ament_package()