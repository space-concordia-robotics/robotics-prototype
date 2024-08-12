cmake_minimum_required(VERSION 3.5)
project(absenc_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
# find_package(arm_controller REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EncoderValues.msg"
 )

add_executable(absenc_node src/absenc_node.cpp src/absenc.cpp)
rosidl_target_interfaces(absenc_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(absenc_node rclcpp std_msgs sensor_msgs lifecycle_msgs rclcpp_lifecycle)

install(TARGETS
  absenc_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
