cmake_minimum_required(VERSION 3.8)
project(gps_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geographic_msgs REQUIRED)

add_executable(rover_localization src/gps_pub.cpp)
ament_target_dependencies(rover_localization rclcpp geographic_msgs)

install(TARGETS 

  rover_localization

  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
