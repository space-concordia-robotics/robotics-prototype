cmake_minimum_required(VERSION 3.5)
project(gps)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)


if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

add_executable(gps_node 
        src/gpsPublisherNode.cpp 
        src/gps2.cpp)
ament_target_dependencies(gps_node rclcpp std_msgs sensor_msgs lifecycle_msgs rclcpp_lifecycle)

add_executable(gps_serviceclient_node
        src/gpsServiceClientNode.cpp)
ament_target_dependencies(gps_serviceclient_node rclcpp std_msgs sensor_msgs lifecycle_msgs rclcpp_lifecycle)

install(TARGETS
        gps_node
        gps_serviceclient_node
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()