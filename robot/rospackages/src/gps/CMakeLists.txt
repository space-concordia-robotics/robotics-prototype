cmake_minimum_required(VERSION 3.8)
project(gps)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

set(GPS_NODE_DIR "${CMAKE_SOURCE_DIR}/../../../rover/gps")
add_subdirectory(${GPS_NODE_DIR} ${CMAKE_BINARY_DIR}/gps_build)

add_executable(gps_publisher_node src/GpsPublisherNode.cpp)
ament_target_dependencies(gps_publisher_node rclcpp std_msgs sensor_msgs)

target_link_libraries(gps_publisher_node gps_node)

install(TARGETS
        gps_publisher_node
        DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
