cmake_minimum_required(VERSION 3.5)
project(gps)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

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
set(HEADER_FILES
        src/packet/UbxPacket.hpp
        src/serdes/UbxSerializer.hpp
        src/serdes/UbxDeserializer.hpp
        src/connection/UbxAbstractConnection.hpp
        src/connection/UbxI2CConnection.hpp
        src/GpsPublisherNode.h
)

set(SOURCE_FILES
        src/packet/UbxPacket.cpp
        #        src/gps3.cpp
        src/serdes/UbxSerializer.cpp
        src/serdes/UbxDeserializer.cpp
        src/connection/UbxAbstractConnection.cpp
        src/connection/UbxI2CConnection.cpp
        src/connection/I2CTransactionBuilder.cpp
        src/packet/UbxPacketNavSat.cpp
        src/GpsPublisherNode.cpp
)

add_library(gps_node STATIC ${SOURCE_FILES} ${HEADER_FILES})
#add_executable(gps_node src/gpsPublisherNode.cpp)


ament_target_dependencies(gps_node rclcpp std_msgs sensor_msgs)
install(TARGETS
        gps_node
        DESTINATION lib/${PROJECT_NAME}
)
include_directories(src)


ament_package()

add_subdirectory(Google_tests)
