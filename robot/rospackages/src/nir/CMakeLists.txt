cmake_minimum_required(VERSION 3.5.2)
project(nir)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        actionlib_msgs
        message_generation
        std_msgs)


## Generate messages in the 'msg' folder
add_message_files(
        FILES
        scan_msg.msg
)


## Generate services in the 'srv' folder
add_service_files(
        FILES
        scan_srv.srv
)
generate_messages(DEPENDENCIES std_msgs)


catkin_package(CATKIN_DEPENDS roscpp message_runtime std_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(core_srcs nir_lib/API.cpp nir_lib/usb.cpp nir_lib/Serial.c)
add_library(core ${core_srcs})
target_include_directories(core PUBLIC nir_lib/ nir_lib/DLP/src nir_lib/Common/include nir_lib/hidapi-master/hidapi /usr/include/libusb-1.0)

find_library(DLPspec
        NAMES libdlpspec.a
        PATHS ${CMAKE_CURRENT_SOURCE_DIR}/nir_lib/DLP/src
        REQUIRED
        )
find_library(hidapi
        NAMES libhidapi-hidraw.so
        PATHS ${CMAKE_CURRENT_SOURCE_DIR}/nir_lib/hidapi-master/hidapi
        REQUIRED
        )

message(STATUS "Using DLP Spectrum library : ${DLPspec}")
message(STATUS "Using HIDapi library : ${hidapi}")

file(GLOB DLP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/nir_lib/DLP/src/*.c)
add_library(DLP SHARED ${DLP_SOURCES})
target_include_directories(DLP PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/nir_lib/DLP/src/)

add_library(NIR_Spectrum src/nir_spectrum.cpp)
target_include_directories(NIR_Spectrum PUBLIC nir_lib/ nir_lib/DLP/src nir_lib/Common/include nir_lib/hidapi-master/hidapi)


add_executable(${PROJECT_NAME}_node src/nir_node.cpp)
target_include_directories(${PROJECT_NAME}_node PUBLIC nir_lib/ nir_lib/DLP/src nir_lib/Common/include nir_lib/hidapi-master/hidapi)
target_link_libraries(${PROJECT_NAME}_node NIR_Spectrum DLP core ${hidapi} ${catkin_LIBRARIES})
