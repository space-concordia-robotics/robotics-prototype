 cmake_minimum_required(VERSION 3.10)
 project(NIR)

 set(core_srcs src/API.cpp src/usb.cpp src/Serial.c)
 add_library(core ${core_srcs})
 target_include_directories(core PUBLIC src/ src/DLP/src src/Common/include src/hidapi-master/hidapi /usr/include/libusb-1.0)


 find_library(DLPspec
         NAMES libdlpspec.a
         PATHS ${CMAKE_CURRENT_SOURCE_DIR}/src/DLP/src
         REQUIRED
         )
 find_library(hidapi
         NAMES libhidapi-hidraw.so
         PATHS ${CMAKE_CURRENT_SOURCE_DIR}/src/hidapi-master/hidapi
         REQUIRED
         )

 message(STATUS "Using DLP Spectrum library : ${DLPspec}")
 message(STATUS "Using HIDapi library : ${hidapi}")

 file(GLOB DLP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/DLP/src/*.c)
 add_library(DLP SHARED ${DLP_SOURCES})
 target_include_directories(DLP PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/DLP/src/)

 add_library(Spectrum Spectrometer.cpp)
 target_include_directories(Spectrum PUBLIC src/ src/DLP/src src/Common/include src/hidapi-master/hidapi)


 add_executable(NIR_EXE main.cpp)
 target_include_directories(NIR_EXE PUBLIC src/ src/DLP/src src/Common/include src/hidapi-master/hidapi)
 target_link_libraries(NIR_EXE Spectrum DLP core ${hidapi} )
