file(MAKE_DIRECTORY build)
set(CMAKE_BINARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build")

cmake_minimum_required(VERSION 3.5.0)
project(MobilePlatform)

#create the library and elf directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

file(GLOB ${PROJECT_NAME}_user_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp) 

#if there are other sources files in this directory, compile them into a library
if(NOT ${${PROJECT_NAME}_user_sources} STREQUAL "")
add_library(${PROJECT_NAME}_user ${${PROJECT_NAME}_user_sources})

target_include_directories(${PROJECT_NAME}_user PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(${PROJECT_NAME}_user PUBLIC ${ARM_GXX_FLAGS})	
endif()

configure_file(${PROJECT_NAME}.ino ${PROJECT_NAME}.ino.cpp)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.ino.cpp)
target_compile_options(${PROJECT_NAME} PUBLIC ${ARM_GXX_FLAGS})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_include_directories(${PROJECT_NAME} PRIVATE ~/Arduino/libraries/LSM303;~/Arduino/libraries/ArduinoBlue/src;~/Arduino/libraries/SparkFun_I2C_GPS_Reading_and_Control/src)

set_target_properties(
${PROJECT_NAME} PROPERTIES 
LINKER_LANGUAGE C 
LINK_FLAGS ${ARM_GCC_LINKER_FLAGS} 
OUTPUT_NAME ${PROJECT_NAME}.elf
)
 
target_link_libraries(
${PROJECT_NAME} 
Servo;Wire;LSM303;ArduinoBlue;SparkFunGPS;SoftwareSerial
$<$<BOOL:${${PROJECT_NAME}_user_sources}>:${PROJECT_NAME}_user> 
teensycore
-lm
) 
