file(MAKE_DIRECTORY build)
set(CMAKE_BINARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build")
set(USER_SOURCES FALSE)

cmake_minimum_required(VERSION 3.5.0)
project(PDS2)

#create the library and elf directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

file(GLOB_RECURSE ${PROJECT_NAME}_user_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

set(USER_SOURCES FALSE)


aux_source_directory(${PROJECT_NAME}_src ${CMAKE_CURRENT_SOURCE_DIR}/src)
#if there are other sources files in this directory, compile them into a library
#if(NOT "${${PROJECT_NAME}_user_sources}" STREQUAL "")

#set(USER_SOURCES TRUE)
add_library(${PROJECT_NAME}_src_lib ${${PROJECT_NAME}_src})

#target_include_directories(${PROJECT_NAME}_user PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/includes /home/nikolas/Arduino/libraries/LSM303;/home/nikolas/Arduino/libraries/ArduinoBlue/src;/home/nikolas/Arduino/libraries/SparkFun_I2C_GPS_Reading_and_Control/src;/home/nikolas/Arduino/libraries/TinyGPS/src;/home/nikolas/Arduino/libraries/EmbeddedTemplateLibrary)
#target_compile_options(${PROJECT_NAME}_user PUBLIC ${ARM_GXX_FLAGS})
#endif()


configure_file(${PROJECT_NAME}.ino ${PROJECT_NAME}.ino.cpp)

add_executable(${PROJECT_NAME}_EXE ${PROJECT_NAME}.ino.cpp)
target_compile_options(${PROJECT_NAME}_EXE PUBLIC ${ARM_GXX_FLAGS})
target_include_directories(${PROJECT_NAME}_EXE PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/includes)
target_include_directories(${PROJECT_NAME}_EXE PRIVATE )


set_target_properties(
${PROJECT_NAME}_EXE PROPERTIES 
LINKER_LANGUAGE C 
LINK_FLAGS ${ARM_GCC_LINKER_FLAGS} 
OUTPUT_NAME ${PROJECT_NAME}.elf
)
 
target_link_libraries(
${PROJECT_NAME}_EXE
$<$<BOOL:${USER_SOURCES}>:${PROJECT_NAME}_user>
internal_comms;Servo;Wire;LSM303;ArduinoBlue;SparkFunGPS;SoftwareSerial;TinyGPS
teensycore
-lm
) 

add_custom_command(
TARGET ${PROJECT_NAME}_EXE
POST_BUILD
COMMAND ${ARM_GCC_OBJCOPY} -O ihex -R.eeprom ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}.elf ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}.hex 
)
add_custom_command(OUTPUT ${PROJECT_NAME}_upload_command
COMMAND ${TEENSY_LOADER_CLI} --mcu=${MCU} -s -v ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}.hex 
DEPENDS ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${PROJECT_NAME}.hex 
)

add_custom_target(${PROJECT_NAME}
DEPENDS ${PROJECT_NAME}_upload_command
)	
