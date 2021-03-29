
set(CMAKE_BINARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)

cmake_minimum_required(VERSION 3.10.2)
project(PDS2)

if(DEFINED ENV{ARDUINO_PATH})
    message(STATUS "Path to Arduino : $ENV{ARDUINO_PATH}")
else()
    message(FATAL_ERROR "Arduino path not configured correctly... check wiki for more info: https://github.com/space-concordia-robotics/robotics-prototype/wiki/Setting-up-Teensyduino")
endif()


set(ARDUINO_PATH $ENV{ARDUINO_PATH})
set(ARDUINO_EXTERNAL_LIB $ENV{HOME}/Arduino/libraries)
set(ARDUINO_LIBS ${ARDUINO_PATH}/libraries)

set(ETL_PATH ${ARDUINO_EXTERNAL_LIB}/EmbeddedTemplateLibrary)
set(ETL_URL https://github.com/ETLCPP/etl)
set(ETL_VERSION 19.4.3)

set(AVR_TOOLS ${ARDUINO_PATH}/hardware/tools)
set(ARDUINO_CORE ${ARDUINO_PATH}/hardware/arduino/avr/cores/arduino)

#Set the path to the arduino libraries you're gonna use
set(STEPPER_PATH ${ARDUINO_LIBS}/Stepper)

set(INTERNAL_COMMS_PATH ../../internal_comms)

#Set the compilers for c and c++
set(CMAKE_C_COMPILER ${AVR_TOOLS}/avr/bin/avr-gcc)
set(CMAKE_CXX_COMPILER ${AVR_TOOLS}/avr/bin/avr-g++)

set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

#Find the arduino core files and store them
file(GLOB ARDUINO_CORE_S_FILES ${ARDUINO_CORE}/*.S)
file(GLOB ARDUINO_CORE_C_FILES ${ARDUINO_CORE}/*.c )
file(GLOB ARDUINO_CORE_CXX_FILES ${ARDUINO_CORE}/*.cpp)

set(AVR_GCC_S_FLAGS -c -g -x assembler-with-cpp -flto -MMD -mmcu=atmega328p)
set(AVR_GCC_C_FLAGS -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega328p)
set(AVR_GXX_FLAGS -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p)
set(AVR_GCC_LINKER_FLAGS "-w -Os -g -flto -fuse-linker-plugin -Wl,--gc-sections -mmcu=atmega328p")

add_definitions(
        -DF_CPU=16000000L
        -DARDUINO=10812
        -DARDUINO_AVR_UNO
        -DARDUINO_ARCH_AVR
)
#The .ino file will have access to these base directories
include_directories(
        ${ARDUINO_PATH}/hardware/arduino/avr/variants/standard
        ${AVR_TOOLS}/avr/avr/include/   
        ${ARDUINO_PATH}/hardware/arduino/avr/cores/arduino
)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

#First arg : List of .cpp files that this library needs
#Second arg : Name of the created library
#Third arg : Include directories that the .cpp file needs, can be empty if there are none
function(createCppLibrary sources libraryName includes)
    add_library(${libraryName} ${sources})

    if(NOT ${includes} STREQUAL "")
        target_include_directories(${libraryName} PUBLIC ${includes}) # PUBLIC so that the project also has access to these includes
    endif()

    target_compile_options(${libraryName} PUBLIC ${AVR_GXX_FLAGS})
endfunction()

#given a name, will check if the library exists in the proper directory, otherwise run install_arduino_libraries.sh which will install it
function(checkLibraries lib_name lib_version lib_url)

if(NOT EXISTS ${ARDUINO_EXTERNAL_LIB}/${lib_name})
execute_process(
COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../../install_arduino_libraries.sh ${lib_name} ${lib_version} ${lib_url}
)
endif()

endfunction()

#S FILES - compiled with gcc
add_library(arduinocore_s  OBJECT ${ARDUINO_CORE_S_FILES})
target_compile_options(arduinocore_s PUBLIC ${AVR_GCC_S_FLAGS})
set_target_properties(arduinocore_s PROPERTIES LINKER_LANGUAGE C)

#C FILES - compiled with gcc
add_library(arduinocore_c OBJECT ${ARDUINO_CORE_C_FILES})
target_compile_options(arduinocore_c PUBLIC ${AVR_GCC_C_FLAGS})

#CPP FILES - compiled with g++
add_library(arduinocore $<TARGET_OBJECTS:arduinocore_s> $<TARGET_OBJECTS:arduinocore_c> ${ARDUINO_CORE_CXX_FILES})
target_compile_options(arduinocore PUBLIC ${AVR_GXX_FLAGS})

checkLibraries(EmbeddedTemplateLibrary ${ETL_VERSION} ${ETL_URL})
#Create the arduino libraries here
createCppLibrary(${STEPPER_PATH}/src/Stepper.cpp Stepper "")
#Create the internal_comms library 
add_library(InternalComms "${INTERNAL_COMMS_PATH}/src/CommandCenter.cpp")
target_include_directories(InternalComms PUBLIC ${INTERNAL_COMMS_PATH}/include PUBLIC ${ETL_PATH}/include)
target_compile_definitions(InternalComms PRIVATE ETL_NO_STL)
target_compile_options(InternalComms PUBLIC ${AVR_GXX_FLAGS})

#createCppLibrary(${CMAKE_CURRENT_SOURCE_DIR}/PDSCommandCenter.cpp PDS_sources ${INTERNAL_COMMS_PATH}/include)
add_library(PDS_sources "PDSCommandCenter.cpp")
target_include_directories(PDS_sources PUBLIC ${INTERNAL_COMMS_PATH}/include PUBLIC ${ETL_PATH}/include)
target_compile_definitions(PDS_sources PUBLIC ETL_NO_STL)
target_compile_options(PDS_sources PUBLIC ${AVR_GXX_FLAGS})

set(PROJECT_LIBRARIES InternalComms)


#Rename the .ino file and add it as an executable with an _exe name extension
configure_file(${PROJECT_NAME}.ino ${PROJECT_NAME}.ino.cpp)
add_executable(${PROJECT_NAME}_EXE ${PROJECT_NAME}.ino.cpp )

#Just in case you make a folder in the current directory, the .ino file will be able to include from it
target_include_directories(${PROJECT_NAME}_EXE PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

#In case you need arduino libraries, give the .ino file the include directory of the header file for that library
target_include_directories(${PROJECT_NAME}_EXE PRIVATE ${STEPPER_PATH}/src)

target_compile_options(${PROJECT_NAME}_EXE PUBLIC ${AVR_GXX_FLAGS})

set_target_properties(
        ${PROJECT_NAME}_EXE PROPERTIES
        #LINKER_LANGUAGE C todo : figure out why using g++ works instead of gcc for linking (arduino ide uses gcc)
        LINK_FLAGS ${AVR_GCC_LINKER_FLAGS}
        OUTPUT_NAME ${PROJECT_NAME}.elf
)

# Need to link arduino core libs to internal comms as well because it needs them
target_link_libraries( InternalComms arduinocore)
#Note that if you need extra arduino libraries, it is a semi colon seperated list using the library name you provided in createCppLibrary
target_link_libraries(
        ${PROJECT_NAME}_EXE
        Stepper
        arduinocore
        InternalComms
        PDS_sources
        -lm
)
