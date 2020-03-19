cmake_minimum_required(VERSION 3.5)
set(CMAKE_SYSTEM_PROCESSOR arm)

project(science)
set(ARDUINO_PATH /home/niko/Software/arduino-1.8.12)

set(TEENSY_CORE ${ARDUINO_PATH}/hardware/teensy/avr/cores)
set(TEENSY_LIB ${ARDUINO_PATH}/hardware/teensy/avr/libraries)
set(ARM_TOOLS ${ARDUINO_PATH}/hardware/tools/arm/bin)

set(CMAKE_C_COMPILER ${ARM_TOOLS}/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER ${ARM_TOOLS}/arm-none-eabi-g++)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/lib)

file(GLOB TEENSY_CORE_FILES ${TEENSY_CORE}/teensy3/*.c ${TEENSY_CORE}/teensy3/*.cpp)
#file(GLOB TEENSY_CORE_S_FILES ${TEENSY_CORE}/teensy3/*.S)


include_directories(
	${TEENSY_CORE}
	${TEENSY_CORE}/teensy3
	${TEENSY_LIB}/Servo
)
set(EABI_GCC_FLAGS "-O2 -g -ffunction-sections -nostdlib -MMD -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant")
set(EABI_GXX_FLAGS "-O2 -g -ffunction-sections -nostdlib -MMD -mthumb -fno-exceptions -fpermissive -felide-constructors -std=gnu++14 -Wno-error=narrowing -fno-rtti -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EABI_GCC_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EABI_GXX_FLAGS}")
add_definitions(
-D__MK66FX1M0__
-DTEENSYDUINO=151
-DARDUINO=10812
-DARDUINO_TEENSY36
-DF_CPU=180000000
-DUSB_SERIAL
-DLAYOUT_US_ENGLISH
)

add_library(teensycore STATIC ${TEENSY_CORE_FILES})

#message(STATUS ${TEENSY_CORE_FILES})


#add_executable(source ${TEENSY_CORE_FILES})
