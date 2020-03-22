cmake_minimum_required(VERSION 3.5)
set(CMAKE_SYSTEM_PROCESSOR arm)

project(science)
set(ARDUINO_PATH /home/niko/Software/arduino-1.8.12)

#SET PATHS
set(ARDUINO_CORE ${ARDUINO_PATH}/hardware/arduino/avr/cores/arduino)
set(TEENSY_CORE ${ARDUINO_PATH}/hardware/teensy/avr/cores)
set(TEENSY_LIB ${ARDUINO_PATH}/hardware/teensy/avr/libraries)
set(ARM_TOOLS ${ARDUINO_PATH}/hardware/tools/arm/bin)

#SET UTILITIES
set(CMAKE_C_COMPILER ${ARM_TOOLS}/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER ${ARM_TOOLS}/arm-none-eabi-g++)
set(CMAKE_OBJCOPY ${ARM_TOOLS}/arm-none-eabi-objcopy)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin)

#FIND ALL SOURCE FILES
file(GLOB TEENSY_CORE_S_FILES ${TEENSY_CORE}/teensy3/*.S)
file(GLOB TEENSY_CORE_C_FILES ${TEENSY_CORE}/teensy3/*.c )
file(GLOB TEENSY_CORE_CXX_FILES ${TEENSY_CORE}/teensy3/*.cpp)

file(GLOB TEENSY_USED_LIBRARIES ${TEENSY_LIB}/Servo/Servo.cpp)

include_directories(
 	${TEENSY_CORE}
	${TEENSY_CORE}/teensy3
	${TEENSY_LIB}/Servo
)

#ALL COMPILERS USE COMMON DEFINES
add_definitions(
-D__MK66FX1M0__
-DTEENSYDUINO=151
-DARDUINO=10812
-DARDUINO_TEENSY36
-DF_CPU=180000000
-DUSB_SERIAL
-DLAYOUT_US_ENGLISH
)
#COMPILER FLAGS
set(ARM_GCC_S_FLAGS -O2 -g -ffunction-sections -nostdlib -MMD -nostdlib -MMD -x assembler-with-cpp -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant)
set(ARM_GCC_C_FLAGS -O2 -g -ffunction-sections -nostdlib -MMD -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant)
set(ARM_GXX_FLAGS -O2 -g -ffunction-sections -nostdlib -MMD -mthumb -fno-exceptions -fpermissive -felide-constructors -std=gnu++14 -Wno-error=narrowing -fno-rtti -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant)

#LINKER FLAGS
set(ARM_GCC_LINKER_FLAGS "-O2 -Wl,--gc-sections,--relax,--defsym=__rtc_localtime=1584728951 -T${TEENSY_CORE}/teensy3/mk66fx1m0.ld -lstdc++ -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant")

#not used for now(only for hexing)
set(ARM_GCC_OBJCOPY_FLAGS "-O ihex -R .eeprom ${CMAKE_BINARY_DIR}/bin/${PROJECT_NAME}.ino.elf hexfile.hex")

#S FILES
add_library(teensycore_s  OBJECT ${TEENSY_CORE_S_FILES})
target_compile_options(teensycore_s PRIVATE ${ARM_GCC_S_FLAGS})
set_target_properties(teensycore_s PROPERTIES LINKER_LANGUAGE C)

#C FILES
add_library(teensycore_c OBJECT ${TEENSY_CORE_C_FILES})
target_compile_options(teensycore_c PUBLIC ${ARM_GCC_C_FLAGS})

#CPP FILES
add_library(teensycore_cxx OBJECT ${TEENSY_CORE_CXX_FILES})
target_compile_options(teensycore_cxx PRIVATE ${ARM_GXX_FLAGS})

#USER LIBRARIES
add_library(mainlibrary $<TARGET_OBJECTS:teensycore_s> $<TARGET_OBJECTS:teensycore_c> $<TARGET_OBJECTS:teensycore_cxx> ${TEENSY_USED_LIBRARIES})
target_compile_options(mainlibrary PRIVATE ${ARM_GXX_FLAGS})

configure_file(${PROJECT_NAME}.ino ${PROJECT_NAME}.ino.cpp)

#CREATE EXE, LINK TO IT, AND OUTPUT AN ELF 
add_executable(${PROJECT_NAME} science.ino.cpp)
target_compile_options(${PROJECT_NAME} PRIVATE ${ARM_GXX_FLAGS})
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE C LINK_FLAGS ${ARM_GCC_LINKER_FLAGS} OUTPUT_NAME ${PROJECT_NAME}.elf)

target_link_libraries(${PROJECT_NAME} mainlibrary) 



