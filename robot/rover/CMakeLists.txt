cmake_minimum_required(VERSION 3.17.0)
set(CMAKE_SYSTEM_PROCESSOR arm)


#set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

set(ARDUINO_PATH /home/niko/Software/arduino-1.8.12)
set(ARDUINO_CORE ${ARDUINO_PATH}/hardware/arduino/avr/cores/arduino)
set(TEENSY_CORE ${ARDUINO_PATH}/hardware/teensy/avr/cores)
set(TEENSY_LIB ${ARDUINO_PATH}/hardware/teensy/avr/libraries)
set(ARDUINO_EXTERNAL_LIB ~/Arduino/libraries)
#set(ROS "/opt/ros/kinetic/include")
#set(BOOST "/usr/include/boost")
set(ARM_TOOLS ${ARDUINO_PATH}/hardware/tools)
set(ARM_LIB ${ARM_TOOLS}/arm/arm-none-eabi/lib)
set(CMAKE_C_COMPILER ${ARM_TOOLS}/arm/bin/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER ${ARM_TOOLS}/arm/bin/arm-none-eabi-g++)
set(ARDUINO_ADD_ON_LIBRARIES "~/Arduino/libraries")
SET(CMAKE_C_COMPILER_WORKS 1)
SET(CMAKE_CXX_COMPILER_WORKS 1)

#Built in libaries
set(SERVO_PATH ${TEENSY_LIB}/Servo)
set(SOFTWARESERIAL_PATH ${TEENSY_LIB}/SoftwareSerial)
set(ENCODER_PATH ${TEENSY_LIB}/Encoder)
set(WIRE_PATH ${TEENSY_LIB}/Wire)

#External Libaries
set(ARDUINOBLUE_PATH ${ARDUINO_EXTERNAL_LIB}/ArduinoBlue/src)
set(SPARKFUN_PATH ${ARDUINO_EXTERNAL_LIB}/SparkFun_I2C_GPS_Reading_and_Control/src)
set(ADAFRUIT_PATH ${ARDUINO_EXTERNAL_LIB}/Adafruit_Motor_Shield_V2_Library)
set(LSM303_PATH ${ARDUINO_EXTERNAL_LIB}/LSM303)

project(rover)


set(ROVER_PATH ${CMAKE_CURRENT_SOURCE_DIR})


#FIND ALL SOURCE FILES
file(GLOB TEENSY_CORE_S_FILES ${TEENSY_CORE}/teensy3/*.S)
file(GLOB TEENSY_CORE_C_FILES ${TEENSY_CORE}/teensy3/*.c )
file(GLOB TEENSY_CORE_CXX_FILES ${TEENSY_CORE}/teensy3/*.cpp)

macro(getSubdirectories result curDir)

file(GLOB subDirs LIST_DIRECTORIES true ${curDir}/*)

set(dirList "")
list(APPEND dirList ${curDir})

foreach(element ${subDirs})

if(IS_DIRECTORY ${element})
list(APPEND dirList ${element}) 
endif()

endforeach()
set(${result} ${dirList})
endmacro()



macro(getIncludeDirectories result rootDir)
file(GLOB subDirs LIST_DIRECTORIES true ${rootDir}/*)

set(includeDirs "")
foreach(element ${subDirs})

if(IS_DIRECTORY ${element} AND NOT ${element} STREQUAL "${rootDir}/build")


file(GLOB inofile ${element}/*.ino)

if("${inofile}" STREQUAL "")
list(APPEND includeDirs ${element})
endif()
 
endif()

endforeach()

set(${result} ${includeDirs}) 
endmacro()

getSubdirectories(TEENSY_LIBRARIES ${TEENSY_LIB})

include_directories(
 	${TEENSY_CORE}
	${TEENSY_CORE}/teensy3
	${ARM_TOOLS}/arm/arm-none-eabi/include/
	${TEENSY_LIBRARIES}
	${ARDUINOBLUE_PATH}
	${SPARKFUN_PATH}

)

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
set(ARM_GXX_FLAGS -Idir${ARM_GCC_INCLUDE_DIR} -O2 -g -ffunction-sections -nostdlib -MMD -mthumb -fno-exceptions -fpermissive -felide-constructors -std=gnu++14 -Wno-error=narrowing -fno-rtti -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant)

#LINKER FLAGS
set(ARM_GCC_LINKER_FLAGS "-O2 -Wl,--gc-sections,--relax,--defsym=__rtc_localtime=1584728951 -T${TEENSY_CORE}/teensy3/mk66fx1m0.ld -lstdc++ -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant")

#not used for now(only for hexing)
#set(ARM_GCC_OBJCOPY_FLAGS "-O ihex -R .eeprom ${CMAKE_BINARY_DIR}/bin/${PROJECT_NAME}.ino.elf hexfile.hex")

#S FILES - THEY SHOULD BE COMPILED USING GCC
add_library(teensycore_s  OBJECT ${TEENSY_CORE_S_FILES})
target_compile_options(teensycore_s PUBLIC ${ARM_GCC_S_FLAGS})
set_target_properties(teensycore_s PROPERTIES LINKER_LANGUAGE C)

#C FILES - THEY SHOULD BE COMPILED USING GCC
add_library(teensycore_c OBJECT ${TEENSY_CORE_C_FILES})
target_compile_options(teensycore_c PUBLIC ${ARM_GCC_C_FLAGS})

#CPP FILES - THEY SHOULD BE COMPILED USING GXX
add_library(teensycore $<TARGET_OBJECTS:teensycore_s> $<TARGET_OBJECTS:teensycore_c> ${TEENSY_CORE_CXX_FILES})
target_compile_options(teensycore PUBLIC ${ARM_GXX_FLAGS})

#FIND ALL THE INO PROJECT FILES 
file(GLOB_RECURSE INOFILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/*.ino)

function(createCppLibrary sources libraryName includes)
add_library(${libraryName} ${sources})

if(NOT ${includes} STREQUAL "")
target_include_directories(${libraryName} PRIVATE ${includes})
endif()

target_compile_options(${libraryName} PUBLIC ${ARM_GXX_FLAGS})
endfunction() 

#BUILT IN LIBARIES
createCppLibrary(${TEENSY_LIB}/Servo/Servo.cpp Servo "")
createCppLibrary(${TEENSY_LIB}/SoftwareSerial/SoftwareSerial.cpp SoftwareSerial "")
createCppLibrary(${ENCODER_PATH}/Encoder.cpp Encoder "")
createCppLibrary("${WIRE_PATH}/Wire.cpp;${WIRE_PATH}/WireIMXRT.cpp;${WIRE_PATH}/WireKinetis.cpp" Wire "")

#EXTERNAL LIBARIES INSTALLED USING THE ARDUINO IDE
createCppLibrary(${ARDUINOBLUE_PATH}/ArduinoBlue.cpp ArduinoBlue "")
createCppLibrary("${ADAFRUIT_PATH}/Adafruit_MotorShield.cpp;${ADAFRUIT_PATH}/utility/Adafruit_MS_PWMServoDriver.cpp" Adafruit ${ADAFRUIT_PATH}/utility)
CreateCppLibrary(${SPARKFUN_PATH}/SparkFun_I2C_GPS_Arduino_Library.cpp SparkFunGPS "")
CreateCppLibrary(${LSM303_PATH}/LSM303.cpp LSM303 ${LSM303_PATH})

foreach(path ${INOFILES})
 get_filename_component(project_dir ${path} DIRECTORY)
 get_filename_component(name ${path} NAME_WE)
 
 

 add_custom_command(
 OUTPUT ${name}_cleanup
 COMMAND rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/${project_dir}/build
 	 rm ${CMAKE_CURRENT_SOURCE_DIR}/${project_dir}/CMakeLists.txt
 )

 set(Project_Name ${name})
 
 if(${name} STREQUAL "Arm")
 set(EXTERNAL_LIBRARIES  Servo)

 elseif(${name} STREQUAL "DcExampleTwoMotors") 
 continue()

 elseif(${name} STREQUAL "RosCommunication")
 continue()

 elseif(${name} STREQUAL "ServoExampleOpenLoop")
 continue()

 elseif(${name} STREQUAL "MobilePlatform")
 
 set(EXTERNAL_LIBRARIES Servo Wire LSM303 ArduinoBlue SparkFunGPS SoftwareSerial)
 set(EXTERNAL_LIBARRIES_INCLUDE_DIRS ${LSM303_PATH})

 
 elseif(${name} STREQUAL "path_send_test")
 continue()

 elseif(${name} STREQUAL "pure_pursuit_follower")
 continue()

 elseif(${name} STREQUAL "PidController")
 set(EXTERNAL_LIBRARIES Adafruit Encoder Wire Servo)
 set(EXTERNAL_LIBARRIES_INCLUDE_DIRS ${ADAFRUIT_PATH})

 elseif(${name} STREQUAL "EncoderFunc")

 elseif(${name} STREQUAL "MultipleConstructors")

 elseif(${name} STREQUAL "science")

 endif()

 configure_file(CMakeTemplate.txt.in ${CMAKE_CURRENT_SOURCE_DIR}/${project_dir}/CMakeLists.txt @ONLY) 
 add_subdirectory(${project_dir}) 

 

endforeach()

 #THIS BUILDS A TARGET, WHICH CAN BE BUILT IN COMMAND LINE TO REMOVE ALL THE GENERATED FILES FROM CMAKE 
  
 add_custom_target(cleanup
 COMMENT "cleaning up..."
 COMMAND rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/build/*
 DEPENDS Arm_cleanup
 
 DEPENDS DcExampleTwoMotors_cleanup
 DEPENDS RosCommunication_cleanup
 DEPENDS ServoExampleOpenLoop_cleanup
 DEPENDS MobilePlatform_cleanup
 DEPENDS path_send_test_cleanup
 DEPENDS pure_pursuit_follower_cleanup
 DEPENDS PidController_cleanup

 DEPENDS EncoderFunc_cleanup
 DEPENDS MultipleConstructors_cleanup
 DEPENDS science_cleanup

 )


