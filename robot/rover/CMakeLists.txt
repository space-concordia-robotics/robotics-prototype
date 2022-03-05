cmake_minimum_required(VERSION 3.10.2)
include(ExternalProject)

set(CMAKE_SYSTEM_PROCESSOR arm)
set(MCU mk66fx1m0)

if(DEFINED ENV{ARDUINO_PATH})
message(STATUS "Path to Arduino : $ENV{ARDUINO_PATH}")
else()
        message(FATAL_ERROR "Arduino path not configured correctly... check wiki for more info: https://github.com/space-concordia-robotics/robotics-prototype/wiki/Setting-up-Teensyduino")
endif()

set(ARDUINO_PATH $ENV{ARDUINO_PATH})

#paths to important utilities
set(ARDUINO_CORE ${ARDUINO_PATH}/hardware/arduino/avr/cores/arduino)
set(TEENSY_CORE ${ARDUINO_PATH}/hardware/teensy/avr/cores)
set(TEENSY_LIB ${ARDUINO_PATH}/hardware/teensy/avr/libraries)
set(ARDUINO_EXTERNAL_LIB $ENV{HOME}/Arduino/libraries)
set(ARM_TOOLS ${ARDUINO_PATH}/hardware/tools)
set(ARM_LIB ${ARM_TOOLS}/arm/arm-none-eabi/lib)
set(ARM_GCC_OBJCOPY ${ARM_TOOLS}/arm/bin/arm-none-eabi-objcopy)
set(TEENSY_LOADER_CLI ${ARM_TOOLS}/teensy_loader_cli)


#given a name, will check if the library exists in the proper directory, otherwise run install_arduino_libraries.sh which will install it
function(checkLibraries lib_name lib_version lib_url)

if(NOT EXISTS ${ARDUINO_EXTERNAL_LIB}/${lib_name})
execute_process(
COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/install_arduino_libraries.sh ${lib_name} ${lib_version} ${lib_url}
)
endif()

endfunction()


#Set the compilers for c and c++
set(CMAKE_C_COMPILER ${ARM_TOOLS}/arm/bin/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER ${ARM_TOOLS}/arm/bin/arm-none-eabi-g++)

#this is to avoid a very well known infite loop bug when cross compiling in cmake
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)

#Pre-Installed Teensy libaries paths
set(SERVO_PATH ${TEENSY_LIB}/Servo)
set(SOFTWARESERIAL_PATH ${TEENSY_LIB}/SoftwareSerial)
set(ENCODER_PATH ${TEENSY_LIB}/Encoder)
set(WIRE_PATH ${TEENSY_LIB}/Wire)

#External Libaries paths
set(ARDUINO_BLUE_PATH ${ARDUINO_EXTERNAL_LIB}/ArduinoBlue/src)
set(ARDUINO_BLUE_URL https://github.com/purwar2016/ArduinoBlue-library)
set(ARDUINO_BLUE_VERSION 3.0.1) 

set(SPARKFUN_GPS_PATH ${ARDUINO_EXTERNAL_LIB}/SparkFun_I2C_GPS_Reading_and_Control/src)
set(SPARKFUN_GPS_URL https://github.com/sparkfun/SparkFun_I2C_GPS_Arduino_Library)
set(SPARKFUN_GPS_VERSION v1.0.2)

set(ADAFRUIT_PATH ${ARDUINO_EXTERNAL_LIB}/Adafruit_Motor_Shield_V2_Library)
set(ADAFRUIT_URL https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library)
set(ADAFRUIT_VERSION 1.0.9)

set(LSM303_PATH ${ARDUINO_EXTERNAL_LIB}/LSM303)
set(LSM303_URL https://github.com/pololu/lsm303-arduino)
set(LSM303_VERSION 3.0.1)

set(AUNIT_PATH ${ARDUINO_EXTERNAL_LIB}/AUnit)
set(AUNIT_URL https://github.com/bxparks/AUnit)
set(AUNIT_VERSION v1.5)

set(UNIXHOSTDUINO_PATH ${ARDUINO_EXTERNAL_LIB}/UnixHostDuino)
set(UNIXHOSTDUINO_URL https://github.com/kbebs/UnixHostDuino)
set(UNIXHOSTDUINO_VERSION v0.1)

set(TINYGPS_PATH ${ARDUINO_EXTERNAL_LIB}/TinyGPS/src)
set(TINYGPS_URL https://github.com/mikalhart/TinyGPSPlus)
set(TINYGPS_VERSION v1.0.2)

set(ETL_PATH ${ARDUINO_EXTERNAL_LIB}/EmbeddedTemplateLibrary)
set(ETL_URL https://github.com/ETLCPP/etl)
set(ETL_VERSION 19.4.3)

project(rover)
checkLibraries(ArduinoBlue ${ARDUINO_BLUE_VERSION}  ${ARDUINO_BLUE_URL})
checkLibraries(Adafruit_Motor_Shield_V2_Library ${ADAFRUIT_VERSION} ${ADAFRUIT_URL})
checkLibraries(LSM303 ${LSM303_VERSION} ${LSM303_URL})
checkLibraries(SparkFun_I2C_GPS_Reading_and_Control ${SPARKFUN_GPS_VERSION} ${SPARKFUN_GPS_URL})
checkLibraries(AUnit ${AUNIT_VERSION} ${AUNIT_URL})
checkLibraries(UnixHostDuino ${UNIXHOSTDUINO_VERSION} ${UNIXHOSTDUINO_URL})
checkLibraries(TinyGPS ${TINYGPS_VERSION} ${TINYGPS_URL})
checkLibraries(EmbeddedTemplateLibrary ${ETL_VERSION} ${ETL_URL})


#find all teensy sources files that will need to be compiled into a single 'core' library
file(GLOB TEENSY_CORE_S_FILES ${TEENSY_CORE}/teensy3/*.S)
file(GLOB TEENSY_CORE_C_FILES ${TEENSY_CORE}/teensy3/*.c )
file(GLOB TEENSY_CORE_CXX_FILES ${TEENSY_CORE}/teensy3/*.cpp)


#function to find all subdirectories for a given directory (non-recursive), useful for finding paths to the teensy libraries
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

#find paths for all pre-installed teensy libraries, so that all projects have access to these paths
getSubdirectories(TEENSY_LIBRARIES ${TEENSY_LIB})

#every project should have access to all of these paths
include_directories(
 	${TEENSY_CORE}
	${TEENSY_CORE}/teensy3
	${ARM_TOOLS}/arm/arm-none-eabi/include/
	${TEENSY_LIBRARIES}
    ${AUNIT_PATH}/src
)
# TODO : use target include directories instead of this (especially the last line)

#every compiler should use these common defines, copy pasted from the Teensyduino verbose output
add_definitions(
-D__MK66FX1M0__
-DTEENSYDUINO=151
-DARDUINO=10812
-DARDUINO_TEENSY36
-DF_CPU=180000000
-DUSB_SERIAL
-DLAYOUT_US_ENGLISH
)

#compiler flags for all compilers, copy pasted from the Teensyduino verbose output
set(ARM_GCC_S_FLAGS -O2 -g -ffunction-sections -nostdlib -MMD -nostdlib -MMD -x assembler-with-cpp -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant)
set(ARM_GCC_C_FLAGS -O2 -g -ffunction-sections -nostdlib -MMD -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant)
set(ARM_GXX_FLAGS -O2 -g -ffunction-sections -nostdlib -MMD -mthumb -fno-exceptions -fpermissive -felide-constructors -std=gnu++14 -Wno-error=narrowing -fno-rtti -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant)

#linker flags, note even c++ projects link using gcc, since we are using the -lstdc++ flag -> from Teensyduino
set(ARM_GCC_LINKER_FLAGS "-O2 -Wl,--gc-sections,--relax,--defsym=__rtc_localtime=1584728951 -T${TEENSY_CORE}/teensy3/${MCU}.ld -lstdc++ -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant")

#S FILES - compiled with gcc
add_library(teensycore_s  OBJECT ${TEENSY_CORE_S_FILES})
target_compile_options(teensycore_s PUBLIC ${ARM_GCC_S_FLAGS})
set_target_properties(teensycore_s PROPERTIES LINKER_LANGUAGE C)

#C FILES - compiled with gcc
add_library(teensycore_c OBJECT ${TEENSY_CORE_C_FILES})
target_compile_options(teensycore_c PUBLIC ${ARM_GCC_C_FLAGS})

#CPP FILES - compiled with g++
add_library(teensycore $<TARGET_OBJECTS:teensycore_s> $<TARGET_OBJECTS:teensycore_c> ${TEENSY_CORE_CXX_FILES})
target_compile_options(teensycore PUBLIC ${ARM_GXX_FLAGS})

#find all the .ino files recursively relative to where the current project (rover in this case)
file(GLOB_RECURSE INOFILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/*.ino)

#This function will create a library and store it with the given name
function(createCppLibrary sources libraryName includes)
        add_library(${libraryName} ${sources})

        if(NOT ${includes} STREQUAL "")
        target_include_directories(${libraryName} PRIVATE ${includes})
        endif()

        target_compile_options(${libraryName} PUBLIC ${ARM_GXX_FLAGS})
endfunction() 

#create default teensy libraries
#the last argument in these functions is incase a library needs additional include paths
createCppLibrary(${TEENSY_LIB}/Servo/Servo.cpp Servo "")
createCppLibrary(${TEENSY_LIB}/SoftwareSerial/SoftwareSerial.cpp SoftwareSerial "")
createCppLibrary(${ENCODER_PATH}/Encoder.cpp Encoder "")
createCppLibrary("${WIRE_PATH}/Wire.cpp;${WIRE_PATH}/WireIMXRT.cpp;${WIRE_PATH}/WireKinetis.cpp" Wire "")

#The last argument indicates if there are any additional include paths that are needed during the creation of a library
#create the external teensy libraries
createCppLibrary(${ARDUINO_BLUE_PATH}/ArduinoBlue.cpp ArduinoBlue "")
createCppLibrary("${ADAFRUIT_PATH}/Adafruit_MotorShield.cpp;${ADAFRUIT_PATH}/utility/Adafruit_MS_PWMServoDriver.cpp" Adafruit ${ADAFRUIT_PATH}/utility)
createCppLibrary(${SPARKFUN_GPS_PATH}/SparkFun_I2C_GPS_Arduino_Library.cpp SparkFunGPS "")
createCppLibrary(${LSM303_PATH}/LSM303.cpp LSM303 ${LSM303_PATH})
createCppLibrary(${TINYGPS_PATH}/TinyGPS++.cpp TinyGPS "")

# For a library that doesn't have a cpp file, but only header files
include_directories(${ETL_PATH}/include)

#libraries should be created before the executables
add_subdirectory(internal_comms)

#create the testing library
file(GLOB_RECURSE AUNIT_SOURCES ${AUNIT_PATH}/*.cpp)
createCppLibrary("${AUNIT_SOURCES}" AUNIT ${AUNIT_PATH}/src)

#go through each .ino project file that was found
foreach(path ${INOFILES})
 get_filename_component(project_dir ${path} DIRECTORY)
 get_filename_component(name ${path} NAME_WE)

 #creates a command which runs a few bash commands to delete the current subproject. 
 #These are useless on their own and need to be attached to a target to work 
 add_custom_command(
 OUTPUT ${project_dir}_cleanup
 COMMAND rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/${project_dir}/build
 	 rm ${CMAKE_CURRENT_SOURCE_DIR}/${project_dir}/CMakeLists.txt
 )

 set(Project_Name ${name})

 if(${name} STREQUAL "Arm")
  set(PROJECT_LIBRARIES Servo internal_comms)
 elseif(${name} STREQUAL "raman")
  set(PROJECT_LIBRARIES internal_comms)
 elseif(${name} STREQUAL "comms_demo")
  set(PROJECT_LIBRARIES internal_comms)
 # this particular project is important and should be supported, issue #409 opened for this
 # for now it will be skipped
 elseif(${name} STREQUAL "PDS")
 continue()
 elseif(${name} STREQUAL "PDS2")
 continue()
 
 elseif(${name} STREQUAL "MobilePlatform")
 
 set(PROJECT_LIBRARIES internal_comms Servo Wire LSM303 ArduinoBlue SparkFunGPS SoftwareSerial TinyGPS)
 set(EXTERNAL_LIBARRIES_INCLUDE_DIRS ${LSM303_PATH} ${ARDUINO_BLUE_PATH} ${SPARKFUN_GPS_PATH} ${TINYGPS_PATH} ${ETL_PATH})

 elseif(${name} STREQUAL "PidController")
 set(PROJECT_LIBRARIES Adafruit Encoder Wire Servo)
 set(EXTERNAL_LIBRARIES_INCLUDE_DIRS ${ADAFRUIT_PATH})
 
 endif()
 
  #write the template with the child project name in child project directory and execute the CMakeLists.txt file in there when created
  configure_file(CMakeTemplate.txt.in ${CMAKE_CURRENT_SOURCE_DIR}/${project_dir}/CMakeLists.txt @ONLY)
  add_subdirectory(${project_dir})

 endforeach()

 #this builds a target, which can be called be built from the command line to delete ALL generated files, and directories. 
 #the command is cmake --build . --target clean

 #you'd think you could give the function a list of dependecies rather than this manual listing of dependecies which we see here, but I havent found a way of doing this...
 add_custom_target(cleanup
 COMMENT "cleaning up..."
 COMMAND rm -rf ${CMAKE_CURRENT_SOURCE_DIR}/build/* ${CMAKE_CURRENT_SOURCE_DIR}/cmake-build-debug
 DEPENDS Arm_cleanup
 DEPENDS MobilePlatform_cleanup
 DEPENDS PidController_cleanup
 DEPENDS science_cleanup
 )

#Add the NIR an an external since it does not use the ARM compiler
ExternalProject_Add(NIR
        SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/science/NIR
        BUILD_ALWAYS 1
        INSTALL_COMMAND ""
        )
