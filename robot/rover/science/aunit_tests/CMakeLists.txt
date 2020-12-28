file(MAKE_DIRECTORY build)
set(CMAKE_BINARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build")

cmake_minimum_required(VERSION 3.5.0)
project(CommandTests)

#create the library and elf directories
get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)

add_executable(CommandTester CommandTests.cpp)
target_include_directories(CommandTester PRIVATE ${AUNIT_PATH}/src ${PARENT_DIR})
target_compile_options(CommandTester PRIVATE ${ARM_GXX_FLAGS})

set_target_properties(CommandTester PROPERTIES
LINKER_LANGUAGE C 
LINK_FLAGS ${ARM_GCC_LINKER_FLAGS}
)
 
target_link_libraries(
        CommandTester
        raman_user
AUNIT
$<$<BOOL:${USER_SOURCES}>:${PROJECT_NAME}_user>
teensycore
-lm
) 


