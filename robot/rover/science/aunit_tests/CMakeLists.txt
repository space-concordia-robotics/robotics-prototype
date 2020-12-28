file(MAKE_DIRECTORY build)
set(CMAKE_BINARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build")

cmake_minimum_required(VERSION 3.5.0)
project(ScienceTests)

#create the library and elf directories
get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
file(GLOB TEST_SOURCE *.cpp)

add_executable(${PROJECT_NAME} ${TEST_SOURCE})
target_include_directories(${PROJECT_NAME} PRIVATE ${AUNIT_PATH}/src ${PARENT_DIR})
target_compile_options(${PROJECT_NAME} PRIVATE ${ARM_GXX_FLAGS})

set_target_properties(${PROJECT_NAME} PROPERTIES
LINKER_LANGUAGE C 
LINK_FLAGS ${ARM_GCC_LINKER_FLAGS}
)
 
target_link_libraries(
        ${PROJECT_NAME}
        raman_user
AUNIT
teensycore
-lm
)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND cp ${TEST_SOURCE} ${CMAKE_CURRENT_SOURCE_DIR}/build/${PROJECT_NAME}.ino)


