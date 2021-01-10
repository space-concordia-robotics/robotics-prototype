cmake_minimum_required(VERSION 3.10.2)
project(ScienceTests)

file(MAKE_DIRECTORY build/obj)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin)

get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
file(GLOB TEST_SOURCE *.cpp)
add_executable(${PROJECT_NAME} ${TEST_SOURCE})
target_compile_options(${PROJECT_NAME} PUBLIC ${ARM_GXX_FLAGS})

set_target_properties(${PROJECT_NAME} PROPERTIES
LINKER_LANGUAGE C 
LINK_FLAGS ${ARM_GCC_LINKER_FLAGS}
)
 
target_link_libraries( ${PROJECT_NAME}
${project_sources}
AUNIT
teensycore
-lm
)

