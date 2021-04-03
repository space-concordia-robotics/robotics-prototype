cmake_minimum_required(VERSION 3.10.2)
#The project name is the only parameter that needs to be changed for any other project needing to do c++ unit testing
project(ScienceTests)

#The target executable will reside in build/bin
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin)

get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
file(GLOB TEST_SOURCE *.cpp)
add_executable(${PROJECT_NAME} ${TEST_SOURCE})
target_compile_options(${PROJECT_NAME} PUBLIC ${ARM_GXX_FLAGS})
target_include_directories(${PROJECT_NAME} PRIVATE ${PARENT_DIR}/include)
set_target_properties(${PROJECT_NAME} PROPERTIES
LINKER_LANGUAGE C 
LINK_FLAGS ${ARM_GCC_LINKER_FLAGS}
)

#Note the second line, CMakeTemplate.txt.in will pass in the name of the library it generated so our testing file can link to
target_link_libraries( ${PROJECT_NAME}
${project_sources}
AUNIT
teensycore
-lm
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMENT "Running test for ${PROJECT_NAME}"
        COMMAND make
        COMMAND ./${PROJECT_NAME}.out
        COMMAND make clean
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        VERBATIM
        )

