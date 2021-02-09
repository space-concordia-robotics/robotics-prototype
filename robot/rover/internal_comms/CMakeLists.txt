file(MAKE_DIRECTORY build)
set(CMAKE_BINARY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/build")

cmake_minimum_required(VERSION 3.5.0)
project("internal_comms")

#create the library and elf directories
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

file(GLOB_RECURSE ${PROJECT_NAME}_library_sources ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp) 

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_library_sources})
target_compile_definitions(${PROJECT_NAME} PUBLIC ETL_NO_STL)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(${PROJECT_NAME} PUBLIC ${ARM_GXX_FLAGS})	

