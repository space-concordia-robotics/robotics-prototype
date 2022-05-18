cmake_minimum_required(VERSION 3.7)

project(ik_solver)

# include and source files
include_directories(include)

set(SOURCE_FILES src/ik-solver.c)

add_executable(ik-solver-test-binary ${SOURCE_FILES})
