cmake_minimum_required(VERSION 3.7)

project(ik_solver)

include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/8d51dc50eb7e7698427fed81b85edad0e032112e.zip
)

FetchContent_MakeAvailable(googletest)

include(GoogleTest)

include_directories(include)
set(SOURCE_FILES src/ik-solver.c unitTests/ik-solver-test.cpp)

add_executable(ik-solver.test ${SOURCE_FILES})

target_link_libraries(ik-solver.test m gtest_main)

gtest_discover_tests(ik-solver.test)
