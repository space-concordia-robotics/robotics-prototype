cmake_minimum_required(VERSION 3.16.3)
project(CCD)

set(CMAKE_CXX_STANDARD 14)
set(COMPILE_FLAGS -O2 -lm -ldl -Wall -Wpointer-arith -finline-functions -ffast-math -funroll-all-loops)

add_library(CCD_DRIVER LC100_Drv.c)
target_include_directories(CCD_DRIVER PRIVATE include)
target_compile_options(CCD_DRIVER PUBLIC ${COMPILE_FLAGS})

add_executable(CCD sample.c)
target_include_directories(CCD PRIVATE include)
target_link_libraries(CCD PRIVATE CCD_DRIVER visa.so)
