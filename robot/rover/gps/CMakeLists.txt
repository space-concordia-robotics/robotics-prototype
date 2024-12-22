cmake_minimum_required(VERSION 3.5)
project(gps)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(HEADER_FILES
        src/serdes/UbxSerializer.hpp
        src/serdes/UbxDeserializer.hpp
        src/connection/UbxAbstractConnection.hpp
        src/connection/UbxI2CConnection.hpp
        src/connection/LinuxI2CTransactionBuilder.cpp
        src/connection/LinuxI2CTransactionBuilder.hpp
        src/packet/UbxPacket.hpp
        src/packet/UbxPacketNavSat.hpp
        src/packet/UbxPacketNavPosLLH.hpp
)

set(SOURCE_FILES
        src/serdes/UbxSerializer.cpp
        src/serdes/UbxDeserializer.cpp
        src/connection/UbxAbstractConnection.cpp
        src/connection/UbxI2CConnection.cpp
        src/connection/LinuxI2CTransactionBuilder.cpp
        src/connection/LinuxI2CTransactionBuilder.cpp
        src/connection/LinuxI2CTransactionBuilder.hpp
        src/packet/UbxPacket.cpp
        src/packet/UbxPacketNavSat.cpp
        src/packet/UbxPacketNavPosLLH.cpp
)

add_library(gps_node STATIC ${SOURCE_FILES} ${HEADER_FILES})
include_directories(src)
target_include_directories(gps_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

install(TARGETS gps_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)


add_subdirectory(Google_tests)
