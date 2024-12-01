cmake_minimum_required(VERSION 3.5)
project(gps)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(HEADER_FILES
        src/packet/UbxPacket.hpp
        src/serdes/UbxSerializer.hpp
        src/serdes/UbxDeserializer.hpp
        src/connection/UbxAbstractConnection.hpp
        src/connection/UbxI2CConnection.hpp
        src/connection/LinuxI2CTransactionBuilder.cpp
        src/connection/LinuxI2CTransactionBuilder.hpp
        src/packet/SatelliteInfo.hpp
)

set(SOURCE_FILES
        src/packet/UbxPacket.cpp
        src/serdes/UbxSerializer.cpp
        src/serdes/UbxDeserializer.cpp
        src/connection/UbxAbstractConnection.cpp
        src/connection/UbxI2CConnection.cpp
        src/connection/LinuxI2CTransactionBuilder.cpp
        src/packet/UbxPacketNavSat.cpp
        src/connection/LinuxI2CTransactionBuilder.cpp
        src/connection/LinuxI2CTransactionBuilder.hpp
        src/packet/UbxPacketNavSat.hpp
)

add_library(gps_node STATIC ${SOURCE_FILES} ${HEADER_FILES})
#add_executable(gps_node src/gpsPublisherNode.cpp)


include_directories(src)


add_subdirectory(Google_tests)
