#ifndef ROVER_SAM_M8Q_GPS_H
#define ROVER_SAM_M8Q_GPS_H

#define _hwDebug
#include <cstdint>
#include <string>
#include <cstdio>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <iostream>
#include <chrono>
#include <time.h>

#include <sys/time.h>

typedef enum
{
 
    UBLOX_STATUS_SUCCESS,
    UBLOX_STATUS_FD_ERROR,
    UBLOX_STATUS_CLASS_ID_MISMATCH,
    UBLOX_STATUS_PREAMBLE_MISMATCH,
    UBLOX_STATUS_NONE,
    UBLOX_STATUS_ACK_FAILURE,
    UBLOX_STATUS_ACK_RECEIVED,
    UBLOX_STATUS_CRC_FAIL,
    UBLOX_STATUS_DATA_RECEIVED, // This indicates that a 'get' (poll) was successful
    UBLOX_STATUS_I2C_COMM_FAILURE,
} ubx_status_t;

const uint8_t UBX_CLASS_NAV = 0x01;  // Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
const uint8_t UBX_CLASS_MON = 0x0A;  // Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
const uint8_t UBX_CLASS_ACK = 0x05;  // Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status

const uint8_t UBX_MON_VER = 0x04;   // Receiver/Software Version. Used for obtaining Protocol Version.

const uint8_t UBX_NAV_SAT = 0x35;       // Satellite Information
const uint8_t UBX_NAV_STATUS = 0x03;

const uint8_t UBX_ACK = 0x01;
const uint8_t UBX_NACK = 0x00;

const uint8_t COM_PORT_I2C   = 0;
const uint8_t COM_PORT_UART1 = 1;
const uint8_t COM_PORT_UART2 = 2;
const uint8_t COM_PORT_USB   = 3;
const uint8_t COM_PORT_SPI   = 4;

const uint8_t UBX_CLASS_CFG = 0x06;
const uint8_t UBX_CFG_PRT= 0x00;
const uint8_t COM_TYPE_UBX = (1 << 0);
#ifndef defaultMaxWait
#define defaultMaxWait 1100
#endif

#define BUFFER_LENGTH 1000
#define MAX_PAYLOAD_SIZE 220

const uint8_t nav_pvt_payload_size = 90;
const uint8_t mon_ver_payload_size = 220;
const uint8_t nav_sat_payload_size = 255;


typedef struct ubx_header_t{
    uint8_t preambleA = 0xB5;
    uint8_t preambleB = 0x62;
    uint8_t _class = 0;
    uint8_t id = 0;
    uint16_t payload_length = 0;
} ubx_header_t;

typedef struct ubx_packet_t{
    ubx_header_t header;
    uint8_t* payload;
    uint8_t checksumA = 0;
    uint8_t checksumB = 0;                                  
} ubx_packet_t;

typedef struct nav_pvt_t{
    union Latitude{int32_t val ; uint8_t bytes[4];} latitude;
    union Longitude{int32_t val ; uint8_t bytes[4];} longitude;
    union HeightEllipsoid{int32_t val ; uint8_t bytes[4];} height_ellipsoid;
    union HeightMSL{int32_t val ; uint8_t bytes[4];} height_msl;
    uint8_t longitude_hpc;
    uint8_t latitude_hpc;
    uint8_t height_ellipsoid_hpc;
    uint8_t height_msl_hpc;
    uint8_t horizontal_accuracy_estimate[4];
    uint8_t vertical_accuracy_estimate[4];
} nav_pvt_t;


typedef struct ubx_err_t{
    ubx_status_t error;
    char message[200];
} ubx_err_t;


class NEO_M9N{
    const uint8_t deviceAddress = 0x42;
    const uint8_t i2cTransactionSize = 32;
    char hwStatusBuffer[100];
    char statusBuffer[100];
    
    inline static int s_fd;

    const uint8_t i2cPollingWait = 100;    // Default to 100ms. Adjusted when user calls setNavigationFrequency() or setHNRNavigationRate() or setMeasurementRate()
    uint32_t lastCheck;

    

};

#endif