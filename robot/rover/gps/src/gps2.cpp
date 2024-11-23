//
// Created by nik on 13/01/24.
//

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
#define HEADER_OFFSET 6
#define RESULT_BUFFER_SIZE 1000

// Global Status Returns
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

const uint8_t UBX_CLASS_NAV = 0x01; // Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
const uint8_t UBX_CLASS_MON = 0x0A; // Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
const uint8_t UBX_CLASS_ACK = 0x05; // Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status

const uint8_t UBX_MON_VER = 0x04; // Receiver/Software Version. Used for obtaining Protocol Version.

const uint8_t UBX_NAV_SAT = 0x35; // Satellite Information
const uint8_t UBX_NAV_STATUS = 0x03;

const uint8_t UBX_ACK = 0x01;
const uint8_t UBX_NACK = 0x00;

const uint8_t COM_PORT_I2C = 0;
const uint8_t COM_PORT_UART1 = 1;
const uint8_t COM_PORT_UART2 = 2;
const uint8_t COM_PORT_USB = 3;
const uint8_t COM_PORT_SPI = 4;

const uint8_t UBX_CLASS_CFG = 0x06;
const uint8_t UBX_CFG_PRT = 0x00;
const uint8_t COM_TYPE_UBX = (1 << 0);
#ifndef defaultMaxWait
#define defaultMaxWait 1100
#endif

#define BUFFER_LENGTH 1000
#define MAX_PAYLOAD_SIZE 220

const uint8_t nav_pvt_payload_size = 90;
const uint8_t mon_ver_payload_size = 220;
const uint8_t nav_sat_payload_size = 255;

typedef struct ubx_header_t
{
	uint8_t preambleA = 0xB5;
	uint8_t preambleB = 0x62;
	uint8_t _class = 0;
	uint8_t id = 0;
	uint16_t payload_length = 0;
} ubx_header_t;

typedef struct ubx_packet_t
{
	ubx_header_t header;
	uint8_t *payload;
	uint8_t checksumA = 0;
	uint8_t checksumB = 0;
} ubx_packet_t;

typedef struct nav_pvt_t
{
	union Latitude
	{
		int32_t val;
		uint8_t bytes[4];
	} latitude;
	union Longitude
	{
		int32_t val;
		uint8_t bytes[4];
	} longitude;
	union HeightEllipsoid
	{
		int32_t val;
		uint8_t bytes[4];
	} height_ellipsoid;
	union HeightMSL
	{
		int32_t val;
		uint8_t bytes[4];
	} height_msl;
	uint8_t longitude_hpc;
	uint8_t latitude_hpc;
	uint8_t height_ellipsoid_hpc;
	uint8_t height_msl_hpc;
	uint8_t horizontal_accuracy_estimate[4];
	uint8_t vertical_accuracy_estimate[4];
} nav_pvt_t;

typedef struct
{
	uint8_t rawData[220];
	uint8_t versionLow;
	uint8_t versionHigh;
	bool moduleQueried;
} moduleSWVersion_t;

typedef struct ubx_err_t
{
	ubx_status_t error;
	char message[200];
} ubx_err_t;

class SAM_M8Q_GPS
{

	const uint8_t deviceAddress = 0x42;
	const uint8_t i2cTransactionSize = 32;
	char hwStatusBuffer[100];
	char statusBuffer[100];
	inline static int s_fd;

	moduleSWVersion_t *moduleVersion = nullptr;

	uint8_t i2cPollingWait =
		100; // Default to 100ms. Adjusted when user calls setNavigationFrequency() or setHNRNavigationRate() or setMeasurementRate()
	uint32_t lastCheck;

public:
	SAM_M8Q_GPS()
	{
		struct timeval tv;
		gettimeofday(&tv, NULL);
		lastCheck = (uint32_t)(tv.tv_sec) * 1000 + (uint32_t)(tv.tv_usec) / 1000;
	};

	void setStatusMessage(const ubx_status_t &status)
	{
		switch (status)
		{
		case UBLOX_STATUS_DATA_RECEIVED:
			sprintf(statusBuffer, "Data received.\n");
			break;
		case UBLOX_STATUS_I2C_COMM_FAILURE:
			sprintf(statusBuffer, "I2C Comms error.\n");
			break;
		case UBLOX_STATUS_CLASS_ID_MISMATCH:
			sprintf(statusBuffer, "Error in class and id matching. \n");
			break;
		case UBLOX_STATUS_PREAMBLE_MISMATCH:
			sprintf(statusBuffer, "Error : Received wrong preamble. \n");
			break;
		case UBLOX_STATUS_CRC_FAIL:
			sprintf(statusBuffer, "Error : CRC verification failed. \n");
			break;
		case UBLOX_STATUS_SUCCESS:
			break;
		case UBLOX_STATUS_NONE:
			break;
		case UBLOX_STATUS_ACK_FAILURE:
			sprintf(statusBuffer, "Ack failure\n");
			break;
		}
	}

	ubx_status_t pollNAV_STATUS(char *res)
	{

		ubx_packet_t ubxPacket;
		ubxPacket.header._class = UBX_CLASS_NAV;
		ubxPacket.header.id = UBX_NAV_STATUS;
		ubxPacket.header.payload_length = 0;

		ubx_status_t err = sendCommand(&ubxPacket);
		if (err != UBLOX_STATUS_SUCCESS)
		{
			setStatusMessage(err);
			return err;
		}
		uint8_t flags2 = ubxPacket.payload[HEADER_OFFSET + 7];
		//        switch (flags2 & 0x1B) {
		//            case 0 : return sprintf(res, "UBX-NAV-STATUS : Acquisition");
		//            case 1 : return sprintf(res, "UBX-NAV-STATUS : Tracking");
		//            case 8 : sprintf(res, "UBX-NAV-STATUS : Power optimized tracking"); return strlen(res);
		//            case 16 : return sprintf(res, "UBX-NAV-STATUS : Inactive");
		//            default: return sprintf(res, "UBX-NAV-STATUS : Unknown : %u",flags2);
		//        }
	}

	ubx_status_t pollNAV_SAT(char *res)
	{

		ubx_packet_t ubxPacket;

		ubxPacket.header._class = UBX_CLASS_NAV;
		ubxPacket.header.id = UBX_NAV_SAT;

		uint8_t buf[nav_sat_payload_size];
		ubxPacket.payload = buf;
		ubx_status_t err = sendCommand(&ubxPacket);

		if (err != UBLOX_STATUS_SUCCESS)
		{
			setStatusMessage(err);
			snprintf(res, BUFFER_LENGTH - strlen(res), "navSat send error %s\n", statusBuffer);
			return err;
		}
		err = receiveResponse(&ubxPacket);
		if (err != UBLOX_STATUS_SUCCESS)
		{
			setStatusMessage(err);
			snprintf(res, BUFFER_LENGTH - strlen(res), "navSat recv error %s\n", statusBuffer);
			return err;
		}
		uint8_t numSatellites = (ubxPacket.header.payload_length - 8) / 12;

		sprintf(res, "\nNAV-SAT INFO - ");
		snprintf(res + strlen(res), RESULT_BUFFER_SIZE, "Detected %u satellites\n", numSatellites);

		for (int i = 0; i < numSatellites; i++)
		{

			uint8_t gnssID = ubxPacket.payload[8 + 12 * i];
			uint8_t signalStrength = ubxPacket.payload[10 + 12 * i];
			uint8_t satID = ubxPacket.payload[9 + 12 * i];

			uint32_t flags;
			memcpy(&flags, &ubxPacket.payload[16 + 12 * i], sizeof(uint32_t));

			char *gnssType;
			char *qualityInd;
			uint8_t qualityIndex = flags & 0x7;

			switch (gnssID)
			{
			case 0:
				gnssType = const_cast<char *>("GPS");
				break;
			case 1:
				gnssType = const_cast<char *>("SBAS");
				break;
			case 2:
				gnssType = const_cast<char *>("Galileo");
				break;
			case 3:
				gnssType = const_cast<char *>("BeiDou");
				break;
			case 4:
				gnssType = const_cast<char *>("IMES");
				break;
			case 5:
				gnssType = const_cast<char *>("QZSS");
				break;
			case 6:
				gnssType = const_cast<char *>("GLONASS");
				break;
			default:
				gnssType = const_cast<char *>("Unknown");
				break;
			}
			switch (qualityIndex)
			{
			case 0:
				qualityInd = const_cast<char *>("no signal");
				break;
			case 1:
				qualityInd = const_cast<char *>("searching signal");
				break;
			case 2:
				qualityInd = const_cast<char *>("signal acquired");
				break;
			case 3:
				qualityInd = const_cast<char *>("signal detected but unusable");
				break;
			case 4:
				qualityInd = const_cast<char *>("code locked and time synchronized");
				break;
			default:
				qualityInd = const_cast<char *>("code and carrier locked and time synchronized");
				break;
			}
			if (qualityIndex >= 4)
			{
				snprintf(res + strlen(res), RESULT_BUFFER_SIZE, "%s, %s, %u dBHz\n", gnssType, qualityInd,
						 signalStrength);
			}
			if (strlen(res) >= RESULT_BUFFER_SIZE)
			{
				return err;
			}
		}
		return err;
	}

	ubx_status_t cfgPRT(char *res, uint8_t portID, uint8_t settings)
	{

		ubx_packet_t ubxPacket;

		ubxPacket.header._class = UBX_CLASS_CFG;
		ubxPacket.header.id = UBX_CFG_PRT;
		ubxPacket.header.payload_length = 1;
		ubxPacket.payload[0] = settings;

		ubx_status_t err = sendCommand(&ubxPacket);
		if (err != UBLOX_STATUS_SUCCESS)
		{
			setStatusMessage(err);
			snprintf(res, BUFFER_LENGTH - strlen(res), "cfgPVT send error %s\n", statusBuffer);
			return err;
		}

		ubx_packet_t ackPacket;
		err = receiveResponse(&ubxPacket, false, &ackPacket);
		if (err != UBLOX_STATUS_SUCCESS)
		{
			setStatusMessage(err);
			snprintf(res, BUFFER_LENGTH - strlen(res), "cfgPVT recv error %s\n", statusBuffer);
			return err;
		}
		ubxPacket.header.payload_length = 20;
		ubxPacket.payload[14] = settings;

		err = sendCommand(&ubxPacket);
		if (err != UBLOX_STATUS_SUCCESS)
		{
			setStatusMessage(err);
			snprintf(res, BUFFER_LENGTH - strlen(res), "cfgPVT send error %s\n", statusBuffer);
			return err;
		}
		err = receiveResponse(&ubxPacket, true, &ackPacket);
		if (err != UBLOX_STATUS_SUCCESS)
		{
			setStatusMessage(err);
			snprintf(res, BUFFER_LENGTH - strlen(res), "cfgPVT recv error %s\n", statusBuffer);
			return err;
		}
	}

	ubx_status_t sendCommand(ubx_packet_t *outPacket)
	{
		calcChecksum(outPacket);
		/*
			Check how the command is being sent (I2C, UART, SPI,...)
		*/
		return sendI2CCommand(outPacket);
	}

	ubx_status_t receiveResponse(ubx_packet_t *outPacket,
								 bool expectingAckOnly = false,
								 ubx_packet_t *ackPacket = nullptr)
	{

		auto retVal = UBLOX_STATUS_NONE;

		auto status = waitForResponse(outPacket, ackPacket, expectingAckOnly);

		if (outPacket->header._class == UBX_CLASS_CFG && ackPacket != nullptr)
		{

			if (ackPacket->payload[0] != outPacket->header._class || ackPacket->payload[1] != outPacket->header.id)
			{
#ifdef _hwDebug
				sprintf(statusBuffer, "ACK-MISMATCH \n. ACKed class : %u - id : %u , request was %u - id : %u",
						ackPacket->header._class, ackPacket->header.id, outPacket->header._class, outPacket->header.id);
#endif
				return UBLOX_STATUS_ACK_FAILURE;
			}
			return UBLOX_STATUS_ACK_RECEIVED;
		}
		return status;
	}
	ubx_status_t pollMON_VER(char *res)
	{

		ubx_packet_t ubx_packet;

		ubx_packet.header._class = UBX_CLASS_MON;
		ubx_packet.header.id = UBX_MON_VER;

		uint8_t buf[mon_ver_payload_size];
		ubx_packet.payload = buf;

		ubx_status_t err = sendCommand(&ubx_packet);
		if (err != UBLOX_STATUS_SUCCESS)
		{
			setStatusMessage(err);
			snprintf(res, BUFFER_LENGTH - strlen(res), "monVer send error %s\n", statusBuffer);
			return err;
		}
		err = receiveResponse(&ubx_packet);
		if (err != UBLOX_STATUS_SUCCESS)
		{
			setStatusMessage(err);
			snprintf(res, BUFFER_LENGTH - strlen(res), "monVer recv error %s\n", statusBuffer);
			return err;
		}
		for (int i = 0; i < 100; i++)
		{
			std::cout << ubx_packet.payload[i];
		}
		return UBLOX_STATUS_SUCCESS;
	}

	uint16_t pollNAV_PVT(char *res, int32_t &lat, int32_t &lng, int32_t &height)
	{
		ubx_packet_t ubx_packet;
		uint8_t buffer[300]{};
		ubx_packet.payload = buffer;

		nav_pvt_t navInformation{};

		ubx_packet.header._class = UBX_CLASS_NAV;
		ubx_packet.header.id = 0x02;

		ubx_status_t err = sendCommand(&ubx_packet);
		if (err != UBLOX_STATUS_SUCCESS)
		{
			setStatusMessage(err);
			snprintf(res, BUFFER_LENGTH - strlen(res), "navPVT send error %s\n", statusBuffer);
			return err;
		}
		err = receiveResponse(&ubx_packet);
		if (err != UBLOX_STATUS_SUCCESS)
		{
			setStatusMessage(err);
			snprintf(res, BUFFER_LENGTH - strlen(res), "navPVT recv error %s\n", statusBuffer);
			return err;
		}
		memcpy(&navInformation.longitude, &ubx_packet.payload[4], sizeof(int32_t));
		memcpy(&navInformation.latitude, &ubx_packet.payload[8], sizeof(int32_t));
		memcpy(&navInformation.height_ellipsoid, &ubx_packet.payload[12], sizeof(int32_t));
		memcpy(&navInformation.height_msl, &ubx_packet.payload[16], sizeof(int32_t));

		lat = navInformation.latitude.val;
		lng = navInformation.longitude.val;
		height = navInformation.height_ellipsoid.val;

		// ---------------- test only -----------

		ubx_packet_t ubx_nav_sat{};
		uint8_t buffer2[300]{};
		ubx_nav_sat.payload = buffer2;

		ubx_nav_sat.header._class = 0x01; // UBX_NAV_SAT = 0x35; // Satellite Information
		ubx_nav_sat.header.id = 0x35;

		sendCommand(&ubx_nav_sat);
		receiveResponse(&ubx_nav_sat, false, nullptr);
		auto numSatellites = ubx_nav_sat.payload[5] | 0;
		std::cout << "\nNumber of satellites after ubx nav sat send and recieve: (now at the end of pollNAV_PVT)" << numSatellites << '\n'
				  << std::endl;

		// ----------------

		int n = snprintf(res,
						 RESULT_BUFFER_SIZE - strlen(res),
						 "NAV-HPPOSLLH : \n Latitude : %f , Longitude : %f , Height (MSL) : %f",
						 navInformation.latitude.val / 10000000.f,
						 navInformation.longitude.val / 10000000.f,
						 navInformation.height_msl.val / 1000.f);
		return strlen(res);
	}

	ubx_status_t openPort(const char *fileName)
	{
		s_fd = open(fileName, O_RDWR);

		if (s_fd < 0)
		{
			// setStatusMessage(UBLOX_STATUS_FD_ERROR);
			//			sprintf(res, "Open Port error :\n");
			std::cout << "Open Port error :\n"
					  << std::endl;
			return UBLOX_STATUS_FD_ERROR;
		}
		return UBLOX_STATUS_SUCCESS;
	}
	ubx_status_t sendI2CCommand(ubx_packet_t *outPacket) const
	{

		uint16_t bytesToSend = outPacket->header.payload_length + 8; // How many bytes need to be sent
		uint16_t bytesSent = 0;										 // How many bytes have been sent
		uint16_t bytesLeftToSend = bytesToSend;						 // How many bytes remain to be sent
		uint16_t startSpot = 0;										 // Payload pointer

		while (bytesLeftToSend > 0)
		{
			uint16_t len = bytesLeftToSend;
			if (len > i2cTransactionSize) // Limit len to i2cTransactionSize
				len = i2cTransactionSize;

			// Set how many bytes will be left to send after this  write
			bytesLeftToSend -= len;

			uint8_t out_buf[len];

			if (bytesLeftToSend == 1)
			{
				len -= 1;
				bytesLeftToSend += 1;
			}
			if (bytesSent == 0)
			{

				out_buf[0] = outPacket->header.preambleA;
				out_buf[1] = outPacket->header.preambleB;
				out_buf[2] = outPacket->header._class;
				out_buf[3] = outPacket->header.id;
				out_buf[4] = outPacket->header.payload_length & 0xFF;
				out_buf[5] = outPacket->header.payload_length >> 8;

				bytesSent += 6;

				uint16_t x = 0;
				// Write payload data
				for (; (x < outPacket->header.payload_length) && (bytesSent < len); x++)
				{
					out_buf[6 + x] = outPacket->payload[x];
					bytesSent++;
				}
				if (bytesSent == (len - 2))
				{
					out_buf[bytesSent] = outPacket->checksumA;
					out_buf[bytesSent + 1] = outPacket->checksumB;
					bytesSent += 2;
				}
				struct i2c_msg message = {(__u16)deviceAddress, 0, bytesSent, out_buf};
				struct i2c_rdwr_ioctl_data ioctl_data = {&message, 1};
				int status = ioctl(s_fd, I2C_RDWR, &ioctl_data);
			}
		}
		return UBLOX_STATUS_SUCCESS;
	}

	void calcChecksum(ubx_packet_t *packet)
	{
		packet->checksumA = 0;
		packet->checksumB = 0;

		packet->checksumA += packet->header._class;
		packet->checksumB += packet->checksumA;

		packet->checksumA += packet->header.id;
		packet->checksumB += packet->checksumA;

		packet->checksumA += (packet->header.payload_length & 0xFF);
		packet->checksumB += packet->checksumA;

		packet->checksumA += (packet->header.payload_length >> 8);
		packet->checksumB += packet->checksumA;

		for (uint16_t i = 0; i < packet->header.payload_length; i++)
		{
			packet->checksumA += packet->payload[i];
			packet->checksumB += packet->checksumA;
		}
	}

	ubx_status_t readIncomingI2C(uint8_t *incomingDataBuf)
	{

		struct timeval tv;
		gettimeofday(&tv, NULL);
		uint32_t msTime = (uint32_t)(tv.tv_sec) * 1000 + (uint32_t)(tv.tv_usec) / 1000;
		/*
			To prevent polling the i2c bus too quickly.
		*/
		while ((msTime - lastCheck) < i2cPollingWait)
		{
			gettimeofday(&tv, NULL);
			msTime = (uint32_t)(tv.tv_sec) * 1000 + (uint32_t)(tv.tv_usec) / 1000;
		}
		lastCheck = msTime;

		uint16_t bytesAvailable = 0;

		uint8_t in_buf[2];
		uint8_t out_buf[1];
		struct i2c_rdwr_ioctl_data packet;
		struct i2c_msg messages[2];

		/*
			Register 0xFD and 0xFE store the availabe bytes from the reciever, that are ready to be read.
		*/
		out_buf[0] = 0xFD;

		in_buf[0] = 0;
		in_buf[1] = 0;
		/*
			Poll the amount of available bytes.
		*/
		messages[0].addr = deviceAddress;
		messages[0].flags = 0;
		messages[0].len = sizeof(out_buf);
		messages[0].buf = out_buf;

		messages[1].addr = deviceAddress;
		messages[1].flags = I2C_M_RD;
		messages[1].len = sizeof(in_buf);
		messages[1].buf = in_buf;

		packet.msgs = messages;
		packet.nmsgs = 2;

		int nbytes = ioctl(s_fd, I2C_RDWR, &packet);
		if (nbytes < 0)
		{
			std::cout << errno;
		}
		if (nbytes != 2)
		{

#ifdef _hwDebug
			snprintf(statusBuffer, RESULT_BUFFER_SIZE - strlen(statusBuffer),
					 "I2C Error in request for register FD, bytes returned %i\n", nbytes);
#endif
			return UBLOX_STATUS_I2C_COMM_FAILURE;
		}
		else
		{
			uint8_t msb = in_buf[0];
			uint8_t lsb = in_buf[1];
			bytesAvailable = (uint16_t)msb << 8 | lsb;
		}

		if (bytesAvailable == 0)
		{
#ifdef _hwDebug
			snprintf(statusBuffer,
					 RESULT_BUFFER_SIZE - strlen(statusBuffer),
					 "Available bytes, got %i\n",
					 bytesAvailable);
#endif
			return UBLOX_STATUS_I2C_COMM_FAILURE;
		}

		std::cout << "Bytes available in readIncomingI2C: " << bytesAvailable << std::endl;
		// Check for undocumented bit error. We found this doing logic scans.
		// This error is rare but if we incorrectly interpret the first bit of the two 'data available' bytes as 1
		// then we have far too many bytes to check. May be related to I2C setup time violations: https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/issues/40
		//            if (bytesAvailable & ((uint16_t)1 << 15))
		//            {
		//                // Clear the MSbit
		//                bytesAvailable &= ~((uint16_t)1 << 15);
		//            }

		uint16_t bytesRead = 0;

		struct i2c_msg in_data_msg;
		in_data_msg.addr = deviceAddress;
		in_data_msg.flags = I2C_M_RD;

		// nbytes = ioctl(s_fd, I2C_RDWR, &packets);

		while (bytesAvailable > 0)
		{
			// Limit to 32 bytes or whatever the buffer limit is for given platform
			uint16_t bytesToRead = bytesAvailable; // 16-bit

			if (bytesToRead > i2cTransactionSize)
				bytesToRead = i2cTransactionSize;

			uint8_t in_data_buf[bytesToRead];
			in_data_msg.len = bytesToRead;
			in_data_msg.buf = in_data_buf;

			packet.msgs = &in_data_msg;
			packet.nmsgs = 1;

			int status = ioctl(s_fd, I2C_RDWR, &packet);

			bytesAvailable -= bytesToRead;
			memcpy(&incomingDataBuf[0], in_data_buf, sizeof(in_data_buf));
			bytesRead += bytesToRead;
		}
		return UBLOX_STATUS_SUCCESS;
	}

	ubx_status_t waitForResponse(ubx_packet_t *ubxPacket,
								 ubx_packet_t *ackPacket = nullptr,
								 bool expectingAckOnly = false)
	{

		uint8_t incomingDataBuffer[300];
		const auto status = readIncomingI2C(incomingDataBuffer);

		printf("\nincomingDataBuffer in waitForResponse(...)\n");

		// TODO: remove
		for (int i = 0; i < 300; i++)
		{
			printf("%02X ", incomingDataBuffer[i]);
			if ((i + 1) % 16 == 0)
			{
				printf("\n");
			}
		}

		if (status == UBLOX_STATUS_SUCCESS)
		{
			std::cout << "UBX packet payload " << (uint64_t)(ubxPacket->payload) << std::endl;

			ubxPacket->header.preambleA = incomingDataBuffer[0];
			ubxPacket->header.preambleB = incomingDataBuffer[1];
			ubxPacket->header._class = incomingDataBuffer[2];
			ubxPacket->header.id = incomingDataBuffer[3];
			ubxPacket->header.payload_length = incomingDataBuffer[4] | (incomingDataBuffer[5] << 8);

			if (!expectingAckOnly)
				memcpy(ubxPacket->payload, &incomingDataBuffer[HEADER_OFFSET], ubxPacket->header.payload_length);
			else
				memcpy(ackPacket->payload, &incomingDataBuffer[HEADER_OFFSET], ubxPacket->header.payload_length);

			if (ackPacket != nullptr)
			{
				uint16_t startPoint = ubxPacket->header.payload_length + HEADER_OFFSET + 2;
				if (expectingAckOnly)
					startPoint = 0;

				ackPacket->header.preambleA = incomingDataBuffer[startPoint];
				ackPacket->header.preambleB = incomingDataBuffer[startPoint];
				ackPacket->header.payload_length =
					incomingDataBuffer[startPoint + 4] | (incomingDataBuffer[startPoint + 5] << 8);

				memcpy(ackPacket->payload, &incomingDataBuffer[startPoint + HEADER_OFFSET], 2);
			}
			ubxPacket->checksumA = incomingDataBuffer[HEADER_OFFSET + ubxPacket->header.payload_length];
			ubxPacket->checksumB = incomingDataBuffer[HEADER_OFFSET + ubxPacket->header.payload_length + 1];
		}
		return status;
	}
};

#endif
//
// int main()
// {
// 	SAM_M8Q_GPS gps;
//
// 	char res[1000]{};
// 	ubx_status_t status = gps.openPort("/dev/i2c-7");
// 	if (status != UBLOX_STATUS_SUCCESS)
// 	{
// 		puts(res);
// 	}
// 	//    status = gps.pollMON_VER(res);
//
// 	while (true)
// 	{
//
// 		int32_t latitude{}, longitude{}, height{};
// 		auto resultLength = gps.pollNAV_PVT(res, latitude, longitude, height);
//
// 		puts(res);
// 		std::cout << "Main function results: " << std::endl;
// 		std::cout << "Latitiude " << latitude << std::endl;
// 		std::cout << "Longitude " << longitude << std::endl;
// 		std::cout << "Height " << height << std::endl;
//
// 		//    if (status != UBLOX_STATUS_SUCCESS)
// 		//    {
// 		//        puts(res);
// 		//    }
//
// 		sleep(1);
// 	}
// }
