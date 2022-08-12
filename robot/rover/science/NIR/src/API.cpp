/**
 *
 * This module provides C callable APIs for each of the command supported by NIRscan Nano microcontroller.
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
*/

/** @file
 * @defgroup statusAPI Query APIs
 * @defgroup scanAPI Scan APIs
 * @defgroup configAPI Scan Configuration APIs
 * @defgroup sdAPI SD card manage APIs
 * @defgroup updateAPI Software Update APIs
 * @defgroup calAPI Calibration APIs
 * @defgroup utilAPI Utility APIs
 * @defgroup sensorAPI Sensor Read APIs
 * @defgroup testAPI Board Test APIs
 * @defgroup debugAPI Debug APIs
 */

#include "API.h"
#include "string.h"
#include "usb.h"
#include "Common.h"
#include "dlpspec_scan.h"
#include <stdlib.h>
#include "dlpspec_calib.h"
#include <stdio.h>
#include "NNOStatusDefs.h"
#include "NNOCommandDefs.h"
#include "Serial.h"

static unsigned char OutputBuffer[USB_MAX_PACKET_SIZE+2];
static unsigned char InputBuffer[sizeof(nnoMessageStruct)+USB_MIN_PACKET_SIZE];
static unsigned char seqNum=0;
static uint8 UARTWriteBuffer[1024 * 8];
static uint8 ReadBuffer[1024 * 8];
#define UARTMAXSIZE 8192
static bool g_UARTConnected = false;
int UART_Supported[62] = {1,1,1,1,1,1,1,1,1,1,0,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,
                          1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1};


int uartMesglength = 0;
/*******************************************************************************/
/*                              Private APIs                                   */
/*******************************************************************************/

static int NNO_Write()
/**
 * This function is private to this file. This function writes the contents of OutputBuffer over USB
 *
 * @return  number of bytes written
 *          <0 = error
 *
 */
{
  return USB_Write((void *)OutputBuffer);
}

static int NNO_Read(int isUART = 1 )
/**
 * This function is private to this file. This function is called to write the read control command and then read back a complete HID packet over USB
 * to InputBuffer.
 *
 * @return  number of bytes read
 *          -2 = nack from target
 *          -1 = error reading
 *
 */
{

    nnoMessageStruct *pMsg = (nnoMessageStruct *)InputBuffer;
    short msg_len=USB_MIN_PACKET_SIZE;

    if(g_UARTConnected && isUART)//UART_Supported[pMsg->payload.cmd])
    {
        memcpy(&UARTWriteBuffer[8],&OutputBuffer[1],65);
        pMsg  = (nnoMessageStruct *)&OutputBuffer[1];
        unsigned int Checksum = 0;
        for(int i = 8; i < pMsg->head.length + 12  ; i++)
        {
            Checksum += UARTWriteBuffer[i];
        }


        UARTWriteBuffer[4] = Checksum;
        UARTWriteBuffer[5] = Checksum >> 8;
        UARTWriteBuffer[6] = Checksum >> 16;
        UARTWriteBuffer[7] = Checksum >> 24;

        //write the head and tail
        UARTWriteBuffer[0] = UART_START_IND_BYTE_0;
        UARTWriteBuffer[1] = UART_START_IND_BYTE_1;
        UARTWriteBuffer[2] = UART_START_IND_BYTE_2;
        UARTWriteBuffer[3] = UART_START_IND_BYTE_3;

        UARTWriteBuffer[pMsg->head.length + 12] = UART_END_IND_BYTE_0;
        UARTWriteBuffer[pMsg->head.length + 13] = UART_END_IND_BYTE_1;
        UARTWriteBuffer[pMsg->head.length + 14] = UART_END_IND_BYTE_2;
        UARTWriteBuffer[pMsg->head.length + 15] = UART_END_IND_BYTE_3;


        nnouartMessageStruct *uMsg = (nnouartMessageStruct *)ReadBuffer;
        int read_length = 0;
        int ret;
        if(Serial_Write(UARTWriteBuffer,pMsg->head.length + 16) == 1)
        {
            if(Serial_Read(ReadBuffer, 8 + sizeof(pMsg->head)) == 1)
            {
                read_length = uMsg->head.length;
            }

            else
                return FALSE;

            if((ret = Serial_Read(&ReadBuffer[12], read_length + 4)) == 1)
            {
                //skip the command name when copying from UART to USB structure
                memcpy(&InputBuffer[0],&ReadBuffer[8],4);
                memcpy(&InputBuffer[4],&ReadBuffer[14],read_length);
                return read_length;
            }
            else if(ret < 1)
            {
               return read_length;
            }
            else
                return FALSE;
        }
        return FALSE;
    }

    int ret_val;
    int bytes_read=0;

    if(USB_Write((void *)OutputBuffer) > 0)
    {
        while(bytes_read < msg_len)
        {
            ret_val =  USB_Read(&InputBuffer[bytes_read]);
            if(ret_val < 0)
            {
                return bytes_read;
            }
            else if (ret_val == 0)
                return NNO_READ_TIMEOUT;

            if(bytes_read == 0)//On reading the header of a message, determine the message length or find if a NACK was received
            {
                if(pMsg->head.flags.resp == NNO_RESP_BUSY)
                    return NNO_CMD_BUSY;
                else if((pMsg->head.flags.resp == NNO_RESP_ERROR) || (pMsg->head.length == 0))
                    return NNO_CMD_NACK;

                msg_len = sizeof(pMsg->head) + pMsg->head.length;
            }
            bytes_read += ret_val;
        }
        return bytes_read;
    }
    return FAIL;

}

static int NNO_GetAck(int isUART = 1)
/**
 * This function is private to this file. This function is called to get acknowledgement from device after a command is written
 *
 * @return  0 = ack from targe but no return value OR returns the 4 byte return value from target
 *          -2 = nack from target
 *          -1 = error reading
 *
 */
{
    nnoMessageStruct *pMsg = (nnoMessageStruct *)InputBuffer;
    if(g_UARTConnected && isUART)
    {

        int read_length = 0;
        nnouartMessageStruct *uMsg = (nnouartMessageStruct *)ReadBuffer;

        if(Serial_Read(ReadBuffer, 8 + sizeof(uMsg->head)) == 1)
        {
            read_length = uMsg->head.length;
        }

        else
            return FALSE;

        if(Serial_Read(&ReadBuffer[12], read_length + 4) == 1)
        {
            //skip the command name when copying from UART to USB structure
            memcpy(&InputBuffer[0],&ReadBuffer[8],4);
            memcpy(&InputBuffer[4],&ReadBuffer[14],read_length);

            if(pMsg->head.flags.resp == NNO_RESP_ERROR)
                return NNO_CMD_NACK;
            else if(pMsg->head.flags.resp == NNO_RESP_BUSY)
                return NNO_CMD_BUSY;
            else
            {
                if(pMsg->head.length != 4)
                    return PASS;
                else
                    return (pMsg->payload.data[0] | pMsg->payload.data[1] << 8 | pMsg->payload.data[2] << 16 | pMsg->payload.data[3] << 24);
            }
        }
        else
            return FALSE;

    }
    else
    {
        int ret_val;


        ret_val =  USB_Read(InputBuffer);

        if(ret_val < 0)
            return ret_val;
        else if (ret_val == 0)
            return NNO_READ_TIMEOUT;

        if(pMsg->head.flags.resp == NNO_RESP_ERROR)
            return NNO_CMD_NACK;
        else if(pMsg->head.flags.resp == NNO_RESP_BUSY)
            return NNO_CMD_BUSY;
        else
        {
            if(pMsg->head.length != 4)
                return PASS;
            else
                return (pMsg->payload.data[0] | pMsg->payload.data[1] << 8 | pMsg->payload.data[2] << 16 | pMsg->payload.data[3] << 24);
        }
    }
}

static int NNO_SendMsg(nnoMessageStruct *pMsg, int isUART = 1)
/**
 * This function is private to this file. This function is called to send a whole HID message over USB; in chunks of 64 bytes.
 *
 * @return  number of bytes sent
 *          -1 = FAIL
 *
 */
{
    int maxDataSize = USB_MAX_PACKET_SIZE-sizeof(pMsg->head);
    int dataBytesSent = MIN(pMsg->head.length, maxDataSize);    //Send all data or max possible

    if(g_UARTConnected == true && isUART)
    {

        memcpy(&UARTWriteBuffer[8],pMsg,pMsg->head.length + 4);

        int length = pMsg->head.length + 16;
        unsigned int Checksum = 0;
        uartMesglength = length;
        for(int i = 8; i < pMsg->head.length + 12  ; i++)
        {
            Checksum += UARTWriteBuffer[i];
        }


        UARTWriteBuffer[4] = Checksum;
        UARTWriteBuffer[5] = Checksum >> 8;
        UARTWriteBuffer[6] = Checksum >> 16;
        UARTWriteBuffer[7] = Checksum >> 24;

        //write the head and tail
        UARTWriteBuffer[0] = UART_START_IND_BYTE_0;
        UARTWriteBuffer[1] = UART_START_IND_BYTE_1;
        UARTWriteBuffer[2] = UART_START_IND_BYTE_2;
        UARTWriteBuffer[3] = UART_START_IND_BYTE_3;

        UARTWriteBuffer[pMsg->head.length + 12] = UART_END_IND_BYTE_0;
        UARTWriteBuffer[pMsg->head.length + 13] = UART_END_IND_BYTE_1;
        UARTWriteBuffer[pMsg->head.length + 14] = UART_END_IND_BYTE_2;
        UARTWriteBuffer[pMsg->head.length + 15] = UART_END_IND_BYTE_3;

        int ret = Serial_Write(UARTWriteBuffer,length);
        return ret;

    }

    OutputBuffer[0]=0; // First byte is the report number
    memcpy(&OutputBuffer[1], pMsg, (sizeof(pMsg->head) + dataBytesSent));

    if(NNO_Write() < 0)
        return FAIL;

    while(dataBytesSent < pMsg->head.length)
    {
        memcpy(&OutputBuffer[1], &pMsg->payload.data[dataBytesSent], USB_MAX_PACKET_SIZE);
        if(NNO_Write() < 0)
            return FAIL;
        dataBytesSent += USB_MAX_PACKET_SIZE;
    }
    return dataBytesSent+sizeof(pMsg->head);
}

static int NNO_PrepReadCmd(uint32 cmd_key)
/**
 * This function is private to this file. Prepares the read-control command packet for the given command code and copies it to OutputBuffer.
 *
 * @param   cmd  - I - USB command code.
 *
 * @return  0 = PASS
 *          -1 = FAIL
 *
 */
{
    nnoMessageStruct msg;

    msg.head.flags.rw = 1; //Read
    msg.head.flags.reply = 1; //Host wants a reply from device
    msg.head.flags.dest = 0; //Projector Control Endpoint
    msg.head.flags.reserved = 0;
    msg.head.flags.resp = 0;
    msg.head.seq = 0;

    msg.payload.cmd = CMD_GET_CMD(cmd_key);
    msg.head.length = 2;

   if(cmd_key == NNO_CMD_FLASH_GET_CHKSUM)
    {
        msg.payload.data[2] = 0x00;
        msg.head.length += 1;
    }

    OutputBuffer[0]=0; // First byte is the report number
    memcpy(&OutputBuffer[1], &msg, (sizeof(msg.head)+sizeof(msg.payload.cmd) + msg.head.length));


    return PASS;
}

static int NNO_PrepReadCmdWithParam(uint32 cmd_key, unsigned int param)
/**
 * This function is private to this file. Prepares the read-control command packet for the given command code and parameter and copies it to OutputBuffer.
 *
 * @param   cmd  - I - USB command code.
 * @param   param - I - parameter to be used for tis read command.
 *
 * @return  0 = PASS
 *          -1 = FAIL
 *
 */
{
    nnoMessageStruct msg;

    msg.head.flags.rw = 1; //Read
    msg.head.flags.reply = 1; //Host wants a reply from device
    msg.head.flags.dest = 0; //Projector Control Endpoint
    msg.head.flags.reserved = 0;
    msg.head.flags.resp = 0;
    msg.head.seq = 0;

    msg.payload.cmd = CMD_GET_CMD(cmd_key);
    msg.head.length = 6;

    msg.payload.data[2] = param;
    msg.payload.data[3] = param >>8;
    msg.payload.data[4] = param >>16;
    msg.payload.data[5] = param >>24;

    OutputBuffer[0]=0; // First byte is the report number
    memcpy(&OutputBuffer[1], &msg, (sizeof(msg.head)+sizeof(msg.payload.cmd) + msg.head.length));


    return PASS;
}

static int NNO_PrepWriteCmd(nnoMessageStruct *pMsg, uint32 cmd_key)
/**
 * This function is private to this file. Prepares the write command packet with given command code in the message structure pointer passed.
 *
 * @param   cmd  - I - USB command code.
 * @param   pMsg - I - Pointer to the message.
 *
 * @return  0 = PASS
 *          -1 = FAIL
 *
 */
{
    pMsg->head.flags.rw = 0; //Write
    //if(cmd == FILE_WRITE_DATA)
      // pMsg->head.flags.reply = 0; //No reply expected from device
   // else
        pMsg->head.flags.reply = 1; //Host wants a reply from device
    pMsg->head.flags.dest = 0; //Projector Control Endpoint
    pMsg->head.flags.reserved = 0;
    pMsg->head.flags.resp = 0;
    pMsg->head.seq = seqNum++;

    pMsg->payload.cmd = CMD_GET_CMD(cmd_key);
    pMsg->head.length = CMD_GET_LEN(cmd_key) + 2;



    return PASS;
}

//static int NNO_PerformScanFlashPatterns(uint8 scanID)
/* Not tested as of now */
/*{
    nnoMessageStruct msg;
    int retval;

    msg.payload.data[2] = scanID;

    NNO_PrepWriteCmd(&msg, NNO_CMD_PERFORM_SCAN_FLASH_PTNS);

    retval = NNO_SendMsg(&msg);
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;

}*/

/*******************************************************************************/
/*                         APIs that query status                              */
/*******************************************************************************/
int NNO_GetVersion(uint32 *pTivaSWVersion, uint32 *pDLPCSWVersion, uint32 *pDLPCFlashBuildVersion, \
                   uint32 *pSpecLibVer, uint32 *pCalDataVer, uint32 *pRefCalDataVer, uint32 *pCfgDataVer)
/**
 *
 * @brief This API reads the Software revision number from Tiva.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x16)
 *
 * @ingroup statusAPI
 *
 * @param   pTivaSWVersion - O - Tiva SW revision information in MAJOR.MINOR.PATCH in byte 2, 1 and 0 respectively
 * @param   pDLPCSWVersion - O - DLPC SW revision information in MAJOR.MINOR.PATCH in byte 2, 1 and 0 respectively
 * @param   pDLPCFlashBuildVersion - O - DLPC flash image revision information in MAJOR.MINOR.PATCH in byte 2, 1 and 0 respectively
 * @param   pSpecLibVer - O - Spectrum Library revision information in MAJOR.MINOR.PATCH in byte 2, 1 and 0 respectively
 * @param   pCalDataVer - O - Calibration data revision information in MAJOR.MINOR.PATCH in byte 2, 1 and 0 respectively
 * @param   pRefCalDataVer - O - RefCal data revision information in MAJOR.MINOR.PATCH in byte 2, 1 and 0 respectively
 * @param   pCfgDataVer - O - Scan Config data revision information in MAJOR.MINOR.PATCH in byte 2, 1 and 0 respectively
 *
 * @return  >=0 = PASS    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    nnoMessageStruct msg;
    int ret_val;

    NNO_PrepReadCmd(NNO_CMD_TIVA_VER);
    if((ret_val = NNO_Read()) > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *pTivaSWVersion = msg.payload.data[0] | msg.payload.data[1] << 8 | msg.payload.data[2] << 16 | msg.payload.data[3] << 24;
        *pDLPCSWVersion = msg.payload.data[4] | msg.payload.data[5] << 8 | msg.payload.data[6] << 16 | msg.payload.data[7] << 24;
        *pDLPCFlashBuildVersion = msg.payload.data[8] | msg.payload.data[9] << 8 | msg.payload.data[10] << 16 | msg.payload.data[11] << 24;
        *pSpecLibVer = msg.payload.data[12] | msg.payload.data[13] << 8 | msg.payload.data[14] << 16 | msg.payload.data[15] << 24;
        *pCalDataVer = msg.payload.data[16] | msg.payload.data[17] << 8 | msg.payload.data[18] << 16 | msg.payload.data[19] << 24;
        *pRefCalDataVer = msg.payload.data[20] | msg.payload.data[21] << 8 | msg.payload.data[22] << 16 | msg.payload.data[23] << 24;
        *pCfgDataVer = msg.payload.data[24] | msg.payload.data[25] << 8 | msg.payload.data[26] << 16 | msg.payload.data[27] << 24;
        return PASS;
    }
    return ret_val;
}

int NNO_TMP_GetErrorMessageFromErrorCode(uint32_t errorCode, char *pErrorString)
/**
 * This function gets the particular type of TEMP error from the error Code
 * @param   errorCode  - I - the errorCode code.
 * @param   pErrorString - I - Pointer to the message.
 *
 * @return  0 = PASS
 *          -1 = FAIL
 *
 */
{
    switch(errorCode)
    {
        case PASS:
            strcpy(pErrorString, "PASS");
            break;
        case NNO_ERROR_TMP006_RESET:
            strcpy(pErrorString, "ERROR_RESET");
            break;
        case NNO_ERROR_TMP006_MANUID:
            strcpy(pErrorString, "ERROR_MANUID");
            break;
        case NNO_ERROR_TMP006_DEVID:
            strcpy(pErrorString, "ERROR_DEVID");
            break;
        case NNO_ERROR_TMP006_READREGISTER:
            strcpy(pErrorString, "ERROR_READREGISTER");
            break;
        case NNO_ERROR_TMP006_WRITEREGISTER:
            strcpy(pErrorString, "ERROR_WRITEREGISTER");
            break;
        case NNO_ERROR_TMP006_I2C:
            strcpy(pErrorString, "ERROR_I2C");
            break;
        case NNO_ERROR_TMP006_TIMEOUT:
            strcpy(pErrorString, "ERROR_TIMEOUT");
            break;
        default:
            strcpy(pErrorString, "ERROR_UNDEFINED");
            break;
    }
    return PASS;
}

int NNO_HDC_GetErrorMessageFromErrorCode(uint32_t errorCode, char *pErrorString)
/**
 * This function gets the particular type of HDC error from the error Code
 * @param   errorCode  - I - the errorCode code.
 * @param   pErrorString - I - Pointer to the message.
 *
 * @return  0 = PASS
 *          -1 = FAIL
 *
 */
{
    switch(errorCode)
    {
        case PASS:
            strcpy(pErrorString, "PASS");
            break;
        case NNO_ERROR_HDC1000_RESET:
            strcpy(pErrorString, "ERROR_RESET");
            break;
        case NNO_ERROR_HDC1000_MANUID:
            strcpy(pErrorString, "ERROR_MANUID");
            break;
        case NNO_ERROR_HDC1000_DEVID:
            strcpy(pErrorString, "ERROR_DEVID");
            break;
        case NNO_ERROR_HDC1000_READREGISTER:
            strcpy(pErrorString, "ERROR_READREGISTER");
            break;
        case NNO_ERROR_HDC1000_WRITEREGISTER:
            strcpy(pErrorString, "ERROR_WRITEREGISTER");
            break;
        case NNO_ERROR_HDC1000_I2C:
            strcpy(pErrorString, "ERROR_I2C");
            break;
        case NNO_ERROR_HDC1000_TIMEOUT:
            strcpy(pErrorString, "ERROR_TIMEOUT");
            break;
        default:
            strcpy(pErrorString, "ERROR_UNDEFINED");
        break;
    }
    return PASS;
}

int NNO_ADC_GetErrorMessageFromErrorCode(uint32_t errorCode, char *pErrorString)
/**
 * This function gets the particular type of ADC error from the error Code
 * @param   errorCode  - I - the errorCode code.
 * @param   pErrorString - I - Pointer to the message.
 *
 * @return  0 = PASS
 *          -1 = FAIL
 *
 */
{
    switch(errorCode)
    {
        case PASS:
            strcpy(pErrorString, "PASS");
            break;
        case ADC_ERROR_TIMEOUT:
            strcpy(pErrorString, "ERROR_TIMEOUT");
            break;
        case ADC_ERROR_POWERDOWN:
            strcpy(pErrorString, "ERROR_POWERDOWN");
            break;
        case ADC_ERROR_POWERUP:
            strcpy(pErrorString, "ERROR_POWERUP");
            break;
        case ADC_ERROR_STANDBY:
            strcpy(pErrorString, "ERROR_STANDBY");
            break;
        case ADC_ERROR_WAKEUP:
            strcpy(pErrorString, "ERROR_WAEKUP");
            break;
        case ADC_ERROR_READREGISTER:
            strcpy(pErrorString, "ERROR_READREGISTER");
            break;
        case ADC_ERROR_WRITEREGISTER:
            strcpy(pErrorString, "ERROR_WRITEREGISTER");
            break;
        case ADC_ERROR_CONFIGURE:
            strcpy(pErrorString, "ERROR_CONFIGURE");
            break;
        case ADC_ERROR_SETBUFFER:
            strcpy(pErrorString, "ERROR_SETBUFFER");
            break;
        default:
            strcpy(pErrorString, "ERROR_UNDEFINED");
            break;
    }
    return PASS;
}

int NNO_SDC_GetErrorMessageFromErrorCode(uint32_t errorCode, char *pErrorString)
/**
 * This function gets the particular type of SDC error from the error Code
 * @param   errorCode  - I - the errorCode code.
 * @param   pErrorString - I - Pointer to the message.
 *
 * @return  0 = PASS
 *          -1 = FAIL
 *
 */
{
    switch(errorCode)
    {
        case PASS:
            strcpy(pErrorString, "PASS");
            break;
        case  NNO_ERROR_SD_CARD_HARD_ERROR:
            strcpy(pErrorString, "ERROR_DISK_ERR");
            break;
        case  NNO_ERROR_SD_CARD_INTERNAL_ERRROR:
            strcpy(pErrorString, "ERROR_INT_ERR");
            break;
        case  NNO_ERROR_SD_CARD_DOES_NOT_WORK:
            strcpy(pErrorString, "ERROR_NOT_READY");
            break;
        case  NNO_ERROR_SD_CARD_FILE_NOT_FOUND:
            strcpy(pErrorString, "ERROR_NO_FILE");
            break;
        case  NNO_ERROR_SD_CARD_PATH_NOT_FOUND:
            strcpy(pErrorString, "ERROR_NO_PATH");
            break;
        case  NNO_ERROR_SD_CARD_INVALID_PATH:
            strcpy(pErrorString, "ERROR_INVALID_NAME");
            break;
        case  NNO_ERROR_SD_CARD_ACCESS_DENIED:
            strcpy(pErrorString, "ERROR_DENIED");
            break;
        case  NNO_ERROR_SD_CARD_ACCESS_PROHIBITED:
            strcpy(pErrorString, "ERROR_EXIST");
            break;
        case  NNO_ERROR_SD_CARD_INVALID_OBJECT:
            strcpy(pErrorString, "ERROR_INVALID_OBJECT");
            break;
        case  NNO_ERROR_SD_CARD_WRITE_PROTECTED:
            strcpy(pErrorString, "ERROR_WRITE_PROTECTED");
            break;
        case  NNO_ERROR_SD_CARD_INVALID_DRIVE_NUM:
            strcpy(pErrorString, "ERROR_INVALID_DRIVE");
            break;
        case  NNO_ERROR_SD_CARD_NOT_ENABLED:
            strcpy(pErrorString, "ERROR_NOT_ENABLED");
            break;
        case  NNO_ERROR_SD_CARD_INVALID_FILE_SYSTEM:
            strcpy(pErrorString, "ERROR_NO_FILESYSTEM");
            break;
        case  NNO_ERROR_SD_CARD_MKFS_INVALID_PARAM:
            strcpy(pErrorString, "ERROR_MKFS_ABORTED");
            break;
        case NNO_ERROR_SD_CARD_TIMEOUT:
            strcpy(pErrorString, "ERROR_TIMEOUT");
            break;
        case  NNO_ERROR_SD_CARD_LOCKED:
            strcpy(pErrorString, "ERROR_LOCKED");
            break;
        case  NNO_ERROR_SD_CARD_NOT_ENOUGH_CORE:
            strcpy(pErrorString, "ERROR_NOT_ENOUGH_CORE");
            break;
        case  NNO_ERROR_SD_CARD_TOO_MANY_OPEN_FILES:
             strcpy(pErrorString, "ERROR_TOO_MANY_OPEN_FILES");
            break;
        default:
            strcpy(pErrorString, "ERROR_UNDEFINED");
            break;
    }

    return PASS;
}

int8_t NNO_GetSpecificErrorStatus(uint32_t field)
/**
 * Reads the specified error status
 *
 * (USB CMD OPCODE: BYTE1: 0x04, BYTE2: 0x06)
 *
 * @ingroup debugAPI
 *
 * @param field - I - Error field of interest as defined in NNOStatusDefs.h
 *
 * @return  1 if the error is set 0 if not; -1 if read failed
 *
 */
{
    nnoMessageStruct *pMsg;
    int ret_val;

    NNO_PrepReadCmdWithParam(NNO_CMD_GET_SPECIFIC_ERR_STATUS, field);

    if((ret_val = NNO_Read()) > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        return pMsg->payload.data[0];
    }
    return ret_val;
}

int16_t NNO_GetSpecificErrorcode(uint8_t type)
/**
 * Reads the specified error code
 *
 * (USB CMD OPCODE: BYTE1: 0x04, BYTE2: 0x06)
 *
 * @ingroup debugAPI
 *
 * @param type - I - Error type of interest as defined in NNOStatusDefs.h
 * NNO_error_codes_type
 *
 * @return  Error code as defined in NNOStatusDefs.h
 *
 */
{
    nnoMessageStruct *pMsg;
    int ret_val;

    NNO_PrepReadCmdWithParam(NNO_CMD_GET_SPECIFIC_ERR_CODE, type);

    if((ret_val = NNO_Read()) > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        return (pMsg->payload.data[0] | (pMsg->payload.data[1] << 8));
    }
    return ret_val;
}

int8_t NNO_ClearSpecificError(uint32_t field)
/**
 * Reads the specified error status
 *
 * (USB CMD OPCODE: BYTE1: 0x04, BYTE2: 0x08)
 *
 * @ingroup debugAPI
 *
 * @param field - I - Error field of interest as defined in NNOStatusDefs.h
 *
 * @return  1 if the error is set 0 if not; -1 if read failed
 *
 */
{
    nnoMessageStruct msg;
    int retval;

    msg.payload.data[2] = field;
    msg.payload.data[3] = field >>8;
    msg.payload.data[4] = field >>16;
    msg.payload.data[5] = field >>24;


    NNO_PrepWriteCmd(&msg, NNO_CMD_CLEAR_SPECIFIC_ERR);

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

/*******************************************************************************/
/*                         Scan related APIs								   */
/*******************************************************************************/

int NNO_DLPCEnable(bool enable, bool enable_lamp)
/**
 * Enables or Disables DLPC150; initializes DLPC150 for scans with patterns from Video interface or internal flash
 * and also enables/disables the lamp.
 *
 * (USB CMD OPCODE: BYTE1: 0x01, BYTE2: 0x05)
 *
 * @ingroup scanAPI
 *
 * @param enable - I - Releases DLPC150 from reset when TRUE and puts under
 * reset when FALSE.(as per scanType enum in API.h)
 * @param enable_lamp - I - true = turn on the lamp; false = leave lamp turned off.
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;    int retval;

    msg.payload.data[2] = enable;
    msg.payload.data[3] = enable_lamp;
    NNO_PrepWriteCmd(&msg, NNO_CMD_DLPC_ENABLE);

    retval = NNO_SendMsg(&msg);
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

int NNO_PerformScan(bool StoreInSDCard)
/**
 * Initiates a scan.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x18)
 *
 * @ingroup scanAPI
 *
 * @param StoreInSDCard - I - Stores scan results in SD card when TRUE.
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;
    int retval;

    msg.payload.data[2] = StoreInSDCard;
    NNO_PrepWriteCmd(&msg, NNO_CMD_PERFORM_SCAN );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;

}

int NNO_InterpretScan(void)
/**
 * Initiates a Scan Interpretation.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x39)
 *
 * @ingroup scanAPI
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;

    int retval;

    NNO_PrepWriteCmd(&msg, NNO_CMD_START_SCAN_INTERPRET);

    retval = NNO_SendMsg(&msg);
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;

}


int NNO_GetScanComplete(void)
/**
 * API for polling scan completion status after a scan was initiated using NNO_PerformScan() API.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x19)
 *
 * @ingroup scanAPI
 *
 * @return enable - O = Scan in progress
 *                  1 = Scan complete
 *                  <0 = Status read failed.
 *
 */
{
    nnoMessageStruct *pMsg;
    char ret_val;

    NNO_PrepReadCmd(NNO_CMD_SCAN_GET_STATUS);
    if((ret_val = NNO_Read()) > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        ret_val = pMsg->payload.data[0];
        return (int)ret_val;
    }
    return ret_val;
}

int NNO_GetEstimatedScanTime(void)
/**
 * Returns the expected time for scan in milliseconds.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x37)
 *
 * @ingroup scanAPI
 *
 * @return expected duration of scan in milliseconds
 *
 */
{
    nnoMessageStruct *pMsg;
    int ret_val;

    NNO_PrepReadCmd(NNO_CMD_READ_SCAN_TIME);
    if((ret_val = NNO_Read()) > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        ret_val = pMsg->payload.data[0] | pMsg->payload.data[1] << 8 | pMsg->payload.data[2] << 16 | pMsg->payload.data[3] << 24;
        return ret_val;
    }
    return ret_val;
}

int NNO_GetFileSizeToRead(NNO_FILE_TYPE fileType)
/**
 * This function is to be called before calling NNO_GetFile() API to determine the size of file that
 * the NIRscan Nano unit will return on a subsequent call to NNO_GetFile() API.
 * The value returned by this API needs to be passed to the NNO_GetFile() API as the sizeInBytes argument.
 *
 * (USB CMD OPCODE: BYTE1: 0x00, BYTE2: 0x2D)
 *
 * @ingroup scanAPI
 *
 * @param fileType - I - Which file we intend to read from the NIRscan Nano
 *
 * @return number of bytes that will be return on subsequent NNO_GetFile() call <BR>
 *         <0 FAIL <BR>
 *
 */
{
    nnoMessageStruct msg;
    int fileSize;
    int ret_val;

    NNO_PrepReadCmdWithParam(NNO_CMD_FILE_GET_READSIZE, fileType);

    if((ret_val = NNO_Read()) > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        fileSize = msg.payload.data[0] | msg.payload.data[1] << 8 | msg.payload.data[2] << 16 | msg.payload.data[3] << 24;
        return fileSize;
    }
    return ret_val;
}

int NNO_GetFileData(void *pData, int *pSizeInBytes)
/**
 * This command reads one complete packet of file data (max 512 bytes of payload).
 * Several such reads may be performed to get a compelte file from the EVM.
 * NNO_GetFileSizeToRead() API shall be called to know how much data overall to expect from the EVM for a given file.
 *
 * (USB CMD OPCODE: BYTE1: 0x00, BYTE2: 0x2E)
 *
 * @ingroup scanAPI
 *
 * @param   pData -I - pointer at which file data read from EVM will be stored (caller has to allocate and deallocate this memory)
 * @param   pSizeInBytes - O - Number of bytes of data actually read as part of this packet.
 *
 * @return  0 = PASS    <BR>
 *          -1 = FAIL  <BR>
 *
 */
{
    nnoMessageStruct *pMsg = (nnoMessageStruct *)InputBuffer;
    int ret;

    NNO_PrepReadCmd(NNO_CMD_FILE_GET_DATA);

    if((ret = NNO_Read()) > 0)
    {
        if(g_UARTConnected)
        {
           pMsg->head.length =  pMsg->head.length - 2;
        }
        memcpy(pData, &pMsg->payload.data[0], pMsg->head.length);
        *pSizeInBytes = pMsg->head.length;
    }
    return ret;
}

int NNO_GetFile(unsigned char *pData, int sizeInBytes)
/**
 * This command reads one complete file from the EVM.
 * It issues several NNO_GetFileData() commands to read the complete file.
 *
 * @param   pData -I - pointer at which file data read from EVM will be stored (caller has to allocate and deallocate this memory)
 * @param   sizeInBytes - O - Number of bytes to be expected in the file. To be obtained by calling NNO_GetFileSizeToRead().
 *
 * @return  Total Bytes read    <BR>
 *          <0 = FAIL  <BR>
 *
 */
{
    unsigned char *pPacket;
    int bytesRead;
    int totalBytesRead = 0;

    if(pData == NULL)
        return FAIL;

    pPacket = (unsigned char *)malloc(sizeof(nnoMessageStruct)+ 64);


    while(sizeInBytes > 0)
    {
        if(NNO_GetFileData(pPacket, &bytesRead) < 0)
        {
            free(pPacket);
            return totalBytesRead;
        }

        memcpy(pData, pPacket, bytesRead);
        pData += bytesRead;
        sizeInBytes -= bytesRead;
        totalBytesRead += bytesRead;
    }

    if(pPacket != NULL)
     free(pPacket);

    return totalBytesRead;
}

int NNO_SetScanControlsDLPCOnOff(bool enable)
/**
 * When enabled; each scan command does not enable/disable DLPC150 and the lamp.
 * The caller will have to manage DLPC150 and Lamp ON/OFF controls using NNO_DLPCEnable().
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x25)
 *
 * @ingroup scanAPI
 *
 * @param enable - O - true enables this behavior and false turns off this behavior.
 *
 * @return  PASS or FAIL
 *
 */
{

    nnoMessageStruct msg;

    int retval;

    msg.payload.data[2] = enable;
    NNO_PrepWriteCmd(&msg, NNO_CMD_SET_DLPC_ONOFF_CTRL );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

int NNO_SetScanNumRepeats(uint16 num)
/**
 * After applying a scan config using NNO_ApplyScanConfig() or NNO_SetActiveScanIndex(), this API can be used to override the
 * number of times the scan is to be repeated for averaging.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x2E)
 *
 * @ingroup scanAPI
 *
 * @param num - I - number of repeats
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;

    int retval;

    msg.payload.data[2] = num;
    msg.payload.data[3] = num >> 8;
    NNO_PrepWriteCmd(&msg, NNO_CMD_SCAN_NUM_REPEATS );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

/*******************************************************************************/
/*                         Scan config management APIs						   */
/*******************************************************************************/
#include "iostream"

using namespace std;
int NNO_SaveScanCfgInEVM(uint8 index, void *pBuffer, int bufSize)
/**
 * Saves given scan config to the EEPROM in EVM.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x1F)
 *
 * @ingroup configAPI
 *
 * @param index   - I - Index at which the given scanConfig needs to be saved.
 * @param pBuffer - I - Pointer to scanConfig structure data in serialized form.
 * @param bufSize - I - Size of data in bytes upon serialization.
 *
 * @return  PASS or FAIL
 *
 */
{
    cout << "buffer size : " << bufSize << endl;
    nnoMessageStruct msg;

    int retval;

    msg.payload.data[2] = index;
    msg.payload.data[3] = bufSize;
    memcpy(&msg.payload.data[4], pBuffer, bufSize);
    NNO_PrepWriteCmd(&msg, NNO_CMD_SCAN_CFG_SAVE );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

int NNO_ApplyScanConfig(void *pBuffer, int bufSize)
/**
 * Applies given scan config to the EVM.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x1E)
 *
 * @ingroup configAPI
 *
 * @param pBuffer - I - Pointer to scanConfig structure data in serialized form.
 * @param bufSize - I - Size of data in bytes upon serialization.
 *
 * @return  < 0 = FAIL; else number of patterns generated for this scan config.
 *
 */
{

    nnoMessageStruct msg;

    int retval;

    memcpy(&msg.payload.data[2], pBuffer, bufSize);
    NNO_PrepWriteCmd(&msg, NNO_CMD_SCAN_CFG_APPLY );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;

}

int NNO_SetActiveScanIndex(uint8 index)
/**
 * Sets the scan configuration at specified index in EVM EEPROM as the active one to be used in subsequent
 * scans commanded by the scan "button" in the EVM.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x24)
 *
 * @ingroup configAPI
 *
 * @param index - I - scanConfig to be set as active.
 *
 * @return PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;

    int retval;

    msg.payload.data[2] = index;
    NNO_PrepWriteCmd(&msg, NNO_CMD_SCAN_SET_ACT_CFG );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

int NNO_GetActiveScanIndex(void)
/**
 * Returns the index of currently active scan configuration from among the ones stored in the EVM EEPROM.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x23)
 *
 * @ingroup configAPI
 *
 * @return index of currently active scan config.
 *          <0 = FAIL
 *
 */
{
    nnoMessageStruct *pMsg;
    int ret_val;

    NNO_PrepReadCmd(NNO_CMD_SCAN_GET_ACT_CFG);

    if((ret_val = NNO_Read()) > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        return pMsg->payload.data[0];
    }
    return ret_val;
}

int NNO_GetNumScanCfg(void)
/**
 * Returns total number of scan configurations stored in the EVM EEPROM.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x22)
 *
 * @ingroup configAPI
 *
 * @return number of scan configs stored in the EVM
 *          <0 = FAIL
 *
 */
{
    nnoMessageStruct *pMsg;
    int ret_val;

    NNO_PrepReadCmd(NNO_CMD_SCAN_CFG_NUM);

    if((ret_val = NNO_Read()) > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        return pMsg->payload.data[0];
    }
    return ret_val;
}

int NNO_GetScanCfg(uint8 index, void *pBuf, uint32 *pSize)
/**
 * Reads the scan configuration stored at the specified index in EEPROM on EVM. The config data
 * is retuned in serialized form and has to be deserialized using the spectrum library function
 * dlpspec_scan_read_configuration().
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x20)
 *
 * @ingroup configAPI
 *
 * @param index - I - index from which scan config is to be read
 * @param pBuf - O - Pointer at which scanConfig data will be returned
 * @param pSize - O - number of bytes returned in pBuf pointer
 *
 * @return PASS or FAIL.
 *
 */
{
    nnoMessageStruct *pMsg = (nnoMessageStruct *)InputBuffer;
    int ret_val;

    NNO_PrepReadCmdWithParam(NNO_CMD_SCAN_CFG_READ, index);

    if((ret_val = NNO_Read()) > 0)
    {
        *pSize = pMsg->payload.data[0] | pMsg->payload.data[1] << 8  | pMsg->payload.data[2] << 16 | pMsg->payload.data[3] << 24;;
        memcpy(pBuf, &(pMsg->payload.data[4]), *pSize);
        return PASS;
    }
    return ret_val;
}

int NNO_EraseAllScanCfg(void)
/**
 * Command to erase all scan configurations stored in the EVM EEPROM except the very first
 * scanConfig that is stored during factory calibration process. The application at startup
 * assumes at least one scanConfig is available in EEPROM. Hence the default/first one is
 * never erased.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x21)
 *
 * @ingroup configAPI
 *
 * @return PASS or FAIL.
 *
 */
{
    nnoMessageStruct msg;

    int retval;

    NNO_PrepWriteCmd(&msg, NNO_CMD_SCAN_CFG_ERASEALL );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

/*******************************************************************************/
/*					APIs for managing files in SD card						   */
/*******************************************************************************/

int NNO_GetNumScanFilesInSD(void)
/**
 * Returns the number of scan result files stored in SD card inserted in the EVM.
 *
 * (USB CMD OPCODE: BYTE1: 0x04, BYTE2: 0x00)
 *
 * @ingroup sdAPI
 *
 * @return index of scan files in SD card
 *          <0 = FAIL
 *
 */
{
    nnoMessageStruct msg;
    int num;
    int ret_val;

    NNO_PrepReadCmd( NNO_CMD_GET_NUM_SCAN_FILES_SD );
    if((ret_val = NNO_Read()) > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        num = msg.payload.data[0] | msg.payload.data[1] << 8 | msg.payload.data[2] << 16 | msg.payload.data[3] << 24;
        return num;
    }
    return ret_val;
}

int NNO_DeleteLastScanFileInSD(void)
/**
 * Deletes the last scan file stored in SD card. This command can be used repeatitively to delete all files from SD card if desired.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x38)
 *
 * @ingroup sdAPI
 *
 * @return PASS or FAIL
 *
 */
{

    nnoMessageStruct msg;

    int retval;

    NNO_PrepWriteCmd( &msg, NNO_CMD_DEL_LAST_SCAN_FILE_SD  );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

/*******************************************************************************/
/*					Software update releated APIs							   */
/*******************************************************************************/

int NNO_GotoTivaBootLoader(void)
/**
 * Use this API to jump to Tiva bootloader and then tiva firmware can be updated.
 *
 * (USB CMD OPCODE: BYTE1: 0x00, BYTE2: 0x2F)
 *
 * @ingroup updateAPI
 */
{
    nnoMessageStruct msg;


    NNO_PrepWriteCmd(&msg, NNO_CMD_GOTO_TIVA_BL );

    return NNO_SendMsg(&msg,0);
}

int NNO_SetFileSizeAndAction(unsigned int dataLen, NNO_FILE_ACTION action)
/**
 * This function is to be called to set the payload size of data to be sent using NNO_DownloadData API.
 * (USB CMD OPCODE: BYTE1: 0x00, BYTE2: 0x2C)
 *
 * @ingroup updateAPI
 *
 * @param dataLen -I - length of download data payload in bytes.
 * @param action - I - Instructs Nano what action to perform with the payload data that will follow.
 *
 * @return >=0 PASS <BR>
 *         <0 FAIL <BR>
 *
 */
{
    nnoMessageStruct msg;

    int retval;

    msg.payload.data[2] = dataLen;
    msg.payload.data[3] = dataLen >> 8;
    msg.payload.data[4] = dataLen >> 16;
    msg.payload.data[5] = dataLen >> 24;
    msg.payload.data[6] = action;
    msg.payload.data[7] = action >> 8;

    NNO_PrepWriteCmd(&msg, NNO_CMD_FILE_SET_WRITESIZE );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

int NNO_WriteFileData(unsigned char *pByteArray, unsigned int dataLen)
/**
 * This function sends one payload of data to the controller at a time. takes the total size of payload
 * in the parameter dataLen and returns the actual number of bytes that was sent in the return value.
 * This function needs to be called multiple times until all of the desired bytes are sent.
 * (USB CMD OPCODE: BYTE1: 0x00, BYTE2: 0x25)
 *
 * @ingroup updateAPI
 *
 * @param pByteArray - I - Pointer to where the data to be downloaded is to be fetched from
 * @param dataLen -I - length in bytes of the total payload data to download.
 *
 * @return number of bytes actually downloaded <BR>
 *         <0 FAIL <BR>
 *
 */
{
    nnoMessageStruct msg;

    int retval;
    unsigned int sendSize;

    if(g_UARTConnected)//todo get an apt downloadable size over UART
        sendSize = NNO_UART_MAX_DATA_SIZE - sizeof(msg.head) - sizeof(msg.payload.cmd) - 2;
    else //leaving USB unchanged
        sendSize = NNO_DATA_MAX_SIZE - sizeof(msg.head) - sizeof(msg.payload.cmd) - 2;//The last -2 is to workaround a bug in bootloader.



    if(dataLen > sendSize)
        dataLen = sendSize;

    memcpy(&msg.payload.data[2], pByteArray, dataLen);

    NNO_PrepWriteCmd(&msg, NNO_CMD_FILE_WRITE_DATA );
    msg.head.length = dataLen+2;

    retval = NNO_SendMsg(&msg);
    if(retval > 0)
    {
        retval = NNO_GetAck();
        if(retval >= 0)
            retval =  dataLen;
    }

    return retval;
}

int NNO_GetFlashChecksum(unsigned int *checksum)
/**
 * This function is to be used to retrieve the flash checksum from the controller.
 * (USB CMD OPCODE: BYTE1: 0x00, BYTE2: 0x15)
 *
 * @ingroup updateAPI
 *
 * @param checksum - O - variable in which the flash checksum is to be returned
 *
 * @return >=0 PASS <BR>
 *         <0 FAIL <BR>
 *
 */
{
    nnoMessageStruct msg;
    int ret_val;

    //Set USB timeout higher as computation of checksum is time intensive
    USB_setTimeout(40000);
    //Serial_SetTimeout(500);

    NNO_PrepReadCmd(NNO_CMD_FLASH_GET_CHKSUM);
    if((ret_val = NNO_Read()) > 0)
    {
        memcpy(&msg, InputBuffer, 65);

        *checksum = msg.payload.data[0];
        *checksum |= (unsigned int)msg.payload.data[1] << 8;
        *checksum |= (unsigned int)msg.payload.data[2] << 16;
        *checksum |= (unsigned int)msg.payload.data[3] << 24;

        USB_setDefaultTimeOut();
       // Serial_SetTimeout(50);

        return PASS;
    }
    USB_setDefaultTimeOut();
    return ret_val;
}

/*******************************************************************************/
/*							Calibration related APIs						   */
/*******************************************************************************/

int NNO_GenCalibPatterns(CALIB_SCAN_TYPES type)
/**
 * Commands Tiva to generate patterns required for the given type of calibration scan.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x2D)
 *
 * @ingroup calAPI
 *
 * @param type - I - Type of scan as per CALIB_SCAN_TYPES enum defined in dlpspsec_calib.h
 *
 *  @return  < 0 = FAIL; else number of patterns generated for this scan config.
 *
 */
{
    nnoMessageStruct msg;

    int retval;

    msg.payload.data[2] = type;
    NNO_PrepWriteCmd(&msg, NNO_CMD_CALIB_GEN_PTNS );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

int NNO_setScanSubImage(uint16 startY, uint16 height)
/**
 * Specify a subimage of patterns to be used in scan if patterns covering the entire DMD height is not desired.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x26)
 *
 * @ingroup calAPI
 *
 * @param startY - I - lines above startY line will be cut out from being displayed on the DMD. These lines will be black/turned off.
 * @param height - I - lines below "height" line will be cut out from being displayed on the DMD. These lines will be black/turned off.
 *
 * @return PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;

    int retval;

    msg.payload.data[2] = startY;
    msg.payload.data[3] = startY >> 8;
    msg.payload.data[4] = height;
    msg.payload.data[5] = height >> 8;
    NNO_PrepWriteCmd(&msg, NNO_CMD_SET_SCAN_SUBIMAGE );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

int NNO_SaveRefCalPerformed(void)
/**
 * Command to store reference calibration data to EEPROM after a refCal scan has been performed.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x30)
 *
 * @ingroup calAPI
 *
 * @return PASS or FAIL.
 *
 */
{
    nnoMessageStruct msg;

    int retval;

    msg.payload.data[2] = 0;
    NNO_PrepWriteCmd(&msg, NNO_CMD_REFCAL_PERFORM);

    retval = NNO_SendMsg(&msg);
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

int NNO_StartSNRScan(void)
/**
 * Initiates the special scan sequence for SNR computation.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x2B)
 *
 * @ingroup calAPI
 *
 * @return PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;

    int retval;

    NNO_PrepWriteCmd(&msg, NNO_CMD_START_SNRSCAN );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

int NNO_StartHadSNRScan(void)
/**
 * Initiates the special scan sequence for Hadamard SNR computation.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x2F)
 *
 * @ingroup calAPI
 *
 * @return PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;

    int retval;

    NNO_PrepWriteCmd(&msg, NNO_CMD_START_HADSNRSCAN );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

int NNO_GetSNRData(int* pVal1 , int* pVal2 , int* pVal3)
/**
 * Reads SNR Value at different time intervals namely 17ms , 133ms and 600ms.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x2C)
 *
 * @ingroup calAPI
 *
 * @param   pVal1  - O - SNR value calculated at 17ms.
 * @param   pVal2  - O - SNR value calculated at 133ms.
 * @param   pVal3  - O - SNR value calculated at 600ms.
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct *pMsg = (nnoMessageStruct *)InputBuffer;

	NNO_PrepReadCmd(NNO_CMD_SAVE_SNRDATA);

	int retval = NNO_Read();
	if(retval > 0)
	{
        pMsg = (nnoMessageStruct *)InputBuffer;
        *pVal1 = pMsg->payload.data[0] | pMsg->payload.data[1] << 8 | pMsg->payload.data[2] << 16 | pMsg->payload.data[3] << 24;
        *pVal2 = pMsg->payload.data[4] | pMsg->payload.data[5] << 8 | pMsg->payload.data[6] << 16 | pMsg->payload.data[7] << 24;
        *pVal3 = pMsg->payload.data[8] | pMsg->payload.data[9] << 8 | pMsg->payload.data[10] << 16 | pMsg->payload.data[11] << 24;
		return PASS;
	}
	return FAIL;
}

int NNO_SetSerialNumber(char* serial_number)
/**
 * Programs the given serial number (8 characters) to EEPROM of the EVM.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x32)
 *
 * @ingroup calAPI
 *
 * @param serial_number - I - Pointer to the serial number string.
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;

    int retval;

     for(int i=0;i<NANO_SER_NUM_LEN;i++)
    {
        msg.payload.data[i+2] = serial_number[i];
    }
    NNO_PrepWriteCmd(&msg, NNO_CMD_SERIAL_NUMBER_WRITE );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;

}

int NNO_GetSerialNumber(char* serial_number)
/**
 * Reads the EVM's serial number (8 characters) and returns.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x33)
 *
 * @ingroup calAPI
 *
 * @param serial_number - O - Pointer to the serial number string.
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct *pMsg;
    int ret_val;

    NNO_PrepReadCmd(NNO_CMD_SERIAL_NUMBER_READ);

    if((ret_val = NNO_Read()) > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        for(int i=0;i<NANO_SER_NUM_LEN;i++)
        {
            serial_number[i] = pMsg->payload.data[i];
        }
        return PASS;
    }
    return ret_val;

}

int NNO_SetModelName(char* model_name)
/**
 * Programs the given model name (16 characters) to EEPROM of the EVM.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x3B)
 *
 * @ingroup calAPI
 *
 * @param model_name - I - Pointer to the model name string.
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;

    int retval;

     for(int i=0;i<NANO_MODEL_NAME_LEN;i++)
    {
        msg.payload.data[i+2] = model_name[i];
    }
    NNO_PrepWriteCmd(&msg, NNO_CMD_MODEL_NAME_WRITE );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;

}

int NNO_GetModelName(char* model_name)
/**
 * Reads the EVM's model name (16 characters) and returns.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x3C)
 *
 * @ingroup calAPI
 *
 * @param model_name - O - Pointer to the model Name string.
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct *pMsg;
    int ret_val;

    NNO_PrepReadCmd(NNO_CMD_MODEL_NAME_READ);

    if((ret_val = NNO_Read()) > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        for(int i=0;i<NANO_MODEL_NAME_LEN;i++)
        {
            model_name[i] = pMsg->payload.data[i];
        }
        return PASS;
    }
    return ret_val;

}


int NNO_SendEEPROMWipe(bool wipe_cal_coeffs, bool wipe_refcal_data, bool wipe_cfg_data)
/**
 * Commands Tiva to wipe selected data from EEPROM.
 * USE WITH CAUTION.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x27)
 *
 * @ingroup calAPI
 *
 * @param wipe_cal_coeffs - I - Erases calibration coefficients from EEPROM
 * @param wipe_refcal_data - I -  Erases reference calibration data from EEPROM.
 * @param wipe_cfg_data - I - Erases scan config data from EEPROM.
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;

    int retval;

    if(wipe_cal_coeffs)
        msg.payload.data[2] = 1;
    else
        msg.payload.data[2] = 0;

    if(wipe_refcal_data)
        msg.payload.data[3] = 1;
    else
        msg.payload.data[3] = 0;

    if(wipe_cfg_data)
        msg.payload.data[4] = 1;
    else
        msg.payload.data[4] = 0;

    NNO_PrepWriteCmd(&msg, NNO_CMD_EEPROM_WIPE );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

int NNO_EEPROMMass_Erase()
/**
 * Commands Tiva to erase entire EEPROM.
 * USE WITH CAUTION.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x36)
 *
 * @ingroup calAPI
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;

    int retval;

    NNO_PrepWriteCmd(&msg, NNO_CMD_EEPROM_MASS_ERASE );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

int NNO_GetCalibStruct(calibCoeffs  *pCalibResult)
/**
 * Returns the calibration coefficients stored in EEPROM of the EVM.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x2A)
 *
 * @ingroup calAPI
 *
 * @param pCalibResult - I - Pointer where the coefficients are to be returned.
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct *msg = (nnoMessageStruct *)InputBuffer;
    int ret_val;

     NNO_PrepReadCmd(NNO_CMD_CALIB_STRUCT_READ);

    if((ret_val = NNO_Read()) > 0)
    {
        dlpspec_calib_read_data(&msg->payload.data[0], sizeof(calibCoeffs)*3);
        memcpy(pCalibResult, &(msg->payload.data[0]), sizeof(calibCoeffs));
        return PASS;

    }
    return ret_val;
}

/*******************************************************************************/
/*								Utility APIs								   */
/*******************************************************************************/

int NNO_ResetTiva(void)
/**
 * Commands Tiva to reset itself.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x1A)
 *
 * @ingroup utilAPI
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;

    int retval;

    NNO_PrepWriteCmd(&msg, NNO_CMD_TIVA_RESET );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
        return PASS;

    return FAIL;
}

int NNO_HibernateMode(void)
/**
 * Puts Tiva in hibernation mode.
 *
 * (USB CMD OPCODE: BYTE1: 0x03, BYTE2: 0x0D)
 *
 * @ingroup utilAPI
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;

    int retval;

    NNO_PrepWriteCmd(&msg, NNO_CMD_HIBERNATE_MODE );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
        return PASS;

    return FAIL;

}

int NNO_SetHibernate(bool newValue)
/**
 * Sets whether NIRscan Nano will enter Hibernation after timeout or not.
 *
 * (USB CMD OPCODE: BYTE1: 0x03, BYTE2: 0x0E)
 *
 * @ingroup utilAPI
 *
 * @param newValue  True sets NIRscan Nano so that it will hibernate.
 *                  False sets NIRscan Nano so that it will not hibernate.
 *
 * @return PASS or FAIL
 */
{
    nnoMessageStruct msg;

    int retval;

    msg.payload.data[2] = newValue;
    NNO_PrepWriteCmd(&msg, NNO_CMD_SET_HIBERNATE );

    retval = NNO_SendMsg(&msg);
    if(retval > 0)
    {
        return NNO_GetAck(0);
    }
    return FAIL;
}

int NNO_GetHibernate(void)
/**
 * Returns hibernation flag from NIRscan Nano, which determines whether it
 * will enter hibernation mode or not.
 *
 * (USB CMD OPCODE: BYTE1: 0x03, BYTE2: 0x0F)
 *
 * @ingroup utilAPI
 *
 * @return 1    hibernation flag active (Nano will hibernate after timeout)
 * @return 0    hibernation flag inactive (Nano will not hibernate)
 * @return <0   FAIL
 *
 */
{
    nnoMessageStruct *pMsg;
    char ret_val;

    NNO_PrepReadCmd(NNO_CMD_GET_HIBERNATE);
    if((ret_val = NNO_Read()) > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        ret_val = pMsg->payload.data[0];
        return (int)ret_val;
    }
    return ret_val;
}
int NNO_SetFixedPGAGain(bool isFixed, uint8 gainVal)
/**
 * Sets Fixed ADC PGA Gain
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x1B)
 *
 * @ingroup utilAPI
 *
 * @param isFixed - I - if we want to Fix the PGA durng scan
 * @param gainVal - I - Sets ADC PGA gain. Valid values are 1,2,4,8,16,32 or 64.
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;
    int retval;

    msg.payload.data[2] = isFixed;
    msg.payload.data[3] = gainVal;
    NNO_PrepWriteCmd(&msg, NNO_CMD_SET_FIXED_PGA);

    retval = NNO_SendMsg(&msg);
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}
int NNO_SetPGAGain(uint8 gainVal)
/**
 * Sets ADC PGA Gain
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x1B)
 *
 * @ingroup utilAPI
 *
 * @param gainVal - I - Sets ADC PGA gain. Valid values are 1,2,4,8,16,32 or 64.
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;

    int retval;

    msg.payload.data[2] = gainVal;
    NNO_PrepWriteCmd(&msg, NNO_CMD_SET_PGA );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

int NNO_GetPGAGain(void)
/**
 * Returns ADC PGA Gain setting
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x28)
 *
 * @ingroup utilAPI
 *
 * @return ADC PGA gain
 *         <0 = FAIL
 *
 */
{
    nnoMessageStruct *pMsg;
    char ret_val;

    NNO_PrepReadCmd(NNO_CMD_GET_PGA);
    if((ret_val = NNO_Read()) > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        ret_val = pMsg->payload.data[0];
        return (int)ret_val;
    }
    return ret_val;
}

int NNO_GetDateTime( uint8 *year, uint8 *month, uint8 *day, uint8 *wday, uint8 *hour, uint8 *min, uint8 *sec )
/**
 * Get the time stamp information from EVM.
 *
 * (USB CMD OPCODE: BYTE1: 0x03, BYTE2: 0x3C)
 *
 * @ingroup utilAPI
 *
 * @param year - I - year
 * @param month - I - month
 * @param day - I - date
 * @param wday - I - day of the week
 * @param hour - I - hour
 * @param min - I - minute
 * @param sec - I - seconds
 *
 * @return PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;
    int ret_val;

    NNO_PrepReadCmd( NNO_CMD_GET_DATE_TIME );
    if((ret_val = NNO_Read()) > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        *year = msg.payload.data[0];
        *month = msg.payload.data[1];
        *day = msg.payload.data[2];
        *wday = msg.payload.data[3];
        *hour = msg.payload.data[4];
        *min = msg.payload.data[5];
        *sec = msg.payload.data[6];
      return PASS;
    }
    return ret_val;
}

int NNO_SetDateTime( uint8 year, uint8 month, uint8 day, uint8 wday, uint8 hour, uint8 min, uint8 sec )
/**
 * Set the date/time information on EVM.
 *
 * (USB CMD OPCODE: BYTE1: 0x03, BYTE2: 0x09)
 *
 * @ingroup utilAPI
 *
 * @param year - I - year
 * @param month - I - month
 * @param day - I - date
 * @param wday - I - day of the week
 * @param hour - I - hour
 * @param min - I - minute
 * @param sec - I - seconds
 *
 * @return PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;

    int retval;

    memcpy(&msg.payload.data[2], &year, 1);
    memcpy(&msg.payload.data[3], &month, 1);
    memcpy(&msg.payload.data[4], &day, 1);
    memcpy(&msg.payload.data[5], &wday, 1);
    memcpy(&msg.payload.data[6], &hour, 1);
    memcpy(&msg.payload.data[7], &min, 1);
    memcpy(&msg.payload.data[8], &sec, 1);
    NNO_PrepWriteCmd( &msg, NNO_CMD_SET_DATE_TIME  );
    
    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

/*******************************************************************************/
/*				APIs for reading various sensors on the EVM					   */
/*******************************************************************************/

int NNO_ReadTemp(int *Ambient, int *Detector)
/**
 * Reads ambient temperature and detector temperature from the EVM.
 *
 * (USB CMD OPCODE: BYTE1: 0x03, BYTE2: 0x00)
 *
 * @ingroup sensorAPI
 *
 * @param   Ambient - O - Ambient temperature in units of a hundredths.
 * @param   Detector - O - Detector temperature in units of a hundredths.
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct *pMsg;
    int ret_val;

    NNO_PrepReadCmd(NNO_CMD_READ_TEMP);
    if((ret_val = NNO_Read()) > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        *Ambient = (pMsg->payload.data[0] | pMsg->payload.data[1] << 8 | pMsg->payload.data[2] << 16 | pMsg->payload.data[3] << 24);
        *Detector  = (pMsg->payload.data[4] | pMsg->payload.data[5] << 8 | pMsg->payload.data[6] << 16 | pMsg->payload.data[7] << 24);
       return PASS;
    }
    return ret_val;
}


int NNO_ReadHum(uint32 *Humidity, int *HDC_Temp)
/**
 * Reads humidity and HDC temperature from the EVM.
 *
 * (USB CMD OPCODE: BYTE1: 0x03, BYTE2: 0x02)
 *
 * @ingroup sensorAPI
 *
 * @param   Humidity - O - Humidity in units of a hundredths.
 * @param   HDC_Temp - O - HDC temperature in units of a hundredths.
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct *pMsg;
    int ret_val;

    NNO_PrepReadCmd(NNO_CMD_READ_HUM);
    if((ret_val = NNO_Read()) > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        *HDC_Temp = (pMsg->payload.data[0] | pMsg->payload.data[1] << 8 | pMsg->payload.data[2] << 16 | pMsg->payload.data[3] << 24) ;
        *Humidity = (pMsg->payload.data[4] | pMsg->payload.data[5] << 8 | pMsg->payload.data[6] << 16 | pMsg->payload.data[7] << 24);
        return PASS;
    }
    return ret_val;
}


int NNO_ReadBattVolt(uint32 *Batt_volt)
/**
 * Reads battery voltage from the EVM.
 *
 * (USB CMD OPCODE: BYTE1: 0x03, BYTE2: 0x0A)
 *
 * @ingroup sensorAPI
 *
 * @param   Batt_volt - O - Battery voltage in units of a hundredths.
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct *pMsg;
    int ret_val;

    NNO_PrepReadCmd(NNO_CMD_READ_BATT_VOLT);
    if((ret_val = NNO_Read()) > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        *Batt_volt = (pMsg->payload.data[0] | pMsg->payload.data[1] << 8 | pMsg->payload.data[2] << 16 | pMsg->payload.data[3] << 24);
        return PASS;
    }
    return ret_val;
}


int NNO_ReadTivaTemp(int *TivaTemp)
/**
 * Reads Tiva temperature from the EVM.
 *
 * (USB CMD OPCODE: BYTE1: 0x03, BYTE2: 0x3B)
 *
 * @ingroup sensorAPI
 *
 * @param   TivaTemp - O - Tiva temperature in units of a hundredths.
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct *pMsg;
    int ret_val;

    NNO_PrepReadCmd(NNO_CMD_READ_TIVA_TEMP);
    if((ret_val = NNO_Read()) > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        *TivaTemp = (pMsg->payload.data[0] | pMsg->payload.data[1] << 8 | pMsg->payload.data[2] << 16 |pMsg->payload.data[3] << 24);
        return PASS;
    }
    return ret_val;
}

int NNO_GetPhotoDetector(uint32* red, uint32* green , uint32* blue)
/**
 * Reads Lamp Photodetector value.
 *
 * (USB CMD OPCODE: BYTE1: 0x04, BYTE2: 0x02)
 *
 * @ingroup sensorAPI
 *
 * @param   red   - O - Always 0.Don't care.
 * @param   green - O - Lamp photodetector value in range 0-4095.
 * @param   blue  - O - Always 0.Don't care.
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct *pMsg = (nnoMessageStruct *)InputBuffer;

    NNO_PrepReadCmd(NNO_CMD_READ_PHOTODETECTOR);

    int retval = NNO_Read();
    if(retval > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        *red = pMsg->payload.data[0] | pMsg->payload.data[1] << 8 | pMsg->payload.data[2] << 16 | pMsg->payload.data[3] << 24;
        *green = pMsg->payload.data[4] | pMsg->payload.data[5] << 8 | pMsg->payload.data[6] << 16 | pMsg->payload.data[7] << 24;
        *blue = pMsg->payload.data[8] | pMsg->payload.data[9] << 8 | pMsg->payload.data[10] << 16 | pMsg->payload.data[11] << 24;
        return PASS;
    }
    return FAIL;
}
/*******************************************************************************/
/*				APIs for testing various components on the EVM				   */
/*******************************************************************************/

int NNO_TestEEPROM(void)
/**
 * Sends command to perform EEPROM test.
 *
 * (USB CMD OPCODE: BYTE1: 0x01, BYTE2: 0x01)
 *
 * @ingroup testAPI
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;
    int ret_val;

    NNO_PrepReadCmd(NNO_CMD_EEPROM_TEST);
    if((ret_val = NNO_Read()) > 0)
    {
        memcpy(&msg, InputBuffer, 65);
        if(msg.payload.data[0] == 0)
            return PASS;
        else
            return FAIL;
    }
    return ret_val;
}

int NNO_TestADC(void)
/**
 * Sends command to perform ADC test.
 *
 * (USB CMD OPCODE: BYTE1: 0x01, BYTE2: 0x02)
 *
 * @ingroup testAPI
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct *pMsg;
    int numBytesRead;

    NNO_PrepReadCmd(NNO_CMD_ADC_TEST);

    numBytesRead = NNO_Read();
    if(numBytesRead > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        if(pMsg->payload.data[0] == PASS)
            return PASS;
        else
            return FAIL;
    }
    return numBytesRead;
}

int NNO_TestTMP(void)
/**
 * Sends command to perform temperature sensor test.
 *
 * (USB CMD OPCODE: BYTE1: 0x01, BYTE2: 0x06)
 *
 * @ingroup testAPI
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct *pMsg;
    int numBytesRead;

    NNO_PrepReadCmd(NNO_CMD_TMP_TEST);

    numBytesRead = NNO_Read();
    if(numBytesRead > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        if(pMsg->payload.data[0] == PASS)
            return PASS;
        else
            return FAIL;
    }
    return numBytesRead;
}


int NNO_TestHDC(void)
/**
 * Sends command to perform HDC test and reports results.
 *
 * (USB CMD OPCODE: BYTE1: 0x01, BYTE2: 0x07)
 *
 * @ingroup testAPI
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct *pMsg;
    int numBytesRead;

    NNO_PrepReadCmd(NNO_CMD_HDC_TEST);

    numBytesRead = NNO_Read();
    if(numBytesRead > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        if(pMsg->payload.data[0] == PASS)
            return PASS;
        else
            return FAIL;
    }
    return numBytesRead;
}

int NNO_TestBT(bool enable)
/**
 * Performs bluetooth enable/disable for the purpose of testing
 *
 * (USB CMD OPCODE: BYTE1: 0x01, BYTE2: 0x08)
 *
 * @ingroup testAPI
 *
 * @param enable - I - enables bluetooth when true; disables when false.
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;

    int retval;

    msg.payload.data[2] = enable;

    NNO_PrepWriteCmd(&msg, NNO_CMD_BT_TEST );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

int NNO_TestSDC(void)
/**
 * Sends command to perform SD Card test and reports results.
 *
 * (USB CMD OPCODE: BYTE1: 0x01, BYTE2: 0x09)
 *
 * @ingroup testAPI
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct *pMsg;
    int numBytesRead;

    NNO_PrepReadCmd(NNO_CMD_SDC_TEST);

    numBytesRead = NNO_Read();
    if(numBytesRead > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        if(pMsg->payload.data[0] == PASS)
            return PASS;
        else
            return FAIL;
    }
    return numBytesRead;
}

int NNO_TestBQ(int *pStatus, int *pBatt_volt, int *pUsb_det, int *pCharge_curr, int *PTS_fault)
/**
 * Sends command to perform battery charger test and reports results.
 *
 * (USB CMD OPCODE: BYTE1: 0x01, BYTE2: 0x03)
 *
 * @ingroup testAPI
 *
 * @param pStatus - O - Battery status
 * @param pBatt_volt - O - Battery voltage.
 * @param pUsb_det - O - Battery charging current and voltage thresholds
 * @param pCharge_curr - O - Charge current
 * @param PTS_fault - O - Thermistor fault/status
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct *pMsg;
    int ret_val;

    NNO_PrepReadCmd(NNO_CMD_BQ_TEST);
    if((ret_val = NNO_Read()) > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        *pStatus = pMsg->payload.data[0] | pMsg->payload.data[1] << 8 | pMsg->payload.data[2] << 16 | pMsg->payload.data[3] << 24;
        *pBatt_volt = pMsg->payload.data[4] | pMsg->payload.data[5] << 8 | pMsg->payload.data[6] << 16 | pMsg->payload.data[7] << 24;
        *pUsb_det = pMsg->payload.data[8] | pMsg->payload.data[9] << 8 | pMsg->payload.data[10] << 16 | pMsg->payload.data[11] << 24;
        *pCharge_curr = pMsg->payload.data[12] | pMsg->payload.data[13] << 8 | pMsg->payload.data[14] << 16 | pMsg->payload.data[15] << 24;
        *PTS_fault = pMsg->payload.data[16] | pMsg->payload.data[17] << 8 | pMsg->payload.data[18] << 16 | pMsg->payload.data[19] << 24;

        return PASS;
    }
    return ret_val;
}

int NNO_TestSDRAM(void)
/**
 * Sends command to perform SDRAM test and reports results.
 *
 * (USB CMD OPCODE: BYTE1: 0x01, BYTE2: 0x04)
 *
 * @ingroup testAPI
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct *pMsg;
    char ret_val;

    NNO_PrepReadCmd(NNO_CMD_SDRAM_TEST);
    if((ret_val = NNO_Read()) > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        ret_val = pMsg->payload.data[0];
        return (int)ret_val;
    }
    return ret_val;
}

int NNO_TestLED(bool enable)
/**
 * Function used to test the LEDs on EVM.
 *
 * (USB CMD OPCODE: BYTE1: 0x01, BYTE2: 0x0B)
 *
 * @ingroup testAPI
 *
 * @param enable - I - true = turns ON green, blue and yellow LED on EVM; false turns OFF those LEDs
 *
 * @return PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;

    int retval;

    msg.payload.data[2] = enable;

   /* if(Serial_Open("COM6") == -1)
    {
        if(Serial_Open("COM2") == -1)
           {
            int ret = Serial_Open("COM3");
        }
    }*/

    NNO_PrepWriteCmd(&msg, NNO_CMD_LED_TEST );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

int NNO_ButtonTestWr( bool enable )
/**
 * Enables button test mode. When in that mode the number of presses on scan button and ON/OFF button are counted
 * and returned when NNO_ButtonTestRd() is called.
 *
 * (USB CMD OPCODE: BYTE1: 0x01, BYTE2: 0x0D)
 *
 * @ingroup testAPI
 *
 * @param enable - I - turn on button test mode when true; turn off button test mode when false.
 *
 * @return PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;

    int retval;

    msg.payload.data[2] = enable;

    NNO_PrepWriteCmd(&msg, NNO_CMD_BUTTON_TEST_WR );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

int NNO_ButtonTestRd( uint8 *button )
/**
 * Returns the number of presses on scan button and ON/OFF button that were detected while in button test mode.
 *
 * (USB CMD OPCODE: BYTE1: 0x01, BYTE2: 0x0C)
 *
 * @ingroup testAPI
 *
 * @param button - O - BIT1 if set means scan button press was detected while in button test mode
 *                     BIT2 if set means on/off button press was detected while in button test mode
 *
 * @return PASS or FAIL
 *
 */
{
    nnoMessageStruct *pMsg;
    int ret_val;

    NNO_PrepReadCmd(NNO_CMD_BUTTON_TEST_RD);

    if((ret_val = NNO_Read()) > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        *button = pMsg->payload.data[0];
        return PASS;
    }
    return ret_val;
}

int NNO_EEPROM_CalTest(void)
/**
 * Debug only function. Overwrites the calibration coefficients and cal data versions with some preset values.
 * USE WITH CAUTION.
 *
 * (USB CMD OPCODE: BYTE1: 0x01, BYTE2: 0x0E)
 *
 * @ingroup testAPI
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;

    int retval;

    NNO_PrepWriteCmd(&msg, NNO_CMD_EEPROM_CAL_TEST );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

int NNO_EraseDLPC150Flash(void)
{
    nnoMessageStruct msg;

    int retval;

    NNO_PrepWriteCmd(&msg, NNO_CMD_ERASE_DLPC_FLASH );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

/*******************************************************************************/
/*					APIs to be used as debug tools							   */
/*******************************************************************************/
int NNO_SetScanNameTag(const char* tag, int len)
/**
 *
 *
 * @param tag - I - Scan name tag to be written
 * @param len - I - Length of tag in bytes
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;

    int retval;
    int i;

    for(i = 0; i < len; i++)
    {
        msg.payload.data[i+2] = tag[i];
    }
    msg.payload.data[i+2] = '\0'; //to null terminate the string
    NNO_PrepWriteCmd(&msg, NNO_CMD_WRITE_SCAN_NAME_TAG );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;

}
int NNO_SendCalibStruct(calibCoeffs  *pCalibResult)
/**
 * Debug only function. Overwrites the calibration result with the one passed to this function.
 * USE WITH CAUTION.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x29)
 *
 * @ingroup debugAPI
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;

    int retval;
    size_t bufferSize = sizeof(calibCoeffs)*3;
    void *pBuffer = malloc(bufferSize);

    if(pBuffer ==NULL)
    {
        return FAIL;
    }
    dlpspec_calib_write_data(pCalibResult, pBuffer, bufferSize);

    memcpy(&msg.payload.data[2], pBuffer, bufferSize);
    free(pBuffer);

    NNO_PrepWriteCmd(&msg, NNO_CMD_CALIB_STRUCT_SAVE );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;

}

int NNO_SetDLPCReg(uint32 addr, uint32 val)
/**
 * Debug only function. Sets DLPC150 registers.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x1C)
 *
 * @ingroup debugAPI
 *
 * @param addr - I - Register address
 * @param val - I - Register value
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct msg;

    int retval;

    msg.payload.data[2] = addr;
    msg.payload.data[3] = addr >> 8;
    msg.payload.data[4] = addr >> 16;
    msg.payload.data[5] = addr >> 24;
    msg.payload.data[6] = val;
    msg.payload.data[7] = val >> 8;
    msg.payload.data[8] = val >> 16;
    msg.payload.data[9] = val >> 24;
    NNO_PrepWriteCmd(&msg, NNO_CMD_SET_DLPC_REG );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;

}

int NNO_GetDLPCReg(uint32 addr, uint32 *pVal)
/**
 * Debug only function. Reads DLPC150 registers.
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x1D)
 *
 * @ingroup debugAPI
 *
 * @param addr - I - Register address
 * @param pVal - I - Pointer where Register value will be returned
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct *pMsg;
    int ret_val;

    NNO_PrepReadCmdWithParam(NNO_CMD_GET_DLPC_REG, addr);
    if((ret_val = NNO_Read()) > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        *pVal = pMsg->payload.data[0] | pMsg->payload.data[1] << 8 | pMsg->payload.data[2] << 16 | pMsg->payload.data[3] << 24;
        return PASS;
    }
    return ret_val;
}


int NNO_ReadDeviceStatus(uint32 *pVal)
/**
 *
 *
 * @param pVal - O - the Device status to be read
 *
 * @return  PASS or FAIL
 *
 */

{
    nnoMessageStruct *pMsg;
    int ret_val;

    NNO_PrepReadCmd(NNO_CMD_READ_DEVICE_STATUS);
    if((ret_val = NNO_Read()) > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;

        *pVal = pMsg->payload.data[0] | pMsg->payload.data[1] << 8 | pMsg->payload.data[2] << 16 | pMsg->payload.data[3] << 24;
        // ret_val = pMsg->payload.data[2];
        return PASS;
    }
    return ret_val;

}
int NNO_ReadErrorStatus(NNO_error_status_struct* error_status)
/**
 *
 *
 * @param error_status - O - the Error status to be read
 *
 * @return  PASS or FAIL
 *
 */
{
    nnoMessageStruct *pMsg;
    int ret_val;

    NNO_PrepReadCmd(NNO_CMD_READ_ERROR_STATUS);
    if((ret_val = NNO_Read()) > 0)
    {
        pMsg = (nnoMessageStruct *)InputBuffer;
        memcpy(error_status, &pMsg->payload.data[0],sizeof(NNO_error_status_struct));
        return PASS;
    }
    return ret_val;

}

int NNO_ResetErrorStatus()
/**
 *
 * Clears the error status bits
 *
 * @return  PASS or FAIL
 *
 */
{

    nnoMessageStruct msg;

    int retval;

    NNO_PrepWriteCmd(&msg, NNO_CMD_RESET_ERROR_STATUS );

    retval = NNO_SendMsg(&msg );
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

int NNO_UpdateRefCalDataWithWORefl(void)
/**
 * UPdates the curren Reference cal data by dividing with the whiteout Reflectance
 *
 * (USB CMD OPCODE: BYTE1: 0x02, BYTE2: 0x38)
 *
 * @ingroup sdAPI
 *
 * @return PASS or FAIL
 *
 */
{

    nnoMessageStruct msg;

    int retval;

    NNO_PrepWriteCmd(&msg, NNO_CMD_UPDATE_REFCALDATA_WOREFL );

    retval = NNO_SendMsg(&msg);
    if(retval > 0)
    {
        return NNO_GetAck();
    }
    return FAIL;
}

int NNO_SetUARTConnected(bool connected)
{
    g_UARTConnected = connected;
    return PASS;
}
