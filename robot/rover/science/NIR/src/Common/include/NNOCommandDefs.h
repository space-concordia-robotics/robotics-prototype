/*
 *
 * Command IDs user in NIRscan Nano
 *
 * Copyright (C) 2014-2015 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
*/

#ifndef NNO_COMMANDEFS_H
#define NNO_COMMANDEFS_H

#define NNO_DATA_MAX_SIZE    512
#define NNO_UART_MAX_DATA_SIZE 220
#define SCAN_NAME_TAG_SIZE	14

#define MIN_WAVELENGTH 900
#define MAX_WAVELENGTH 1700

// Max storage space allocated in EEPROM for scan configs
#define EEPROM_MAX_SCAN_CFG_STORAGE 20

#define NNO_RESP_SUCCESS 0
#define NNO_RESP_ERROR	 1
#define NNO_RESP_BUSY    2

#define NANO_MODEL_NAME_LEN   16

typedef enum packettype
{
    REQUEST,
    RESPONSE
}
NNO_PACKET_TYPE;

typedef struct _nnoMessageStruct
{
    struct _nnoHead
    {
        struct _packetcontrolStruct
        {
            unsigned char dest		:2; /* 0 - Command, 2 - Debug logs */
            unsigned char reserved	:2; /* Future use */
            unsigned char resp		:2; /* cmd Handler Response; 0 - Success, 1 - Error, 2 - Busy */
            unsigned char reply		:1; /* Host wants a reply from device */
            unsigned char rw		:1; /* Write = 0; Read = 1 */
        } flags;
        unsigned char seq;
        unsigned short length;
    } head;

    union
    {
        unsigned short cmd;
        unsigned char data[NNO_DATA_MAX_SIZE];
    } payload;
} nnoMessageStruct;
typedef struct _uartMsgStruct
{
    unsigned char startInd[4];
    unsigned int chkSum;
    struct _nnouartHead
    {
        struct _uartpacketcontrolStruct
        {
            unsigned char dest		:2; /* 0 - Command, 2 - Debug logs */
            unsigned char reserved	:2; /* Future use */
            unsigned char resp		:2; /* cmd Handler Response; 0 - Success, 1 - Error, 2 - Busy */
            unsigned char reply		:1; /* Host wants a reply from device */
            unsigned char rw		:1; /* Write = 0; Read = 1 */
        } flags;
        unsigned char seq;
        unsigned short length;
    } head;

    union
    {
        unsigned short cmd;
        unsigned char data[512];
    } payload;
    unsigned char endInd[4];
} nnouartMessageStruct;

typedef enum
{
    NNO_FILE_SCAN_DATA,
    NNO_FILE_SCAN_CONFIG,
    NNO_FILE_REF_CAL_DATA,
    NNO_FILE_REF_CAL_MATRIX,
	NNO_FILE_HADSNR_DATA,
    NNO_FILE_SCAN_CONFIG_LIST,
    NNO_FILE_SCAN_LIST,
    NNO_FILE_SCAN_DATA_FROM_SD,
    NNO_FILE_INTERPRET_DATA,
    NNO_FILE_MAX_TYPES
} NNO_FILE_TYPE;

/**
 *  Enumeration of actions to perform on file payload to be sent to the EVM
 */
typedef enum
{
    NNO_FILE_DLPC_UPDATE,
    NNO_FILE_PTN_LOAD_SDRAM,
    NNO_FILE_REFCAL_DATA
} NNO_FILE_ACTION;


typedef enum Cmd1Msg
{
	CMD1_BUSY,
    CMD1_ACK,
    CMD1_NACK,
	CMD1_WRITE,
    CMD1_WRITE_RESPONSE,
    CMD1_READ,
    CMD1_READ_RESPONSE
} CMD1_TYPE;

#ifdef PART_TM4C129XNCZAD
#define CMD_KEY(cmd2, cmd3, cmd1, len) (((cmd2) << 16 ) | ( (cmd3) << 8 ) | (cmd1) )
#else	// GUI
#define CMD_KEY(cmd2, cmd3, cmd1, len) (((cmd2) << 16 ) | ( (cmd3) << 8 ) | (len) )
#define CMD_GET_CMD(key) (key >> 8)
#define CMD_GET_LEN(key) (key & 0xFF)
#endif

#ifndef PART_TM4C129XNCZAD
#define SET_SCAN_CONFIG_SIZE	sizeof(scanConfig)*2
#define WRITE_SCAN_CONFIG_SIZE 		sizeof(scanConfig)*2+2
#define CALIB_COEFFS_SIZE		sizeof(calibCoeffs)*3
#else
#define SET_SCAN_CONFIG_SIZE	0
#define WRITE_SCAN_CONFIG_SIZE 	0
#define CALIB_COEFFS_SIZE		0
#endif
 
#define NNO_CMD_FLASH_GET_CHKSUM        CMD_KEY(0x00, 0x15, CMD1_READ, 	0x01)
#define NNO_CMD_FILE_WRITE_DATA         CMD_KEY(0x00 ,0x25, CMD1_WRITE, 0x00)
#define NNO_CMD_FILE_SET_WRITESIZE      CMD_KEY(0x00 ,0x2A, CMD1_WRITE, 0x06)
#define NNO_CMD_READ_FILE_LIST_SIZE     CMD_KEY(0x00, 0x2B, CMD1_READ, 	0x00)
#define NNO_CMD_READ_FILE_LIST          CMD_KEY(0x00, 0x2C, CMD1_READ, 	0x00)
#define NNO_CMD_FILE_GET_READSIZE       CMD_KEY(0x00, 0x2D, CMD1_READ,	0x01)
#define NNO_CMD_FILE_GET_DATA           CMD_KEY(0x00, 0x2E, CMD1_READ,	0x00)
#define NNO_CMD_GOTO_TIVA_BL            CMD_KEY(0x00 ,0x2F, CMD1_WRITE,	0x00)
#define NNO_CMD_EEPROM_TEST             CMD_KEY(0x01 ,0x01, CMD1_READ,	0x00)
#define NNO_CMD_ADC_TEST                CMD_KEY(0x01 ,0x02, CMD1_READ,	0x00)
#define NNO_CMD_BQ_TEST                 CMD_KEY(0x01 ,0x03, CMD1_READ,	0x00)
#define NNO_CMD_SDRAM_TEST              CMD_KEY(0x01 ,0x04, CMD1_READ,	0x00)
#define NNO_CMD_DLPC_ENABLE             CMD_KEY(0x01 ,0x05, CMD1_WRITE,	0x02)
#define NNO_CMD_TMP_TEST                CMD_KEY(0x01 ,0x06, CMD1_READ,	0x00)
#define NNO_CMD_HDC_TEST                CMD_KEY(0x01 ,0x07, CMD1_READ,	0x00)
#define NNO_CMD_BT_TEST                 CMD_KEY(0x01 ,0x08, CMD1_WRITE,	0x01)
#define NNO_CMD_SDC_TEST                CMD_KEY(0x01 ,0x09, CMD1_READ,	0x01)
#define NNO_CMD_SENSOR_READ             CMD_KEY(0x01 ,0x0A, CMD1_READ,	0x01)
#define NNO_CMD_LED_TEST                CMD_KEY(0x01 ,0x0B, CMD1_WRITE,	0x01)
#define NNO_CMD_BUTTON_TEST_RD          CMD_KEY(0x01 ,0x0C, CMD1_READ,	0x01)
#define NNO_CMD_BUTTON_TEST_WR          CMD_KEY(0x01 ,0x0D, CMD1_WRITE,	0x01)
#define NNO_CMD_EEPROM_CAL_TEST         CMD_KEY(0x01 ,0x0E, CMD1_WRITE,	0x00)
#define NNO_CMD_TIVA_VER                CMD_KEY(0x02 ,0x16, CMD1_READ,	0x00)
#define NNO_CMD_STORE_PTN_SDRAM         CMD_KEY(0x02 ,0x17, CMD1_WRITE,	0x00)
#define NNO_CMD_PERFORM_SCAN            CMD_KEY(0x02, 0x18, CMD1_WRITE,	0x01)
#define NNO_CMD_SCAN_GET_STATUS         CMD_KEY(0x02 ,0x19, CMD1_READ,	0x01)
#define NNO_CMD_TIVA_RESET              CMD_KEY(0x02 ,0x1A, CMD1_WRITE,	0x00)
#define NNO_CMD_SET_PGA                 CMD_KEY(0x02 ,0x1B, CMD1_WRITE,	0x01)
#define NNO_CMD_SET_DLPC_REG            CMD_KEY(0x02 ,0x1C, CMD1_WRITE,	0x08)
#define NNO_CMD_GET_DLPC_REG            CMD_KEY(0x02 ,0x1D, CMD1_READ,	0x04)
#define NNO_CMD_SCAN_CFG_APPLY          CMD_KEY(0x02 ,0x1E, CMD1_WRITE,	SET_SCAN_CONFIG_SIZE)
#define NNO_CMD_SCAN_CFG_SAVE           CMD_KEY(0x02 ,0x1F, CMD1_WRITE,	WRITE_SCAN_CONFIG_SIZE)
#define NNO_CMD_SCAN_CFG_READ           CMD_KEY(0x02, 0x20, CMD1_READ,	0x01)
#define NNO_CMD_SCAN_CFG_ERASEALL       CMD_KEY(0x02 ,0x21, CMD1_WRITE,	0x00)
#define NNO_CMD_SCAN_CFG_NUM            CMD_KEY(0x02, 0x22, CMD1_READ,	0x00)
#define NNO_CMD_SCAN_GET_ACT_CFG        CMD_KEY(0x02, 0x23, CMD1_READ,	0x00)
#define NNO_CMD_SCAN_SET_ACT_CFG        CMD_KEY(0x02, 0x24, CMD1_WRITE,	0x01)
#define NNO_CMD_SET_DLPC_ONOFF_CTRL     CMD_KEY(0x02 ,0x25, CMD1_WRITE,	0x01)
#define NNO_CMD_SET_SCAN_SUBIMAGE       CMD_KEY(0x02 ,0x26, CMD1_WRITE,	0x04)
#define NNO_CMD_EEPROM_WIPE             CMD_KEY(0x02 ,0x27, CMD1_WRITE,	0x03)
#define NNO_CMD_GET_PGA                 CMD_KEY(0x02 ,0x28, CMD1_READ,	0x01)
#define NNO_CMD_CALIB_STRUCT_SAVE       CMD_KEY(0x02 ,0x29, CMD1_WRITE,	CALIB_COEFFS_SIZE)
#define NNO_CMD_CALIB_STRUCT_READ       CMD_KEY(0x02, 0x2A, CMD1_READ,	CALIB_COEFFS_SIZE)
#define NNO_CMD_START_SNRSCAN           CMD_KEY(0x02 ,0x2B, CMD1_WRITE,	0x00)
#define NNO_CMD_SAVE_SNRDATA            CMD_KEY(0x02 ,0x2C, CMD1_READ,	0x0C)
#define NNO_CMD_CALIB_GEN_PTNS          CMD_KEY(0x02 ,0x2D, CMD1_WRITE,	0x01)
#define NNO_CMD_SCAN_NUM_REPEATS        CMD_KEY(0x02 ,0x2E, CMD1_WRITE,	0x02)
#define NNO_CMD_START_HADSNRSCAN        CMD_KEY(0x02 ,0x2F, CMD1_WRITE,	0x00)
#define NNO_CMD_REFCAL_PERFORM          CMD_KEY(0x02 ,0x30, CMD1_WRITE,	0x01)
#define NNO_CMD_PERFORM_SCAN_FLASH_PTNS	CMD_KEY(0x02 ,0x31, CMD1_WRITE,	0x01)
#define NNO_CMD_SERIAL_NUMBER_WRITE     CMD_KEY(0x02 ,0x32, CMD1_WRITE,	0x08)
#define NNO_CMD_SERIAL_NUMBER_READ      CMD_KEY(0x02 ,0x33, CMD1_READ,	0x00)
#define NNO_CMD_WRITE_SCAN_NAME_TAG     CMD_KEY(0x02, 0x34, CMD1_WRITE,	SCAN_NAME_TAG_SIZE)
#define NNO_CMD_DEL_SCAN_FILE_SD   		CMD_KEY(0x02, 0x35, CMD1_WRITE,	0x04)
#define NNO_CMD_EEPROM_MASS_ERASE       CMD_KEY(0x02 ,0x36, CMD1_WRITE,	0x00)
#define NNO_CMD_READ_SCAN_TIME			CMD_KEY(0x02, 0x37, CMD1_READ,	0x00)
#define NNO_CMD_DEL_LAST_SCAN_FILE_SD   CMD_KEY(0x02, 0x38, CMD1_WRITE,	0x00)
#define NNO_CMD_START_SCAN_INTERPRET    CMD_KEY(0x02, 0x39, CMD1_WRITE, 0x00)
#define NNO_CMD_SCAN_INTERPRET_GET_STATUS CMD_KEY(0x02, 0x3A, CMD1_READ,  0x01)
#define NNO_CMD_MODEL_NAME_WRITE        CMD_KEY(0x02 ,0x3B, CMD1_WRITE,	0x10)
#define NNO_CMD_MODEL_NAME_READ         CMD_KEY(0x02 ,0x3C, CMD1_READ,	0x00)
#define NNO_CMD_READ_TEMP   			CMD_KEY(0x03, 0x00, CMD1_READ,	0x00)
#define NNO_CMD_READ_HUM				CMD_KEY(0x03, 0x02, CMD1_READ,	0x00)
#define NNO_CMD_SET_DATE_TIME			CMD_KEY(0x03, 0x09, CMD1_WRITE,	0x07)
#define NNO_CMD_READ_BATT_VOLT			CMD_KEY(0x03, 0x0A, CMD1_READ,	0x00)
#define NNO_CMD_READ_TIVA_TEMP			CMD_KEY(0x03 ,0x0B, CMD1_READ,	0x00)
#define NNO_CMD_GET_DATE_TIME			CMD_KEY(0x03 ,0x0C, CMD1_READ,	0x00)
#define NNO_CMD_HIBERNATE_MODE			CMD_KEY(0x03 ,0x0D, CMD1_WRITE,	0x00)
#define NNO_CMD_SET_HIBERNATE           CMD_KEY(0x03, 0x0E, CMD1_WRITE, 0x01)
#define NNO_CMD_GET_HIBERNATE           CMD_KEY(0x03, 0x0F, CMD1_READ,  0x01)
#define NNO_CMD_GET_NUM_SCAN_FILES_SD	CMD_KEY(0x04 ,0x00, CMD1_READ,	0x00)
#define NNO_CMD_READ_PHOTODETECTOR  	CMD_KEY(0x04 ,0x02, CMD1_READ,	0x00)
#define NNO_CMD_READ_DEVICE_STATUS		CMD_KEY(0x04 ,0x03, CMD1_READ,	0x00)
#define NNO_CMD_READ_ERROR_STATUS		CMD_KEY(0x04 ,0x04, CMD1_READ,	0x00)
#define NNO_CMD_RESET_ERROR_STATUS		CMD_KEY(0x04 ,0x05, CMD1_WRITE,	0x00)
#define NNO_CMD_GET_SPECIFIC_ERR_STATUS	CMD_KEY(0x04 ,0x06, CMD1_READ,	0x04)
#define NNO_CMD_GET_SPECIFIC_ERR_CODE	CMD_KEY(0x04 ,0x07, CMD1_READ,	0x01)
#define NNO_CMD_CLEAR_SPECIFIC_ERR		CMD_KEY(0x04 ,0x08, CMD1_WRITE,	0x04)
#define NNO_CMD_UPDATE_REFCALDATA_WOREFL CMD_KEY(0x04 ,0x0A, CMD1_WRITE,0x00)
#define NNO_CMD_ERASE_DLPC_FLASH	    CMD_KEY(0x04, 0x0B, CMD1_WRITE, 0x00)
#define NNO_CMD_SET_FIXED_PGA           CMD_KEY(0x04 ,0x0C, CMD1_WRITE,	0x02)


#endif // NNO_COMMANDEFS_H
