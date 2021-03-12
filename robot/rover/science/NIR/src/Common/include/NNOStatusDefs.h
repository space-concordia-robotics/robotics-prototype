/*
 * Device and error status definitions
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
 */

#ifndef NNOSTATUSDEFS_H_
#define NNOSTATUSDEFS_H_

#define MAX_NUM_DEVICE_STATUS 32
#define MAX_NUM_ERROR_STATUS  32

/************ 		Device Status Definitions **********************/
// TIVA status = 0 - ERROR, 1 - NORMAL
#define NNO_STATUS_TIVA						0x00000001
#define NNO_STATUS_SCAN_IN_PROGRESS			0x00000002
#define NNO_STATUS_SD_CARD_PRESENT			0x00000004
#define NNO_STATUS_SD_CARD_OPER_IN_PROG		0x00000008
#define NNO_STATUS_BLE_STACK_OPEN			0x00000010
#define NNO_STATUS_ACTIVE_BLE_CONNECTION	0x00000020
#define NNO_STATUS_SCAN_INTERPRET_IN_PROGRESS 0x00000040
#define NNO_STATUS_MAX						NNO_STATUS_SCAN_INTERPRET_IN_PROGRESS

/************ 		Error Status Defintions **********************/
#define NNO_ERROR_SCAN					0x00000001
#define NNO_ERROR_ADC					0x00000002
#define NNO_ERROR_SD_CARD				0x00000004
#define NNO_ERROR_EEPROM				0x00000008
#define NNO_ERROR_BLE					0x00000010
#define NNO_ERROR_SPEC_LIB				0x00000020
#define NNO_ERROR_HW					0x00000040
#define NNO_ERROR_TMP006				0x00000080
#define NNO_ERROR_HDC1000				0x00000100
#define NNO_ERROR_BATTERY_EMPTY			0x00000200
#define NNO_ERROR_INSUFFICIENT_MEMORY	0x00000400
#define NNO_ERROR_UART					0x00000800
#define NNO_ERROR_MAX					NNO_ERROR_UART

/******************** 	Scan Error codes   **********************/
#define NNO_ERROR_SCAN_DLPC150_BOOT_ERROR 			0x00000001
#define NNO_ERROR_SCAN_DLPC150_INIT_ERROR			0x00000002
#define	NNO_ERROR_SCAN_DLPC150_LAMP_DRIVER_ERROR	0x00000004
#define NNO_ERROR_SCAN_DLPC150_CROP_IMG_FAILED		0x00000008
#define NNO_ERROR_SCAN_ADC_DATA_ERROR				0x00000010
#define NNO_ERROR_SCAN_CFG_INVALID					0x00000020
#define NNO_ERROR_SCAN_PATTERN_STREAMING			0x00000040
#define NNO_ERROR_SCAN_DLPC150_READ_ERROR 			0x00000080


/******************** 	ADC Error codes   **********************/
#define NNO_ERROR_ADC_START			0x00000001
#define ADC_ERROR_TIMEOUT       	NNO_ERROR_ADC_START
#define ADC_ERROR_POWERDOWN     	(NNO_ERROR_ADC_START + 1)
#define ADC_ERROR_POWERUP       	(NNO_ERROR_ADC_START + 2)
#define ADC_ERROR_STANDBY       	(NNO_ERROR_ADC_START + 3)
#define ADC_ERROR_WAKEUP        	(NNO_ERROR_ADC_START + 4)
#define ADC_ERROR_READREGISTER  	(NNO_ERROR_ADC_START + 5)
#define ADC_ERROR_WRITEREGISTER 	(NNO_ERROR_ADC_START + 6)
#define ADC_ERROR_CONFIGURE     	(NNO_ERROR_ADC_START + 7)
#define ADC_ERROR_SETBUFFER     	(NNO_ERROR_ADC_START + 8)
#define ADC_ERROR_COMMAND       	(NNO_ERROR_ADC_START + 9)

/******************** 	SD card Error codes   *******************/
#define NNO_ERROR_SDC_START						0x00000001
#define NNO_ERROR_SD_CARD_HARD_ERROR			(NNO_ERROR_SDC_START)
#define NNO_ERROR_SD_CARD_INTERNAL_ERRROR		(NNO_ERROR_SDC_START + 1)
#define NNO_ERROR_SD_CARD_DOES_NOT_WORK			(NNO_ERROR_SDC_START + 2)
#define NNO_ERROR_SD_CARD_FILE_NOT_FOUND		(NNO_ERROR_SDC_START + 3)
#define NNO_ERROR_SD_CARD_PATH_NOT_FOUND		(NNO_ERROR_SDC_START + 4)
#define NNO_ERROR_SD_CARD_INVALID_PATH      	(NNO_ERROR_SDC_START + 5)
#define NNO_ERROR_SD_CARD_ACCESS_DENIED     	(NNO_ERROR_SDC_START + 6)
#define NNO_ERROR_SD_CARD_ACCESS_PROHIBITED 	(NNO_ERROR_SDC_START + 7)
#define NNO_ERROR_SD_CARD_INVALID_OBJECT    	(NNO_ERROR_SDC_START + 8)
#define NNO_ERROR_SD_CARD_WRITE_PROTECTED   	(NNO_ERROR_SDC_START + 9)
#define NNO_ERROR_SD_CARD_INVALID_DRIVE_NUM 	(NNO_ERROR_SDC_START + 10)
#define NNO_ERROR_SD_CARD_NOT_ENABLED        	(NNO_ERROR_SDC_START + 11)
#define NNO_ERROR_SD_CARD_INVALID_FILE_SYSTEM   (NNO_ERROR_SDC_START + 12)
#define NNO_ERROR_SD_CARD_MKFS_INVALID_PARAM	(NNO_ERROR_SDC_START + 13)
#define NNO_ERROR_SD_CARD_TIMEOUT        		(NNO_ERROR_SDC_START + 14)
#define NNO_ERROR_SD_CARD_LOCKED        		(NNO_ERROR_SDC_START + 15)
#define NNO_ERROR_SD_CARD_NOT_ENOUGH_CORE       (NNO_ERROR_SDC_START + 16)
#define NNO_ERROR_SD_CARD_TOO_MANY_OPEN_FILES   (NNO_ERROR_SDC_START + 17)

/********************* 	HW Error codes   ************************/
#define NNO_ERROR_HW_START				0x00000001
#define NNO_ERROR_HW_DLPC150			(NNO_ERROR_HW_START)
#define NNO_ERROR_HW_MAX				(NNO_ERROR_HW_START + 1) // Modify this entry when new codes are added above

/******************** 	TMP006 Error codes   *******************/
#define NNO_ERROR_TMP006_START			0x00000001
#define NNO_ERROR_TMP006_MANUID        	(NNO_ERROR_TMP006_START)
#define NNO_ERROR_TMP006_DEVID         	(NNO_ERROR_TMP006_START + 1)
#define NNO_ERROR_TMP006_RESET          (NNO_ERROR_TMP006_START + 2)
#define NNO_ERROR_TMP006_READREGISTER  	(NNO_ERROR_TMP006_START + 3)
#define NNO_ERROR_TMP006_WRITEREGISTER	(NNO_ERROR_TMP006_START + 4)
#define NNO_ERROR_TMP006_TIMEOUT	    (NNO_ERROR_TMP006_START + 5)
#define NNO_ERROR_TMP006_I2C			(NNO_ERROR_TMP006_START + 6)
#define NNO_ERROR_TMP006_MAX			(NNO_ERROR_TMP006_START + 6) // Modify this entry when new codes are added above

/******************** 	HDC1000 Error codes   *******************/
#define NNO_ERROR_HDC1000_START			0x00000001
#define NNO_ERROR_HDC1000_MANUID        (NNO_ERROR_HDC1000_START)
#define NNO_ERROR_HDC1000_DEVID         (NNO_ERROR_HDC1000_START + 1)
#define NNO_ERROR_HDC1000_RESET         (NNO_ERROR_HDC1000_START + 2)
#define NNO_ERROR_HDC1000_READREGISTER  (NNO_ERROR_HDC1000_START + 3)
#define NNO_ERROR_HDC1000_WRITEREGISTER (NNO_ERROR_HDC1000_START + 4)
#define NNO_ERROR_HDC1000_TIMEOUT		(NNO_ERROR_HDC1000_START + 5)
#define NNO_ERROR_HDC1000_I2C		    (NNO_ERROR_HDC1000_START + 6)
#define NNO_ERROR_HDC1000_MAX			(NNO_ERROR_HDC1000_START + 6) // Modify this entry when new codes are added above

typedef enum
{
	NNO_error_code_scan,
	NNO_error_code_adc,
	NNO_error_code_sd,
	NNO_error_code_eeprom,
	NNO_error_code_ble,
	NNO_error_code_spec_lib,
	NNO_error_code_hw,
	NNO_error_code_tmp,
	NNO_error_code_hdc,
	NNO_error_code_battery,
	NNO_error_code_memory,
	NNO_error_code_uart,
	NNO_error_code_max
} NNO_error_codes_type;		// the order of enums should match the error code order above

#define MAX_BLE_PKT_SIZE 20
#define RESERVED_SIZE    (MAX_BLE_PKT_SIZE - 4 - NNO_error_code_max -1)	/* -1 because
																		 * ble uses two
																		 * bytes
																		 */

typedef struct
{
 	int8_t scan;
 	int8_t adc;
 	int8_t sd;
 	int8_t eeprom;
 	int16_t ble;
 	int8_t spec_lib;
 	int8_t hw;
 	int8_t tmp;
 	int8_t hdc;
 	int8_t battery;
 	int8_t memory;
 	int8_t uart;
 	int8_t reserved[RESERVED_SIZE];	// Future use
} NNO_error_codes_struct;


typedef struct
{
	uint32_t status;
	NNO_error_codes_struct errorCodes;
} NNO_error_status_struct; // Size of struct should not exceed MTU size for BLE

typedef struct
{
	uint32_t deviceStatus;
	NNO_error_status_struct errorStatus;
} NNO_status_struct;

#endif /* NNOSTATUSDEFS_H_ */
