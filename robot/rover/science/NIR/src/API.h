/*
 * API.h
 *
 * This module provides C callable APIs for each of the command supported by LightCrafter4500 platform and detailed in the programmer's guide.
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
*/

/** @file
 */

#ifndef API_H
#define API_H

#include "Common.h"
#include "dlpspec_scan.h"
#include "dlpspec_calib.h"
#include "NNOSNRDefs.h"
#include "NNOCommandDefs.h"
#include "NNOStatusDefs.h"

/* Bit masks. */
#define BIT0        0x01
#define BIT1        0x02
#define BIT2        0x04
#define BIT3        0x08
#define BIT4        0x10
#define BIT5        0x20
#define BIT6        0x40
#define BIT7        0x80
#define BIT8      0x0100
#define BIT9      0x0200
#define BIT10     0x0400
#define BIT11     0x0800
#define BIT12     0x1000
#define BIT13     0x2000
#define BIT14     0x4000
#define BIT15     0x8000
#define BIT16 0x00010000
#define BIT17 0x00020000
#define BIT18 0x00040000
#define BIT19 0x00080000
#define BIT20 0x00100000
#define BIT21 0x00200000
#define BIT22 0x00400000
#define BIT23 0x00800000
#define BIT24 0x01000000
#define BIT25 0x02000000
#define BIT26 0x04000000
#define BIT27 0x08000000
#define BIT28 0x10000000
#define BIT29 0x20000000
#define BIT30 0x40000000
#define BIT31 0x80000000

#define DMD_WIDTH 864
#define DMD_HEIGHT 480

#define PASS 0
#define FAIL -1
#define NNO_CMD_NACK -2
#define NNO_CMD_BUSY -3
#define NNO_READ_TIMEOUT -4

#define NNO_SCAN_IN_PROGRESS 0
#define NNO_SCAN_COMPLETE 1

#define NNO_STORE_SCAN_IN_SD true
#define NNO_DONT_STORE_SCAN_IN_SD false

#define UART_START_IND_BYTE_0 0x41
#define UART_START_IND_BYTE_1 0x42
#define UART_START_IND_BYTE_2 0x43
#define UART_START_IND_BYTE_3 0x44

#define UART_END_IND_BYTE_0 UART_START_IND_BYTE_3
#define UART_END_IND_BYTE_1 UART_START_IND_BYTE_2
#define UART_END_IND_BYTE_2 UART_START_IND_BYTE_1
#define UART_END_IND_BYTE_3 UART_START_IND_BYTE_0
enum scanType
{
    USE_SPL_PATTERNS_FROM_SPLASH,
    USE_PATTERN_FROM_SDRAM,
    USE_PATTERN_FROM_SDCARD,
    GENERATE_SCAN_PATTERN,
};

/** 
 *  Enumeration of calibration scan types
 */
enum scanID
{
    SCAN_SLIT_FOCUS,
    SCAN_DET_ALIGN,
    SCAN_FULL_DMD,
    SCAN_DMD_TOP,
    SCAN_DMD_MID,
    SCAN_DMD_BOT
};
/** 
 *  Enumeration of calibration actions
 */
enum calibrationID
{
    CAL_AR_SOURCE,
	CAL_VERIFY,
    CAL_REF_SAMPLE,
    CAL_SNR_COMPUTE,
    CAL_SYS_CHECK,
};

/* APIs that query status */
int NNO_GetVersion(uint32 *pTivaSWVersion, uint32 *pDLPCSWVersion, uint32 *pDLPCFlashBuildVersion, uint32 *pSpecLibVer, uint32 *pCalDataVer, uint32 *pRefCalDataVer, uint32 *pCfgDataVer);
int8_t NNO_GetSpecificErrorStatus(uint32_t field);
int16_t NNO_GetSpecificErrorcode(uint8_t type);
int8_t NNO_ClearSpecificError(uint32_t field);
int NNO_TMP_GetErrorMessageFromErrorCode(uint32_t errorCode, char *pErrorString);
int NNO_HDC_GetErrorMessageFromErrorCode(uint32_t errorCode, char *pErrorString);
int NNO_ADC_GetErrorMessageFromErrorCode(uint32_t errorCode, char *pErrorString);
int NNO_SDC_GetErrorMessageFromErrorCode(uint32_t errorCode, char *pErrorString);
int NNO_ReadDeviceStatus(uint32 *pVal);
int NNO_ReadErrorStatus(NNO_error_status_struct* error_status);
int NNO_ResetErrorStatus();

/* Scan related APIs */
int NNO_DLPCEnable(bool enable, bool enable_lamp);
int NNO_PerformScan(bool StoreInSDCard);
int NNO_InterpretScan(void);
int NNO_GetScanComplete(void);
int NNO_GetEstimatedScanTime(void);
int NNO_GetFileSizeToRead(NNO_FILE_TYPE fileType);
int NNO_GetFileData(void *pData, int *pSizeInBytes);
int NNO_GetFile(unsigned char *pData, int sizeInBytes);
int NNO_SetScanControlsDLPCOnOff(bool enable);
int NNO_SetScanNumRepeats(uint16 num);

/* Scan config management APIs */
int NNO_SaveScanCfgInEVM(uint8 index, void *pBuffer, int bufSize);
int NNO_ApplyScanConfig(void *pBuffer, int bufSize);
int NNO_SetActiveScanIndex(uint8 index);
int NNO_GetActiveScanIndex(void);
int NNO_GetNumScanCfg(void);
int NNO_GetScanCfg(uint8 index, void *pBuf, uint32 *pSize);
int NNO_EraseAllScanCfg(void);

/* APIs for managing files in SD card */
int NNO_GetNumScanFilesInSD(void);
int NNO_DeleteLastScanFileInSD(void);

/* Software update releated APIs */
int NNO_GotoTivaBootLoader(void);
int NNO_SetFileSizeAndAction(unsigned int dataLen, NNO_FILE_ACTION action);
int NNO_WriteFileData(unsigned char *pByteArray, unsigned int dataLen);
int NNO_GetFlashChecksum(unsigned int*checksum);

/* Calibration related APIs */
int NNO_GenCalibPatterns(CALIB_SCAN_TYPES type);
int NNO_setScanSubImage(uint16 startY, uint16 height);
int NNO_SaveRefCalPerformed(void);
int NNO_StartSNRScan(void);
int NNO_StartHadSNRScan(void);
int NNO_GetSNRData(int* pVal1 , int* pVal2 , int* pVal3);
int NNO_SetSerialNumber(char* serial_number);
int NNO_GetSerialNumber(char* serial_number);
int NNO_SetModelName(char* model_number);
int NNO_GetModelName(char* model_number);
int NNO_SendEEPROMWipe(bool wipe_cal_coeffs, bool wipe_refcal_data, bool wipe_cfg_data);
int NNO_EEPROMMass_Erase( void );
int NNO_GetCalibStruct(calibCoeffs  *pCalibResult);

/* Utility APIs */
int NNO_ResetTiva(void);
int NNO_HibernateMode(void);
int NNO_SetHibernate(bool newValue);
int NNO_GetHibernate(void);
int NNO_SetPGAGain(uint8 gainVal);
int NNO_GetPGAGain(void);
int NNO_GetDateTime( uint8 *year, uint8 *month, uint8 *day, uint8 *wday, uint8 *hour, uint8 *min, uint8 *sec );
int NNO_SetDateTime( uint8 year, uint8 month, uint8 day, uint8 wday, uint8 hour, uint8 min, uint8 sec );
int NNO_SetFixedPGAGain(bool isFixed, uint8 gainVal);

/* APIs for reading various sensors on the EVM */
int NNO_ReadTemp(int *Ambient, int *Detector);
int NNO_ReadHum(uint32 *Humidity, int *HDC_Temp);
int NNO_ReadBattVolt(uint32 *Batt_volt);
int NNO_ReadTivaTemp(int *TivaTemp);
int NNO_GetPhotoDetector(uint32* red, uint32* green , uint32* blue);

/* APIs for testing various components on the EVM */
int NNO_TestEEPROM(void);
int NNO_TestADC(void);
int NNO_TestTMP(void);
int NNO_TestHDC(void);
int NNO_TestBT(bool);
int NNO_TestSDC(void);
int NNO_TestBQ(int *pStatus, int *pBatt_volt, int *pUsb_det, int *pCharge_curr, int *PTS_fault);
int NNO_TestSDRAM(void);
int NNO_TestLED(bool enable);
int NNO_ButtonTestWr( bool enable );
int NNO_ButtonTestRd( uint8 *button );
int NNO_EEPROM_CalTest(void);

/* APIs to be used as debug tools */
int NNO_SendCalibStruct(calibCoeffs  *pCalibResult);
int NNO_SetDLPCReg(uint32 addr, uint32 val);
int NNO_GetDLPCReg(uint32 addr, uint32 *pVal);
int NNO_EraseDLPC150Flash(void);
int NNO_SetScanNameTag(const char* tag, int len);

int NNO_GetSNRDataStructure(int SNRHadStruc[HADSNR_NUM_DATA][HADSNR_LENGTH]);
int NNO_UpdateRefCalDataWithWORefl(void);
int NNO_SetUARTConnected(bool connected);
#endif // API_H
