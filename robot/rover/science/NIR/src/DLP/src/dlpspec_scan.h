/*****************************************************************************
**
**  Copyright (c) 2015 Texas Instruments Incorporated.
**
******************************************************************************
**
**  DLP
**
*****************************************************************************/


// Inclusion Guard
#ifndef _DLPSPEC_SCAN_H
#define _DLPSPEC_SCAN_H


// Includes
#include <stdint.h>
#include <stddef.h>
#include "dlpspec_setup.h"
#include "dlpspec_types.h"

/**
 * @addtogroup group_scan
 *
 * @{
 */

/** Version number for future compatibility if changes are required */
#define CUR_SCANDATA_VERSION 1

/** Maximum number of sections allowed in a slew scan definition/config */
#define SLEW_SCAN_MAX_SECTIONS 5

/** Supported scan types */
typedef enum
{
// Library scan types: 0-127 reserved for future library expansion
    COLUMN_TYPE     = 0,
    HADAMARD_TYPE   = 1,
    SLEW_TYPE		= 2,
// User extended scan types: 128-255 reserved for customer expansion
}SCAN_TYPES;


/** 
 * @name Definitions for scanConfig
 * 
 * These values determine the DMD row (Y axis) locations and height for the 
 * partial height top, middle, and bottom scans. These three scans are used 
 * during calibration to map out the curvature and wavelength positions of the 
 * DMD.
 */
/// @{

#define SCAN_CFG_FILENAME_LEN 40
#define NANO_SER_NUM_LEN 8
#define SCAN_CONFIG_HEAD \
    uint8_t     scan_type; /**< must be defined in #SCAN_TYPES */ \
    uint16_t    scanConfigIndex; /**< Unique ID per spectrometer which is modified when the config is changed. Can be used to determine whether a cached version of the config is valid per spectrometer SN. */ \
    char        ScanConfig_serial_number[NANO_SER_NUM_LEN];  /**< Serial number of the spectrometer */\
    char        config_name[SCAN_CFG_FILENAME_LEN]; /**< User friendly scan configuration name for display */
#define SCAN_CONFIG_HEAD_FORMAT "cvc#c#"

#define SCAN_CONFIG_STUB         \
    uint16_t    wavelength_start_nm; /**< Minimum wavelength to start the scan from, in nm. */ \
    uint16_t    wavelength_end_nm;  /**< Maximum wavelength to end the scan at, in nm. */ \
    uint8_t     width_px;  /**< Pixel width of the patterns. Increasing this will increase SNR, but reduce resolution. */ \
    uint16_t    num_patterns; /**< Number of desired points in the spectrum. */ \
    uint16_t    num_repeats; /**< Number of times to repeat the scan on the spectromter before averaging the scans together and returning the results. This can be used to increase integration time. */
#define SCAN_CFG_STUB_FORMAT "vvcvv"

/**
 * @brief Describes a scan configuration.
 */
typedef struct
{
    SCAN_CONFIG_HEAD
    SCAN_CONFIG_STUB
}scanConfig;


typedef enum
{
	T_635_US,
	T_1270_US, 
	T_2450_US,
	T_5080_US,
	T_15240_US,
	T_30480_US,
	T_60960_US,
}EXP_TIME;

typedef struct
{
    uint8_t     section_scan_type; /**< must be defined in #SCAN_TYPES */ \
    uint8_t     width_px;  /**< Pixel width of the patterns. Increasing this
							 will increase SNR, but reduce resolution. */ \
    uint16_t    wavelength_start_nm; /**< Minimum wavelength to start the scan 
									   from, in nm. */ \
    uint16_t    wavelength_end_nm;  /**< Maximum wavelength to end the scan at,
									  in nm. */ \
    uint16_t    num_patterns; /**< Number of desired points in the spectrum. */ \
    uint16_t    exposure_time; /**< Time for for which each pattern	in this 
								 section will be exposed. Values should be 
								as per EXP_TIME enum above	 */ \
}slewScanSection;
#define SLEW_SCAN_CFG_SECT_FORMAT "ccvvvv"

struct slewScanConfigHead
{
    SCAN_CONFIG_HEAD
    uint16_t    num_repeats; /**< Number of times to repeat the scan on the spectromter before averaging the scans together and returning the results. This can be used to increase integration time. */
	uint8_t		num_sections; /**< Number of sections that make up this scan defintion */
};

typedef struct
{
	struct slewScanConfigHead head;
	slewScanSection section[SLEW_SCAN_MAX_SECTIONS];
}slewScanConfig;

typedef union
{
	scanConfig scanCfg;
	slewScanConfig slewScanCfg;
}uScanConfig;

#define SLEW_SCAN_CFG_HEAD_FORMAT SCAN_CONFIG_HEAD_FORMAT "vc"
/**
 * TPL format string for #scanConfig
 */
#define SCAN_CFG_FORMAT SCAN_CONFIG_HEAD_FORMAT SCAN_CFG_STUB_FORMAT

/// @}

/** 
 * @name Definitions for #scanData
 * 
 * These values determine the DMD row (Y axis) locations and height for the 
 * partial height top, middle, and bottom scans. These three scans are used 
 * during calibration to map out the curvature and wavelength positions of the 
 * DMD.
 */
/// @{
#define SCAN_DATA_VERSION \
    uint32_t    header_version; /**< Version number for future backward compatibility in the case that this structure changes. */
#define SCAN_DATA_VERSION_FORMAT  "u"

#define DATE_TIME_STRUCT \
    uint8_t     year; /**< years since 2000 */ \
    uint8_t     month; /**< months since January [0-11] */ \
    uint8_t     day; /**< day of the month [1-31] */ \
    uint8_t     day_of_week; /**< days since Sunday [0-6] */ \
    uint8_t     hour; /**< hours since midnight [0-23] */ \
    uint8_t     minute; /**< minutes after the hour [0-59] */ \
    uint8_t     second; /**< seconds after the minute [0-60] */
#define DATE_TIME_FORMAT "ccccccc"

#define SCAN_NAME_LEN 20
#define SCAN_DATA_HEAD_NAME                                     \
    char                scan_name[SCAN_NAME_LEN]; /**< User friendly scan name */ \

#define SCAN_DATA_HEAD_BODY                                     \
    int16_t             system_temp_hundredths; /**< System temperature in hundredths of a degree Celsius: 123 = 1.23 degC */ \
    int16_t             detector_temp_hundredths; /**< Detector temperature in hundredths of a degree Celsius: 123 = 1.23 degC */ \
    uint16_t            humidity_hundredths; /**< Relative humidity in hundredths of a percent: 123 = 1.23% relative humidity */ \
    uint16_t            lamp_pd; /**< Lamp monitor photodiode value */ \
    uint32_t            scanDataIndex; /**< Unique index for scan. Can be used to determine whether a scan has already been downloaded from a particular spectrometer SN. */ \
    calibCoeffs 	    calibration_coeffs; /**< Calibration coefficients for the spectrometer this scan was taken with */ \
    char                serial_number[NANO_SER_NUM_LEN]; /**< Serial number of the spectrometer this scan was taken with */ \
    uint16_t            adc_data_length; /**< Number of ADC samples in adc_data array */ \
    uint8_t             black_pattern_first; /**< First occurrence of an all-off DMD pattern during the scan, zero indexed. */ \
    uint8_t             black_pattern_period; /**< Period of black pattern recurrence */ \
    uint8_t				pga; /**< PGA gain used during this scan */

#define SCAN_DATA_HEAD SCAN_DATA_HEAD_NAME DATE_TIME_STRUCT SCAN_DATA_HEAD_BODY

#define SCAN_DATA_HEAD_FORMAT "c#" DATE_TIME_FORMAT "jjvvu" "$(" CALIB_COEFFS_FORMAT ")" "c#vccc"


/** 
 * Could be reduced to `MAX_PATTERNS_PER_SCAN + ((MAX_PATTERNS_PER_SCAN + 23)/24)`.
 * Kept at 864 for legacy reasons even though it's longer than necessary
 */ 
#define ADC_DATA_LEN 864

/**
 * @brief Data output of a scan.
 * 
 * Contains all necessary information to interpret into an intensity spectrum.
 */
typedef struct
{
    SCAN_DATA_VERSION
    SCAN_DATA_HEAD_NAME
    DATE_TIME_STRUCT
    SCAN_DATA_HEAD_BODY
    SCAN_CONFIG_HEAD
    SCAN_CONFIG_STUB
    int32_t   adc_data[ADC_DATA_LEN];
} scanData;

#define ADC_DATA_FORMAT "i#"

/**
 * TPL format string for #scanData
 */
#define SCAN_DATA_FORMAT SCAN_DATA_VERSION_FORMAT SCAN_DATA_HEAD_FORMAT SCAN_CFG_FORMAT ADC_DATA_FORMAT

typedef struct
{
    SCAN_DATA_VERSION
    SCAN_DATA_HEAD_NAME
    DATE_TIME_STRUCT
    SCAN_DATA_HEAD_BODY
	slewScanConfig slewCfg;
    int32_t   adc_data[ADC_DATA_LEN];
} slewScanData;
/**
 * TPL format string for #slewScanData head
 */
#define SLEW_SCAN_DATA_HEAD_FORMAT SCAN_DATA_VERSION_FORMAT SCAN_DATA_HEAD_FORMAT 

typedef union
{
	scanData data;
	slewScanData slew_data;
} uScanData;

/**
 * Safe length for a serialized #scanData blob
 */
#define SCAN_DATA_BLOB_SIZE (sizeof(uScanData)+150)
#define OLD_SCAN_DATA_BLOB_SIZE (sizeof(scanData)+100)

/// @}

/**
 * @brief Scan results, which is generated when interpreting data from #scanData.
 */
typedef struct
{
    SCAN_DATA_VERSION
    SCAN_DATA_HEAD_NAME
    DATE_TIME_STRUCT
    SCAN_DATA_HEAD_BODY
    slewScanConfig	cfg; /**< Scan configuration used to take this scan */
    double		wavelength[ADC_DATA_LEN]; /**< Computed wavelength center in nm for each corresponding intensity value */
    int         intensity[ADC_DATA_LEN]; /**< Computed intensity for each corresponding wavelength center */
    int         length; /**< number of valid elements in the wavelength and intensity arrays */
} scanResults;


#ifdef __cplusplus
extern "C" {
#endif

DLPSPEC_ERR_CODE dlpspec_get_scan_config_dump_size(const uScanConfig *pCfg, 
		size_t *pBufSize);
DLPSPEC_ERR_CODE dlpspec_scan_read_configuration(void *pBuf, const size_t bufSize);
DLPSPEC_ERR_CODE dlpspec_scan_write_configuration(const uScanConfig *pCfg, 
		void *pBuf, const size_t bufSize);
DLPSPEC_ERR_CODE dlpspec_get_scan_data_dump_size(const uScanData *pData, 
		size_t *pBufSize);
DLPSPEC_ERR_CODE dlpspec_scan_interpret(const void *pBuf, const size_t bufSize,
	   	scanResults *pResults);
DLPSPEC_ERR_CODE dlpspec_scan_write_data(const uScanData *pData, void *pBuf, 
		const size_t bufSize);
DLPSPEC_ERR_CODE dlpspec_scan_read_data(void *pBuf, const size_t bufSize);
DLPSPEC_ERR_CODE dlpspec_scan_interpReference(const void *pRefCal, 
		size_t calSize, const void *pMatrix, size_t matrixSize, 
		const scanResults *pScanResults, scanResults *pRefResults);
int32_t dlpspec_scan_genPatterns(const uScanConfig* pCfg, 
		const calibCoeffs *pCoeffs, const FrameBufferDescriptor *pFB);
DLPSPEC_ERR_CODE dlpspec_scan_bendPatterns(const FrameBufferDescriptor *pFB , 
		const calibCoeffs* calCoeff, const int32_t numPatterns);
SCAN_TYPES dlpspec_scan_slew_get_cfg_type(const slewScanConfig *pCfg);
int16_t dlpspec_scan_slew_get_num_patterns(const slewScanConfig *pCfg);
int16_t dlpspec_scan_slew_get_end_nm(const slewScanConfig *pCfg);
uint32_t dlpspec_scan_get_exp_time_us(EXP_TIME time_enum);
DLPSPEC_ERR_CODE dlpspec_scan_section_get_adc_data_range(const slewScanData 
		*pData, int section_index,int *p_section_start_index, uint16_t *p_num_patterns,
		uint16_t *p_num_black_patterns);

#ifdef __cplusplus      /* matches __cplusplus construct above */
}
#endif

/** @} // group group_scan
 *
 */

#endif //_DLPSPEC_SCAN_H
