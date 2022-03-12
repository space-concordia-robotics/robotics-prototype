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
#ifndef _DLPSPEC_CALIB_H
#define _DLPSPEC_CALIB_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "dlpspec_types.h"
#include "dlpspec_scan.h"

/**
 * @addtogroup group_calib
 *
 * @{
 */

#define SIGN(x) ((x) >= 0 ? 1 : -1)

/** 
 * @name Calibration pattern definitions
 * 
 * These values determine the DMD row (Y axis) locations and height for the 
 * partial height top, middle, and bottom scans. These three scans are used 
 * during calibration to map out the curvature and wavelength positions of the 
 * DMD.
 */
//@{

/**
 * @brief Pixel width of calibration patterns.
 * 
 * Special calibration patterns should be wide enough to capture enough signal
 * for calibration, and narrow enough so that the total resolution of the
 * instrument (optical resolution \f$ \ast \f$ DMD pixel pitch \f$ \ast \f$
 * pattern width) is valid for the calibration sample being used.
 */
#define WAVELEN_CAL_PTN_WIDTH 5

/**
 * @brief DMD column center of first calibration pattern.
 * 
 * For client to relate calibration pattern number to calibration center DMD
 * column. For example: DMD column = patternNumber + 
 * WAVELEN_CAL_PTN_CENTER_OFFSET. WAVELEN_CAL_PTN_CENTER_OFFSET must be greater than:
 * `(#MIN_DMD_COLUMN + ((WAVELEN_CAL_PTN_WIDTH - 1)/2))`, which ensures that
 * the first column used in the first calibration pattern is within the DMD extents
 * defined by #MIN_DMD_COLUMN and #MAX_DMD_COLUMN.
 */
#define WAVELEN_CAL_PTN_CENTER_OFFSET 2
/** DMD height in pixels of partial-height DMD scans. */
#define DMD_TOP_MID_BOT_SCAN_HEIGHT 120
/** DMD row number of the top of the DMD top scan. */
#define DMD_TOP_SCAN_START_Y 60
/** DMD row number of the center of the DMD top scan. */
#define DMD_TOP_SCAN_CENTRE_Y (DMD_TOP_SCAN_START_Y + DMD_TOP_MID_BOT_SCAN_HEIGHT/2)
/** DMD row number of the top of the DMD middle scan. */
#define DMD_MID_SCAN_START_Y 180
/** DMD row number of the center of the DMD middle scan. */
#define DMD_MID_SCAN_CENTRE_Y (DMD_MID_SCAN_START_Y + DMD_TOP_MID_BOT_SCAN_HEIGHT/2)
/** DMD row number of the top of the DMD bottom scan. */
#define DMD_BOT_SCAN_START_Y 300
/** DMD row number of the center of the DMD bottom scan. */
#define DMD_BOT_SCAN_CENTRE_Y (DMD_BOT_SCAN_START_Y + DMD_TOP_MID_BOT_SCAN_HEIGHT/2)
//@}

/**
 * @brief Supported alignment and calibration scan types.
 * 
 * These scans are predefined and separate from the general scan definition 
 * through scan configurations. They are used during unit alignment and 
 * calibration only.
 */
typedef enum
{
// Library scan types: 0-127 reserved for future library expansion
    SLIT_ALIGN_SCAN = 2,
    DET_ALIGN_SCAN  = 3,
	LEFT_DMD_SCAN = 4,
	LEFT_DMD_TOP_SCAN = 5,
	LEFT_DMD_MID_SCAN = 6,
	LEFT_DMD_BOT_SCAN = 7,
	RIGHT_DMD_SCAN = 8,
	RIGHT_DMD_TOP_SCAN = 9,
	RIGHT_DMD_MID_SCAN = 10,
	RIGHT_DMD_BOT_SCAN = 11,
	CALIB_SCAN_TYPES_MAX = RIGHT_DMD_BOT_SCAN
// User extended scan types: 128-255 reserved for customer expansion
}CALIB_SCAN_TYPES;


#ifdef __cplusplus
extern "C" {
#endif

/** @brief Write ::refCalMatrix struct to serialized blob. */
DLPSPEC_ERR_CODE dlpspec_calib_write_ref_matrix(const refCalMatrix *pData, void *pBuf, const size_t bufSize);
/** @brief Compute full width half max of a peak. */
DLPSPEC_ERR_CODE dlpspec_calib_getFWHM(const double *values, const int num_values, const int peak_location, int * peak_height, double * fwhm);
/** @brief Compute left and right locations mid-way between peak and trough. */
DLPSPEC_ERR_CODE dlpspec_calib_get_halfmax_loc(double *values, int num_values, int peak_location, double *left_halfmax_loc, double *right_halfmax_loc);
/** @brief Find peaks in a spectrum. */
int32_t dlpspec_calib_findPeaks(const double *values, const int num_values, const double peak_sel_divisor, int *peak_inds);
/** @brief Interpolate peak location between evenly spaced discrete values. */
DLPSPEC_ERR_CODE dlpspec_calib_findPeaks3(const double y1, const double y2, const double y3, double * offset);
/** @brief Interpolate peak location between non evenly spaced discrete values. */
DLPSPEC_ERR_CODE dlpspec_calib_findPeakInterp(const double x1,const double x2,const double x3,const double y1,const double y2,const double y3,double * xe,double * ye,double * d2);
/** @brief Check if peak positions are in similar relative positions to expected. */
DLPSPEC_ERR_CODE dlpspec_calib_checkPeakDist(const double *peak_pos, const int num_values, const double *ref_pos, const double tolerance);
/** @brief Generate polynomial coefficients from input (x,y) points. */
DLPSPEC_ERR_CODE dlpspec_calib_genPxToPyCoeffs(const int num_peaks, const double *px_measured, const double *py_measured, double*px_to_py_coeffs, double *rsquared);
/** @brief Generate polynomial coefficients for slit image curve fitting. */
DLPSPEC_ERR_CODE dlpspec_calib_genPxyToCurveCoeffs(const double *peaks, const double *y_values,  const int num_peaks, const int num_measurements, double *pxy_to_curve_coeffs);
/** @brief Generate shift vector from polynomial coefficients for slit image curve fitting. */
DLPSPEC_ERR_CODE dlpspec_calib_genShiftVector(const double *pol_coeffs, const int dmd_height, int8_t *shift_vector);
/** @brief Generate calibration patterns. */
int32_t dlpspec_calib_genPatterns(const CALIB_SCAN_TYPES scan_type, FrameBufferDescriptor *pFB);
/** @brief Interpret serialized #scanData blob from a calibration scan. */
DLPSPEC_ERR_CODE dlpspec_calib_interpret(const void *pBuf, const size_t bufSize, scanResults *pResults, const CALIB_SCAN_TYPES type);
/** @brief Serialize calibration coefficients. */
DLPSPEC_ERR_CODE dlpspec_calib_write_data(const calibCoeffs *pCfg, void *pBuf, const size_t bufSize);
/** @brief Deserialize calibration coefficients. */
DLPSPEC_ERR_CODE dlpspec_calib_read_data(void *pBuf, size_t bufSize);


/** @} // group group_calib
 *
 */

#ifdef __cplusplus      /* matches __cplusplus construct above */
}
#endif


#endif //_DLPSPEC_CALIB_H
