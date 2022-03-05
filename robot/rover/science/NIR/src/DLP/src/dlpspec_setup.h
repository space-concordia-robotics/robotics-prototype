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
#ifndef _DLPSPEC_SETUP_H
#define _DLPSPEC_SETUP_H

/** Minimum DMD column number */
#define MIN_DMD_COLUMN	0

/** Maximum- DMD column number */
#define MAX_DMD_COLUMN  853

/**
 * Max patterns supported. This should be no larger than the allocated
 * size of the pattern frame buffer in the embedded device.
 */
#define MAX_PATTERNS_PER_SCAN 624

/**
 * @brief Workaround for pre-1.1.8 bug in NIRscan Nano Tiva firmware which 
 * would overwrite the first two or four bytes of serialized reference calibration
 * matrix and reference calibration coefficients. If not using this library with
 * any systems with that version of firmware or prior, the developer should undefine
 * this macro.
 */
#define NANO_PRE_1_1_8_BLE_WORKAROUND

/**
 * @brief Number of coefficients in polynomial fitting functions.
 *
 * Controls the number of coefficients and therefore the order of the polynomial
 * used for the function relating DMD columns to wavelengths, and the function
 * controlling pattern curvature. *NOTE: Matrix determinant computation in
 * dlpspec_calib_genPxToPyCoeffs() must be changed if you change this.
 */
#define PX_TO_LAMBDA_NUM_POL_COEFF	3

#endif //_DLPSPEC_SETUP_H
