/*****************************************************************************
**
**  Copyright (c) 2015 Texas Instruments Incorporated.
**
******************************************************************************
**
**  DLP
**
*****************************************************************************/

#ifndef _DLP_SPEC_COL_H
#define _DLP_SPEC_COL_H

// Includes
#include <stdint.h>
#include "dlpspec_types.h"
#include "dlpspec_scan.h"

/**
 * @addtogroup group_scan_col
 *
 * @{
 */

/**
 * @brief Describes a Column pattern definition. 
 * 
 * Describes the patterns necessary for a particular column scan on a specific 
 * DLP spectrometer. This is then used by the pattern generation function to generate
 * the actual frame buffer full of patterns.
 */
typedef struct 
{
	uint16_t numPatterns; /**< Number of binary DMD patterns required for the scan */
	uint16_t colWidth; /**< Width in pixels of each group of on-state pixels in each pattern */
	uint16_t colMidPix[MAX_PATTERNS_PER_SCAN]; /**< The DMD column number corresponding to the center of the on-state group of pixels for each pattern */
}patDefCol;

#ifdef __cplusplus
extern "C" {
#endif

// Function prototypes
int32_t dlpspec_scan_col_genPatterns(const patDefCol *patDefCol,
	   	const FrameBufferDescriptor *pFB, uint32_t startPattern);
DLPSPEC_ERR_CODE dlpspec_scan_col_genPatDef(const scanConfig *pScanConfig, 
		const calibCoeffs *pCoeffs, patDefCol *patDef);
DLPSPEC_ERR_CODE dlpspec_scan_col_interpret(const uScanData *pScanData, 
		scanResults *pResults);


#ifdef __cplusplus      /* matches __cplusplus construct above */
}
#endif

/** @} // group group_scan_col
 *
 */

#endif //_DLP_SPEC_COL_H
