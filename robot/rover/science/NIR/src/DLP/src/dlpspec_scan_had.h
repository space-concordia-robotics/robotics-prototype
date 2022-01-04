/*****************************************************************************
**
**  Copyright (c) 2015 Texas Instruments Incorporated.
**
******************************************************************************
**
**  DLP
**
*****************************************************************************/

#ifndef _DLPSPEC_SCAN_HAD_H
#define _DLPSPEC_SCAN_HAD_H

/**
 * @addtogroup group_scan_had
 *
 * @{
 */

/**
 * Maximum Hadamard matrix order requested. Increasing this will increase the code
 * space used by the compiled library and increase the size of the maximum 
 * Hadamard matrix used when creating Hadamard scans. Decreasing this too much 
 * will create sparse Hadamard sets, where much less than 50% of the DMD pixels
 * are in the on state during each pattern in a Hadamard scan. Increasing this 
 * above the following, where EXP = the expected number of columns between the
 * shortest and longest wavelength supported by the spectrometer will not provide 
 * any additional functionality.
 * 
 * EXP / #MIN_COL_GROUP_WIDTH + #MIN_PX_BETWEEN_COL_GROUPS
 * 
 * The actual matrix order available is based on this value and accessible as 
 * #HAD_MATRIX_MAX_ORDER_AVAIL.
 */
#define HAD_MATRIX_MAX_ORDER_REQ 256

/**
 * @def MAX_HADAMARD_CREATED
 * Input to Hadamard matrix pre-compilation (see pre-compile/dump_mat.c).
 * Defines the maximum Hadamard matrix created in the included .bin files.
 */

/**
 * @def HAD_MATRIX_MAX_ORDER_AVAIL
 * The largest Hadamard order that exists without being greater than 
 * #HAD_MATRIX_MAX_ORDER_REQ. This is the largest matrix available to the library.
 */

// Includes
#include <stdint.h>
#include "dlpspec_types.h"
#include "dlpspec_scan.h"

/** Minimum width of a Hadamard column group, in pixels. */
#define MIN_COL_GROUP_WIDTH 1
/**
 * Minimum gap of always off pixels between two adjacent column groups, in pixels.
 * This is necessary in order to assure that diffraction efficiency changes will
 * not occur within different patterns of the same Hadamard set due to the 
 * potential difference in the width of a group of on pixels as two column groups
 * become adjacent. Hadamard sampling theory assumes that different portions of the spectrum
 * the spectrum add linearly, and that can only be achieved if the system response
 * per wavelength is constant over all patterns of a given Hadamard set.
 */
#define MIN_PX_BETWEEN_COL_GROUPS 2
/** 
 * Maximum width of a Hadamard column group, in pixels. Widths beyond this may
 * stil generate valid pattern sets, but are not necessarily guaranteed to, as
 * this sets the array size used in the Hadamard pattern definition.
 */
#define MAX_COL_GROUP_WIDTH 14
/** Maximum number of Hadamard sets per Hadamard scan, computed from other inputs. */
#define MAX_HAD_SETS (MAX_COL_GROUP_WIDTH + MIN_PX_BETWEEN_COL_GROUPS)
/** 
 * Maximum number of column groups in a single Hadamard set, computed from other inputs. 
 * This may be arbitrarily limited to #HAD_MATRIX_MAX_ORDER_AVAIL if desired.
 */
#define MAX_HAD_COL_GROUPS ((MAX_DMD_COLUMN + 1) / (MIN_COL_GROUP_WIDTH + MIN_PX_BETWEEN_COL_GROUPS))

/** 
 * @brief Describes the parameters of a single Hadamard set.
 */
typedef struct
{
    uint16_t    numColGroups; /**< Number of unique column groups shown on the DMD during all of the patterns */
    uint16_t    hadOrder; /**< Hadamard matrix size used to generate this Hadamard set */
    uint16_t    colGroupNum[MAX_HAD_COL_GROUPS]; /**< Array containing the indices of the overall column group represented by each column group in this Hadamard set */
    uint16_t    colMidPix[MAX_HAD_COL_GROUPS]; /**< The DMD column number corresponding to the center of the on-state group of pixels for each column group in this Hadamard set */
}hadSet;

/** 
 * @brief Describes an entire Hadamard pattern definition.
 * 
 * This usually includes multiple
 * Hadamard sets, in order to ensure there is no overlap between adjacent column
 * groups, and that #MIN_PX_BETWEEN_COL_GROUPS is observed.
 */
typedef struct
{
	uint16_t    numPatterns; /**< Number of binary DMD patterns required for the scan */
	uint16_t    colWidth; /**< Width in pixels of each group of on-state pixels in each pattern */
    uint8_t     numSets; /**< Number of Hadamard sets the scan is broken into, to account for edge diffraction effects. */
	hadSet      set[MAX_HAD_SETS]; /**< Array of Hadamard set definitions */
}patDefHad;


#ifdef __cplusplus
extern "C" {
#endif

// Function prototypes
DLPSPEC_ERR_CODE dlpspec_scan_had_interpret(const uScanData *pScanData, 
		scanResults *pResults);
DLPSPEC_ERR_CODE dlpspec_scan_had_genPatDef(const scanConfig *pScanConfig, 
		const calibCoeffs *pCoeffs, patDefHad *patDefH);
int32_t dlpspec_scan_had_genPatterns(const patDefHad *patDefHad, 
		const FrameBufferDescriptor *pFB, uint32_t startPattern);


#ifdef __cplusplus      /* matches __cplusplus construct above */
}
#endif

/** @} // group group_scan_had
 *
 */

#endif //_DLPSPEC_SCAN_HAD_H
