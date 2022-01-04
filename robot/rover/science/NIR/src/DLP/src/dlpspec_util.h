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
#ifndef _DLPSPEC_UTIL_H
#define _DLPSPEC_UTIL_H

#include <stdint.h>
#include "dlpspec_setup.h"
#include "dlpspec_types.h"

/**
 * @addtogroup group_util
 *
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

DLPSPEC_ERR_CODE dlpspec_util_nmToColumn(const double nm, const double *coeffs, double *column);
DLPSPEC_ERR_CODE dlpspec_util_columnToNm(const double column,  const double *coeffs, double *nm);
DLPSPEC_ERR_CODE dlpspec_util_columnToNmDistance(const double column_distance, const double *coeffs, double *nm);

#ifdef __cplusplus      /* matches __cplusplus construct above */
}
#endif

/** @} // group group_util
 *
 */

#endif //_DLPSPEC_UTIL_H
