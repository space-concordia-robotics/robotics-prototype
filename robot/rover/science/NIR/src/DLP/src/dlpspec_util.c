/*****************************************************************************
**
**  Copyright (c) 2015 Texas Instruments Incorporated.
**
******************************************************************************
**
**  DLP Spectrum Library
**
*****************************************************************************/

#include "stdlib.h"
#include "math.h"
#include "dlpspec_util.h"
#include "dlpspec_types.h"

/**
 * @addtogroup group_util
 *
 * @{
 */

DLPSPEC_ERR_CODE dlpspec_util_nmToColumn(const double nm, const double *coeffs, double *column)
/**
 * Function to output compute corresponding DMD column number given a wavelength
 *
 * @param[in]   nm      wavelenght in nm
 * @param[in]   coeffs  Coefficient from wavelenght calibration
 * @param[out]  column  DMD column for which wavelength is desired
 *
 * @return      Error code
 *
 */
{
	double factor;
	DLPSPEC_ERR_CODE ret_val = (DLPSPEC_PASS);

	if ((coeffs == NULL) || (column == NULL))
		return ERR_DLPSPEC_NULL_POINTER;
    
    //First check to see if coeffs are linear (1st order). If so, return linear fit. This guards against divide by zero.
    if (0 == coeffs[2])
    {
        if (0 != coeffs[1])
        {
            *(column) = (nm - coeffs[0]) / coeffs[1];
        }
        else
        {
            ret_val = ERR_DLPSPEC_INVALID_INPUT;
        }
    }
    else
    {
    	//Compute first factor and check if the DMD column is within acceptable range, if not compute the next one
    	factor = ((-1.0 * coeffs[1]) + sqrt(coeffs[1]*coeffs[1] - 4.0 *coeffs[2]*(coeffs[0]-nm))) / (2.0*coeffs[2]);
    	if ((factor >= MIN_DMD_COLUMN) && (factor <= MAX_DMD_COLUMN))
    		*(column) = factor;
    	else
    	{
    		factor = ((-1.0 * coeffs[1]) - sqrt(coeffs[1]*coeffs[1] - 4.0 *coeffs[2]*(coeffs[0]-nm))) / (2.0*coeffs[2]);
    		if ((factor >= MIN_DMD_COLUMN) && (factor <= MAX_DMD_COLUMN))
    			*(column) = factor;
    		else
    			ret_val = ERR_DLPSPEC_FAIL;
    	}
    }

	return ret_val;
}

DLPSPEC_ERR_CODE dlpspec_util_columnToNm(const double column,  const double *coeffs, double *nm)
/**
 * Function to output compute corresponding wavelength given a DMD column number
 *
 * @param[in]   column  DMD column for which wavelength is desired
 * @param[in]   coeffs  Coefficient from wavelength calibration
 * @param[out]  nm      wavelength in nm
 *
 * @return  Error code
 */
{
	if ((coeffs == NULL) || (nm == NULL))
		return ERR_DLPSPEC_NULL_POINTER;

	*(nm) = coeffs[2] * column * column + coeffs[1] * column + coeffs[0];

	return (DLPSPEC_PASS);
}

DLPSPEC_ERR_CODE dlpspec_util_columnToNmDistance(const double column_distance,  const double *coeffs, double *nm)
/**
 * Function to output wavelength distance in nm at the center of the DMD given a DMD 
 * distance in columns
 *
 * @param[in]    column_distance DMD column distance which distance in nm is desired
 * @param[in]    coeffs          Coefficient from wavelength calibration
 * @param[out]   nm              wavelength distance in nm
 *
 * @return   Error code
 *
 */
{
	double start_nm;
	double end_nm;
	double start_px = MAX_DMD_COLUMN / 2;
	double end_px = start_px + column_distance;
    
    DLPSPEC_ERR_CODE ret_val = (DLPSPEC_PASS);

    if ((coeffs == NULL) || (nm == NULL))
		return ERR_DLPSPEC_NULL_POINTER;
    
	ret_val = dlpspec_util_columnToNm(start_px, coeffs, &start_nm);
    if (ret_val < 0)
    {
        return ret_val;
    }
	
	ret_val = dlpspec_util_columnToNm(end_px, coeffs, &end_nm);
    if (ret_val < 0)
    {
        return ret_val;
    }
    
	*nm = (int)fabs(end_nm - start_nm);
	
	return ret_val;
}

/** @} // group group_util
 *
 */
