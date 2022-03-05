/*****************************************************************************
**
**  Copyright (c) 2015 Texas Instruments Incorporated.
**
******************************************************************************
**
**  DLP Spectrum Library
**
*****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "dlpspec_calib.h"
#include "dlpspec_types.h"
#include "dlpspec_helper.h"
#include "dlpspec_util.h"
#include "dlpspec_scan.h"

/**
 * @addtogroup group_calib
 *
 * @{
 */

DLPSPEC_ERR_CODE dlpspec_calib_write_ref_matrix(const refCalMatrix *pData, 
		void *pBuf, const size_t bufSize)
{
/**
 * Serializes @p pData into @p pBuf
 * 
 * @param[in]   pData  pointer to refCalMatrix struct
 * @param[out]  pBuf   pointer to buffer where serialized #refCalMatrix will be stored
 * @param[in]   bufSize size allocated at @p pBuf for serialized #refCalMatrix
 *    
 * @return      Error code
 *
 */
  
	DLPSPEC_ERR_CODE ret_val = (DLPSPEC_PASS);
  
	if ((pData == NULL) || (pBuf == NULL))
	{
		return (ERR_DLPSPEC_NULL_POINTER); 
	}

	ret_val = dlpspec_serialize(pData, pBuf, bufSize, REF_CAL_MATRIX_TYPE);

	return ret_val;
}

DLPSPEC_ERR_CODE dlpspec_calib_findPeaks3(const double y1, const double y2, 
		const double y3, double * offset)
/**
 * @brief Interpolate peak location between discrete values.
 * 
 * Parabolic interpolation of peak location given 3 equally-spaced points. This function 
 * assumes the x distance between x1, x2, and x3 are equal. @p y2 should be greater than 
 * both @p y1 and @p y3 so that the peak is between x1 and x3. @p offset is the relative delta-x 
 * value to be added to x2 to get the interpolated x axis peak location.
 * 
 * @param[in]   y1  y coordinate of point 1 (x1, y1)
 * @param[in]   y2  y coordinate of point 2 (x2, y2)
 * @param[in]   y3  y coordinate of point 3 (x3, y3)
 *
 * @param[out]  offset  y coordinate offset which should be added to y2 to yield the interpolated peak
 *    
 * @return      Error code
 */
{
    if (((y1 - 2 * y2 + y3) != 0) && (y1 <= y2) && (y3 <= y2))
    {
        *offset = (0.5 * (y1 - y3) / (y1 - 2 * y2 + y3));
    }
	else
    {
        *offset = 0;
        return (ERR_DLPSPEC_INVALID_INPUT);
    }

    return (DLPSPEC_PASS);
}

DLPSPEC_ERR_CODE dlpspec_calib_findPeakInterp(const double x1,const double x2,const double x3,const double y1,const double y2,const double y3,double * xe,double * ye,double * d2)
/**
 * @brief Interpolate peak location between discrete values.
 * 
 * Parabolic interpolation of peak location given 3 non-equally-spaced points. This function 
 * Calculates the co-ordinates @p xe, @p ye, and the slope @p d2 of the extremum. Negative 
 * values of @p d2 indicates a valid maximum, positive values of @p d2 indicate a valid minimum.
 * @p y2 should be greater or less than both @p y1 and @p y3 so that the peak is between x1 and x3.
 * If it is not, the return value will be ERR_DLPSPEC_INVALID_INPUT.
 * 
 * @param[in]   x1  x coordinate of point 1 (x1, y1)
 * @param[in]   x2  x coordinate of point 2 (x2, y2)
 * @param[in]   x3  x coordinate of point 3 (x3, y3)
 * @param[in]   y1  y coordinate of point 1 (x1, y1)
 * @param[in]   y2  y coordinate of point 2 (x2, y2)
 * @param[in]   y3  y coordinate of point 3 (x3, y3)
 *
 * @param[out]  xe  x coordinate of extrema found
 * @param[out]  ye  y coordinate of extrema found
 * @param[out]  d2  2nd derivative at extrema found
 *    
 * @return      Error code
 */

{ 
  double d1;
  
  //
  if ((x3-x2) == 0 || (x1-x2) == 0 || (x3-x1) == 0)
	  return ERR_DLPSPEC_INVALID_INPUT;
  
  *d2 = 2*((y3-y2)/(x3-x2)-(y1-y2)/(x1-x2))/(x3-x1);
  d1 = (y3-y2)/(x3-x2) - 0.5 * *d2 * (x3-x2);
  
  if (*d2) {
    *xe = x2 - d1 / *d2;
    *ye = y2 + 0.5*d1*(*xe-x2);
  } else { // curve is a straight line, with no local max or min
    return ERR_DLPSPEC_INVALID_INPUT;
  } 
  
  if ((*xe > x3)||(*xe < x1))
	  return ERR_DLPSPEC_INVALID_INPUT; // Peak was not between extremes passed in 
  else
	  return DLPSPEC_PASS;
} 

DLPSPEC_ERR_CODE dlpspec_calib_getFWHM(const double *values, const int num_values, 
		const int peak_location, int * peak_height, double * fwhm)
/**
 * Finds and returns the FWHM (Full Width Half Max) and peak height given peak location and set of values.
 *
 * @param[in]   values          Pointer to the buffer that contains the computed spectrum values.
 * @param[in]   num_values      Number of values present in values buffer.
 * @param[in]   peak_location   Index in values array where a peak is located whose FWHM needs to be determined.
 * @param[out]  peak_height     Return for mean peak height
 * @param[out]  fwhm            Return for fwhm, in pixels.
 *  
 * @return  ≤0     FWHM in units of number of indices in the values array
 * @return  <0     Error codes
 *
 */
{
    double half_width1, half_width2;
    double base_height1=0.0, base_height2=0.0;
    int i;
    double last_height, target_height;
    
    if ((values == NULL) || (peak_height == NULL))
        return (ERR_DLPSPEC_NULL_POINTER);


    if ((num_values < 0) || (peak_location < 0) || (peak_location >= num_values))
    	return (ERR_DLPSPEC_INVALID_INPUT);

    //Find left height of peak
    last_height = values[peak_location] + 1;
    for(i=peak_location; i>0; i--)
    {
        if(values[i] < last_height)
        {
            last_height = values[i];
            base_height1 = values[peak_location] - last_height;
        }
        else
        {
            break;
        }
    }

    //Find right height of peak
    last_height = values[peak_location] + 1;
    for(i=peak_location; i<num_values; i++)
    {
        if(values[i] < last_height)
        {
            last_height = values[i];
            base_height2 = values[peak_location] - last_height;
        }
        else
        {
            break;
        }
    }
    
    //Compute mean height of peak
    *peak_height = (base_height1 + base_height2) / 2;

    half_width1 = peak_location; //worst case if we never found half max
    target_height = values[peak_location] - (*peak_height / 2);
    
    //Find half max location to the left of the peak
    for(i=peak_location; i>=0; i--)
    {
        if(values[i] < target_height)
        {
            //Linear interpolation of half-max horizontal distance
            half_width1 = (double)peak_location - i - ((target_height - values[i])
				   	/ (values[i+1] - values[i]));
            break;
        }
    }

    half_width2 = num_values -1 - peak_location; //worst case if we never found half max
    
    //Find half max location to the right of the peak
    for(i=peak_location; i<num_values; i++)
    {
        if(values[i] < target_height)
        {
            //Linear interpolation of half-max horizontal distance
            half_width2 = (double)i - peak_location - ((target_height - values[i])
				   	/ (values[i-1] - values[i]));
            break;
        }
    }

    *fwhm = (half_width2 + half_width1);
    return (DLPSPEC_PASS);

}

DLPSPEC_ERR_CODE dlpspec_calib_get_halfmax_loc(double *values, int num_values, 
		int peak_location, double *left_halfmax_loc, double *right_halfmax_loc)
/**
 * @brief   Finds and returns the FWHM (Full Width Half Max) and peak height given peak location and set of values.
 *
 * @param[in]   values          Pointer to the buffer that contains the computed absorption spectrum values.
 * @param[in]   num_values      Number of values present in values buffer.
 * @param[in]   peak_location   Index in values array where a peak is located whose FWHM needs to be determined.
 * 
 * @param[out]  left_halfmax_loc    location of the point to the left of peak where height drops to half of the peak
 * @param[out]  right_halfmax_loc   location of the point to the right of peak where height drops to half of the peak
 *    
 * @retval   Error codes
 *
 */
{
    double base_height_left=0.0, base_height_right=0.0;
    int i;
    double last_height, target_height;
    
    if ((values == NULL) || (left_halfmax_loc == NULL) || (right_halfmax_loc == NULL))
        return (ERR_DLPSPEC_NULL_POINTER);

    if ((num_values < 0) || (peak_location < 0) || (peak_location >= num_values))
    	return (ERR_DLPSPEC_INVALID_INPUT);

    //Find left height of peak
    last_height = values[peak_location] + 1;
    for(i=peak_location; i>0; i--)
    {
        if(values[i] < last_height)
        {
            last_height = values[i];
            base_height_left = values[peak_location] - last_height;
        }
        else
        {
            break;
        }
    }

    //Find right height of peak
    last_height = values[peak_location] + 1;
    for(i=peak_location; i<num_values; i++)
    {
        if(values[i] < last_height)
        {
            last_height = values[i];
            base_height_right = values[peak_location] - last_height;
        }
        else
        {
            break;
        }
    }
    
    *left_halfmax_loc = peak_location; //worst case if we never found half max
    target_height = values[peak_location] - (base_height_left / 2);
    
    //Find half max location to the left of the peak
    for(i=peak_location; i>=0; i--)
    {
        if(values[i] < target_height)
        {
            //Linear interpolation of half-max horizontal distance
            *left_halfmax_loc = (double)i + ((target_height - values[i]) / 
					(values[i+1] - values[i]));
            break;
        }
    }

    *right_halfmax_loc = num_values -1; //worst case if we never found half max
    target_height = values[peak_location] - (base_height_right / 2);
    
    //Find half max location to the right of the peak
    for(i=peak_location; i<num_values; i++)
    {
        if(values[i] < target_height)
        {
            //Linear interpolation of half-max horizontal distance
            *right_halfmax_loc = (double)i - ((target_height - values[i]) / 
					(values[i-1] - values[i]));
            break;
        }
    }

	return (DLPSPEC_PASS);

}

int32_t dlpspec_calib_findPeaks(const double *values, const int num_values, 
		const double peak_sel_divisor, int *peak_inds)
/**
 * Finds and returns the peak values and the locations of the peak for a given set of values.
 *
 * @param[in]   values              Pointer to the buffer that contains the computed absorption spectrum values.
 * @param[in]   num_values          Number of values present in values buffer.
 * @param[in]   peak_sel_divisor    This input will decide the criteria to select peaks. Values that are higher than the preceding dip/valley by
 *                                  an amount >= (max-min)/peak_sel_divisor will be counted as a peak.
 * @param[out]  peak_inds           Location/index/position of the Peak values in the absorption spectrum file input that matches the condition above.
 *
 * @return  number of peaks found in the input data set
 * @return  <0 = Error codes as #DLPSPEC_ERR_CODE
 *
 */
{
    double *diff_vals=NULL;
    double *peak_vals=NULL;
    int i;
    int j;
    int *peak_valley_indices=NULL;
    int *peak_locs=NULL;
    int num_peaks_vallies;
    double *peaks_vallies=NULL;
    double min_val;
    double max_val;
    double sel;
    int ret_val=0;
    int start_index;
    double left_min;
    double temp_val;
    int temp_loc=0;
    int max_peaks;
    int found_peak;
    int peak_index;

    if ((values == NULL) || (peak_inds == NULL))
    {
        return ERR_DLPSPEC_NULL_POINTER;
    }

    if ((peak_sel_divisor < 0.0) || (num_values < 1))
    {
        return ERR_DLPSPEC_INVALID_INPUT;
    }

    diff_vals = (double *)malloc((num_values-1)*sizeof(double));
    peak_valley_indices = (int *)malloc(num_values * sizeof(int));
    peaks_vallies = (double *)malloc(num_values * sizeof(double));

    if(diff_vals == NULL || peaks_vallies == NULL || peak_valley_indices == NULL)
    {
        ret_val = ERR_DLPSPEC_INSUFFICIENT_MEM;
        goto cleanup_and_exit;
    }

    //Find derivatives or diffs
    for(i=0; i < num_values-1; i++)
    {
        diff_vals[i] = values[i+1] - values[i];
    }

    //Find indices where derivate changes sign - these are our potential peaks and valleys
    j=0;
    /* Include end points in potential peaks and vallies */
    //disabling end points because last pattern (910) was commonly causing a peak
    //peak_valley_indices[j++] = 0;
    num_peaks_vallies = 0;
    for(i=1; i<num_values-1; i++)
    {
        if(SIGN(diff_vals[i]) != SIGN(diff_vals[i-1]))
        {
            peak_valley_indices[j++] = i;
        }
    }

    /* Include end points in potential peaks and vallies */
    //disabling end points because last pattern (910) was commonly causing a peak
    //peak_valley_indices[j++] = num_values-1;
    //num_peaks_vallies = j;
    num_peaks_vallies = j-1;
    if(num_peaks_vallies <= 2)
    {
        ret_val = ERR_DLPSPEC_FAIL;
        goto cleanup_and_exit;
    }
    min_val = values[0];
    max_val = values[0];
    for(i=0; i<num_peaks_vallies; i++)
    {
        peaks_vallies[i] = values[peak_valley_indices[i]];
        if(peaks_vallies[i] < min_val)
            min_val = peaks_vallies[i];
        if(peaks_vallies[i] > max_val)
            max_val = peaks_vallies[i];
    }

    if(peak_sel_divisor)    //avoid division by zero
        sel = (max_val - min_val)/peak_sel_divisor;
    else
        sel = (max_val - min_val);

    /* Deal with first point - since we just took it, it may not alternate 
	 * like the rest*/
    start_index=0;
    /* if first value is larger than second and second is larger than third, 
	 * we can remove the second point from potential peaks */
    if( peaks_vallies[0] >= peaks_vallies[1] )
    {
        if(peaks_vallies[1] >= peaks_vallies[2])
        {
            peaks_vallies[1] = peaks_vallies[0];
            start_index = 1;
        }
    }
    else
    {
        /* if first value is smaller than second and second is smaller than 
		 * third, we can remove the first two points from potential peaks */
        if( peaks_vallies[1] < peaks_vallies[2] )
            start_index = 2;
        else
            start_index = 1;
    }

    /* Initialize loop variables */
#if 0
    DEBUG_MSG("num_peaks_vallies = %d\n", num_peaks_vallies);
    DEBUG_MSG("we are starting at the peak at index %d\n", start_index);
    DEBUG_MSG("Selectivity divisor = %d\n", peak_sel_divisor);
#endif
    //max_peaks = (num_peaks_vallies-start_index+1)/2;
    max_peaks = num_peaks_vallies;
    peak_locs = (int *)malloc(max_peaks*sizeof(int));
    peak_vals = (double *)malloc(max_peaks*sizeof(double));
    if(peak_locs == NULL || peak_vals == NULL)
    {
        ret_val = ERR_DLPSPEC_INSUFFICIENT_MEM;
        goto cleanup_and_exit;
    }
    found_peak = false;
    temp_val = min_val;
    left_min = min_val;
    j=0;

    for(i=start_index; i<num_peaks_vallies-1; i++)
    {
        /* i is at a peak */
        /* Reset peak finding if we had a peak and next peak is larger than this or if the left min was small enough */
        if(found_peak)
        {
            temp_val = min_val;
            found_peak = false;
        }

        if((peaks_vallies[i] > temp_val) && (peaks_vallies[i] > left_min + sel))
        {
            temp_loc = i;
            temp_val = peaks_vallies[i];
        }

        i++; //Move on to the valley
        if(i == num_peaks_vallies-1)
            break; //Make sure we don't read past the array

        /* down at least sel from peak */
        if((found_peak == false) && (temp_val > peaks_vallies[i] + sel))
        {
            found_peak = true;
            left_min = peaks_vallies[i];
            peak_locs[j] = temp_loc; //Add peak to index
            peak_vals[j] = temp_val;
            j++;
        }
        else if(peaks_vallies[i] < left_min) //new left min
            left_min = peaks_vallies[i];
    }

    //Handle last point
    if((peaks_vallies[num_peaks_vallies-1] > temp_val) && 
			(peaks_vallies[num_peaks_vallies-1] > left_min + sel))
    {
        peak_locs[j] = num_peaks_vallies-1;
        peak_vals[j] = peaks_vallies[num_peaks_vallies-1];
        j++;
    }
    else if((found_peak == false) && (temp_val > min_val))
    {
        peak_locs[j] = temp_loc;
        peak_vals[j] = temp_val;
        j++;
    }

    for(i=0; i<j; i++)
    {
        peak_index = peak_valley_indices[peak_locs[i]];
        peak_inds[i] = peak_index;
    }
    ret_val = j;

cleanup_and_exit:
    if(diff_vals != NULL)
        free(diff_vals);
    if(peak_valley_indices != NULL)
        free(peak_valley_indices);
    if(peaks_vallies != NULL)
        free(peaks_vallies);
    if(peak_locs != NULL)
        free(peak_locs);
    if(peak_vals != NULL)
        free(peak_vals);
    return ret_val;
}

#ifndef _WIN32
DLPSPEC_ERR_CODE dlpspec_calib_checkPeakDist(const double *peak_pos, 
		const int num_values, const double *ref_pos, const double tolerance)
/**
 * Checks if the peaks conforms with the expected distribution and returns the result
 *
 * @param[in]   peak_pos    Pointer to the buffer that contains peak positions
 * @param[in]   num_values 	Number of values present in values buffer
 * @param[in]   ref_pos 	Pointer to the buffer that contains reference peak positions; please ensure that there are as many ref pos as peak_pos
 * @param[in]   tolerance 	Tolerance to be used while checking relative positions
 *
 * @return  Error codes
 *
 */
{
	const double *ref_pos_vals;
	DLPSPEC_ERR_CODE ret_val = (DLPSPEC_PASS);
	int i = 0;

    /* Default values for NIRscan Nano EVM only */
    double refPeakPos[] = {452, 516, 560,723,784,824};
    int numRefPeakPos = sizeof(refPeakPos)/sizeof(double);

    if (peak_pos == NULL)
        return ERR_DLPSPEC_NULL_POINTER;

	if ((num_values <= 2) || ((ref_pos == NULL) && (num_values != numRefPeakPos)))
		return ERR_DLPSPEC_INVALID_INPUT;

	// check input reference data and used the hard-coded values when not present
	if (ref_pos == NULL)
		ref_pos_vals = refPeakPos;
	else
		ref_pos_vals = ref_pos;

	for (i=2; i < num_values;i++)
	{
		double peak_pos_temp = 0.0;
		if (peak_pos[i-1] != peak_pos[i-2])
			peak_pos_temp = (double)(peak_pos[i]-peak_pos[i-1])/
				(double)(peak_pos[i-1]-peak_pos[i-2]);
		double ref_pos_temp = 0.0;
		if ((ref_pos_vals[i-1] != ref_pos_vals[i-2]))
			ref_pos_temp = (double)(ref_pos_vals[i]-ref_pos_vals[i-1])/
				(double)(ref_pos_vals[i-1]-ref_pos_vals[i-2]);

		if ((peak_pos_temp/ref_pos_temp >= (1 - tolerance)) &&
			(peak_pos_temp/ref_pos_temp <= (1 + tolerance)))
		{
			continue;
		}
		else
		{
			ret_val = ERR_DLPSPEC_FAIL;
			break;
		}
	}

	return ret_val;
}
#endif


DLPSPEC_ERR_CODE dlpspec_calib_genPxToPyCoeffs(const int num_peaks, 
		const double *px_measured, const double *py_measured, 
		double*px_to_py_coeffs, double *rsquared)
/**
 * Finds a second order polynomial that fits the given x and y input values and returs the three co-efficients for that polynomial
 *
 * @param[in]   num_peaks           Number of points in @p *px_measured and @p *py_measured
 * @param[in]   px_measured         Pointer to the pixel peak locations. This should be 
 * @param[in]   py_measured         Pointer to the 6 nanometer values
 * @param[out]  px_to_py_coeffs     Pointer to the computed polynomial coefficients in the following order c, b, a which should be used to compute any given y value as
 *                                  y = ax2 + bx + c
 * @param[out]  rsquared            Pointer to r squared coefficient of determination for the derrived 2nd order equation.
 *    
 * @return  number of peaks found in the input data set
 *          <0  Error codes as DLPSPEC_ERR_CODE
 *
 */
{
  DLPSPEC_ERR_CODE ret_val = DLPSPEC_PASS;
  double *x_trans,*res, *x;
  double nm;
  double ss = 0;
  double ss_term = 0;
  double rss = 0;
  double rss_term = 0;
  double ymean = 0;
  double a_inv[PX_TO_LAMBDA_NUM_POL_COEFF][PX_TO_LAMBDA_NUM_POL_COEFF], a[PX_TO_LAMBDA_NUM_POL_COEFF][PX_TO_LAMBDA_NUM_POL_COEFF];
  int i,j;
  double determinant=0;

  if ((px_measured == NULL) || (py_measured == NULL) || (px_to_py_coeffs == NULL))
      return (ERR_DLPSPEC_NULL_POINTER);

  if ((num_peaks <= 0))
	  return(ERR_DLPSPEC_INVALID_INPUT);

  x_trans = (double *)malloc(PX_TO_LAMBDA_NUM_POL_COEFF*num_peaks*sizeof(double));
  res = (double *)malloc(PX_TO_LAMBDA_NUM_POL_COEFF*num_peaks*sizeof(double));
  x = (double *)malloc(PX_TO_LAMBDA_NUM_POL_COEFF*num_peaks*sizeof(double));

  if ((x_trans == NULL) || (res == NULL) || (x == NULL))
  {
    ret_val = ERR_DLPSPEC_INSUFFICIENT_MEM;
    goto cleanup_and_exit;
  }

  //Create X matrix
  for(i=0;i<num_peaks;i++)
  {
    double val = 1;
    for(j=0;j<PX_TO_LAMBDA_NUM_POL_COEFF;j++)
    {
      //int offset = (i == 0) ? 0 : (i-1);
      x[i * PX_TO_LAMBDA_NUM_POL_COEFF + j] = val;
      val = val * px_measured[i];
    }
  }
  
  //get x transpose
  ret_val = dlpspec_matrix_transpose(x,x_trans,num_peaks,PX_TO_LAMBDA_NUM_POL_COEFF);
  if (ret_val < 0)
  {
    goto cleanup_and_exit;
  }

  //a = x_trans * x
  ret_val = dlpspec_matrix_mult(x_trans,x,&a[0][0],PX_TO_LAMBDA_NUM_POL_COEFF,
		  num_peaks,PX_TO_LAMBDA_NUM_POL_COEFF);
  if (ret_val < 0)
  {
    goto cleanup_and_exit;
  }

  //Get inverse of (X_trans*X) matrix
  determinant = a[0][0]*((a[1][1]*a[2][2]) - (a[2][1]*a[1][2])) -a[0][1]*(a[1][0]*a[2][2] - a[2][0]*a[1][2]) + a[0][2]*(a[1][0]*a[2][1] - a[2][0]*a[1][1]);
  if (0 == determinant)
  {
      ret_val = ERR_DLPSPEC_INVALID_INPUT;
      goto cleanup_and_exit;
  }
  for(j=0;j<PX_TO_LAMBDA_NUM_POL_COEFF;j++){
    for(i=0;i<PX_TO_LAMBDA_NUM_POL_COEFF;i++)
      a_inv[i][j] = ((a[(i+1)%PX_TO_LAMBDA_NUM_POL_COEFF][(j+1)%PX_TO_LAMBDA_NUM_POL_COEFF] * a[(i+2)%PX_TO_LAMBDA_NUM_POL_COEFF][(j+2)%PX_TO_LAMBDA_NUM_POL_COEFF])
                     - (a[(i+1)%PX_TO_LAMBDA_NUM_POL_COEFF][(j+2)%PX_TO_LAMBDA_NUM_POL_COEFF]*a[(i+2)%PX_TO_LAMBDA_NUM_POL_COEFF][(j+1)%PX_TO_LAMBDA_NUM_POL_COEFF]))
                     / determinant;
  }

  ret_val = dlpspec_matrix_mult(&a_inv[0][0], x_trans, res, 
		  PX_TO_LAMBDA_NUM_POL_COEFF, PX_TO_LAMBDA_NUM_POL_COEFF, num_peaks);
  if (ret_val < 0)
  {
    goto cleanup_and_exit;
  }

  //Get beta matrix
  ret_val = dlpspec_matrix_mult(res, py_measured, px_to_py_coeffs, 
		  PX_TO_LAMBDA_NUM_POL_COEFF, num_peaks, 1);
  if (ret_val < 0)
  {
    goto cleanup_and_exit;
  }
  
  //Compute rsquared
  for(i=0;i<num_peaks;i++)
  {
    ymean += py_measured[i];
  }
  ymean = ymean / num_peaks;
  for(i=0;i<num_peaks;i++)
  {
    ss_term = py_measured[i] - ymean;
    ss += ss_term * ss_term;
    
    ret_val = dlpspec_util_columnToNm(px_measured[i], px_to_py_coeffs, &nm);
    if (ret_val < 0)
    {
        goto cleanup_and_exit;
    }
    
    rss_term = py_measured[i] - nm;
    rss += rss_term * rss_term;
  }
  if (ss != 0)
  {
      *rsquared = 1 - (rss / ss);
  }
  else
  {
      ret_val = ERR_DLPSPEC_INVALID_INPUT;
      goto cleanup_and_exit;
  }

cleanup_and_exit:
  if (x_trans != NULL)
    free(x_trans);
  if (res != NULL)
    free(res);
  if (x != NULL)
    free(x);

  return ret_val;
}

DLPSPEC_ERR_CODE find_mean_and_standard_deviation(const double* data, 
		const int data_count , double* average , double* std)
/**
 * Finds mean and standard deviation of @p *data
 *
 * @param[in]   data        Pointer to input data points array
 * @param[in]   data_count  Number of data points
 * @param[out]  average     Pointer to the computed average
 * @param[out]  std         Pointer to the computed standard deviation
 *
 * @return  Error codes
 *
 */
{
    double mean=0.0, sum_deviation=0.0;
    int i;
    
    if (data_count <= 0)
  	    return(ERR_DLPSPEC_INVALID_INPUT);

   for(i=0; i<data_count;++i)
    {
        mean+=data[i];
    }
    mean=mean/data_count;
    *average = mean;

    for(i=0; i<data_count;++i)
        sum_deviation+=(data[i]-mean)*(data[i]-mean);

    *std = sqrt(sum_deviation/data_count);

    return (DLPSPEC_PASS);
}

DLPSPEC_ERR_CODE dlpspec_calib_genPxyToCurveCoeffs(const double *peaks, 
		const double *y_values,  const int num_peaks, const int num_measurements,
	   	double *pxy_to_curve_coeffs)
/**
 * This function computes the polynomial coefficients using DMD row used for measurement and corresponding peak location
 * These coefficients can then be used to compute the shift during pattern generation
 *
 * @param[in]   peaks               Pointer to the pixel peak locations on top, middle & bottom of DMD (ordered with peaks closest to the centre column of DMD first and the farthest from centre as the last)
 * @param[in]   num_measurements    Number of measurements taken; typically 3, one on top, middle and bottom of DMD
 * @param[in]   num_peaks           number of peaks measured
 * @param[in]   y_values            y values correspoding to each set of measurements
 * @param[out]  pxy_to_curve_coeffs Pointer to the computed polynomial coefficients in the following order c, b, a which
 *                                  should be used to compute any given y value as y = ax2 + bx + c
 *
 * @return  Error codes
 */
{
  int i=0, j=0;
  DLPSPEC_ERR_CODE ret_val = (DLPSPEC_PASS);
  double *temp_peaks,*temp_coeffs;
  double *a_coeff_array;
  double *b_coeff_array;
  double a_mean;
  double b_mean;
  double a_std_dev;
  double b_std_dev;
  double a_diff, b_diff;
  double rSquared;

  if ((peaks == NULL) || (y_values == NULL) || (pxy_to_curve_coeffs == NULL))
      return ERR_DLPSPEC_NULL_POINTER;

  if ((num_measurements <= 0) || (num_peaks <= 2))
    return ERR_DLPSPEC_INVALID_INPUT;

  temp_peaks = (double *)malloc(num_measurements*sizeof(double));
  temp_coeffs = (double *)malloc(PX_TO_LAMBDA_NUM_POL_COEFF*sizeof(double)*num_peaks);
  a_coeff_array = (double *)malloc(sizeof(double)*num_peaks);
  b_coeff_array = (double *)malloc(sizeof(double)*num_peaks);

  if ((temp_peaks == NULL) || (temp_coeffs == NULL) || (a_coeff_array == NULL)
		  || (b_coeff_array == NULL))
  {
    ret_val = ERR_DLPSPEC_INSUFFICIENT_MEM;
    goto cleanup_and_exit;
  }

  for(i=0;i<num_peaks;i++)
  {

      //Create array of relative peak positions
      for(j=0;j<num_measurements;j++)
      {
          temp_peaks[j] = (double)(*(peaks + i + j * num_peaks));
      }

      ret_val = dlpspec_calib_genPxToPyCoeffs(num_measurements, y_values, 
			  temp_peaks, &temp_coeffs[PX_TO_LAMBDA_NUM_POL_COEFF*i], &rSquared);
      if (ret_val < 0)
      {
          goto cleanup_and_exit;
      }

  }

  /* Peaks must be arranged in the ascending order of their distance from the 
   * centre of the DMD; so pick the first set of peaks and related coefficients
   *  if those coefficients does not fall outside of the standard deviation of
   *  the set */
  for(i=0;i<num_peaks;i++)
  {
      b_coeff_array[i] = temp_coeffs[i*PX_TO_LAMBDA_NUM_POL_COEFF+1];
      a_coeff_array[i] = temp_coeffs[i*PX_TO_LAMBDA_NUM_POL_COEFF+2];
  }

  ret_val = find_mean_and_standard_deviation(a_coeff_array, num_peaks, &a_mean,
		  &a_std_dev);
  if (ret_val < 0)
  {
      goto cleanup_and_exit;
  }
  
  ret_val = find_mean_and_standard_deviation(b_coeff_array, num_peaks, &b_mean,
		  &b_std_dev);
  if (ret_val < 0)
  {
      goto cleanup_and_exit;
  }

  for(i=0; i<num_peaks; i++)
  {

      a_diff = a_coeff_array[i] - a_mean;
      b_diff = b_coeff_array[i] - b_mean;

      if(a_diff < 0)
          a_diff = 0-a_diff;

      if(b_diff < 0)
          b_diff = 0-b_diff;

      if((a_diff < a_std_dev) && (b_diff < b_std_dev))
      {
            pxy_to_curve_coeffs[0] = temp_coeffs[i*PX_TO_LAMBDA_NUM_POL_COEFF] -
			   	peaks[num_peaks+i];
            pxy_to_curve_coeffs[1] = temp_coeffs[i*PX_TO_LAMBDA_NUM_POL_COEFF+1];
            pxy_to_curve_coeffs[2] = temp_coeffs[i*PX_TO_LAMBDA_NUM_POL_COEFF+2];
            break;
      }
  }

cleanup_and_exit:
  if(temp_peaks != NULL)
    free(temp_peaks);
  if(temp_coeffs != NULL)
    free(temp_coeffs);
  if(a_coeff_array != NULL)
    free(a_coeff_array);
  if(b_coeff_array != NULL)
    free(b_coeff_array);

  return ret_val;

}

DLPSPEC_ERR_CODE dlpspec_calib_genShiftVector(const double *pol_coeffs, 
		const int dmd_height, int8_t *shift_vector)
/**
 * Generates Shift vector for each row in DMD. Values indicate the required ***left*** shift for a given DMD row
 *
 * @param[in]   pol_coeffs      Pointer to polynomial coefficients in the following order c, b, a which
 *                              should be used to compute any given y value as y = ax2 + bx + c
 * @param[in]   dmd_height      Height of DMD being used
 * @param[out]  shift_vector    Pointer to the computed shift_vector from top to bottom of DMD. Values indicate required left shift. Length should be at least @p dmd_height.
 *
 * @return  Error codes
 */
{
  DLPSPEC_ERR_CODE ret_val = (DLPSPEC_PASS);
  int i=0;
  double shift_val;

  if ((pol_coeffs == NULL) || (shift_vector == NULL))
      return ERR_DLPSPEC_NULL_POINTER;

  if ((dmd_height <= 0))
    return ERR_DLPSPEC_INVALID_INPUT;

  for(i=0;i<dmd_height;i++)
  {
	  shift_val = (pol_coeffs[2] * i * i +
                      pol_coeffs[1] * i +
                      pol_coeffs[0]);

      shift_vector[i] = round(shift_val);
  }

  return ret_val;
}


int32_t dlpspec_calib_genPatterns(const CALIB_SCAN_TYPES scan_type, 
		FrameBufferDescriptor *pFB)
/**
 * Generates patterns for calibration scans
 *
 * @param[in]   scan_type   Calibration scan type
 * @param[out]  pFB         Pointer to frame buffer to fill with patterns
 * 
 * @return  >0	Num Patterns
 * @return  <0	Error codes as #DLPSPEC_ERR_CODE
 * 
 */
{
	int x;
	RectangleDescriptor rect;
	int numPatterns=0;
	int patterns_per_image;
    uint32_t num_buffers=0;
	int frameBufferSz = 0;
	int step;
	int width_px, start_px, end_px;

	if (pFB == NULL)
		return (ERR_DLPSPEC_NULL_POINTER);

	frameBufferSz = (pFB->width * pFB->height * (pFB->bpp/8));

	if(pFB->bpp == 16)
		patterns_per_image=16;
	else
		patterns_per_image=24;

	switch(scan_type)
	{
		case SLIT_ALIGN_SCAN:
			start_px = 0;
			end_px = 853;
			width_px = 6;
			step = 4;
			break;
		case DET_ALIGN_SCAN:
			start_px = 0;
			end_px = 853;
			width_px = 255;
			step = 284;
			break;
		case LEFT_DMD_SCAN:
		case LEFT_DMD_TOP_SCAN:
		case LEFT_DMD_MID_SCAN:
		case LEFT_DMD_BOT_SCAN:
			start_px = WAVELEN_CAL_PTN_CENTER_OFFSET - ((WAVELEN_CAL_PTN_WIDTH - 1)/2);
			end_px = 430;
			width_px = WAVELEN_CAL_PTN_WIDTH;
			step = 1;
			break;
		case RIGHT_DMD_SCAN:
		case RIGHT_DMD_TOP_SCAN:
		case RIGHT_DMD_MID_SCAN:
		case RIGHT_DMD_BOT_SCAN:
			start_px = 430-WAVELEN_CAL_PTN_WIDTH+2;
			end_px = MAX_DMD_COLUMN;
			width_px = WAVELEN_CAL_PTN_WIDTH;
			step = 1;
			break;
		default:
			return (ERR_DLPSPEC_ILLEGAL_SCAN_TYPE);
	}

	
	for(x=start_px; x <= end_px+1-width_px; x+=step)
	{
		if(numPatterns % patterns_per_image == 0)
		{
			//First clear the area of interest
			rect.startX = 0;
			rect.startY = 0;
			rect.height = pFB->height;
			rect.width = pFB->width;
			rect.pixelVal = 0;
			DrawRectangle(&rect, pFB, true);
		}

		rect.startX = x;
		rect.startY = 0;
		rect.height = pFB->height;
		rect.width = width_px;
		rect.pixelVal = 1 << (numPatterns%patterns_per_image);

		DrawRectangle(&rect, pFB, false);
		numPatterns++;
		if(numPatterns % patterns_per_image == 0)
		{
			//Advance frame buffer pointer
			pFB->frameBuffer += frameBufferSz/4;
			num_buffers++;
			if(num_buffers == pFB->numFBs)
				break;
		}
	}
	
	return numPatterns;
}


DLPSPEC_ERR_CODE dlpspec_calib_write_data(const calibCoeffs *pCoeffs, void *pBuf,
	   	const size_t bufSize)
/**
 * Writes calibration coefficients from @p *pCoeffs to serialized data in @p *pBuf
 *
 * @param[in]   pCoeffs     Calibration coefficients struct to serialize
 * @param[out]  pBuf        Buffer to store serialized struct in
 * @param[in]   bufSize     Size of @p pBuf, in bytes
 * 
 * @return  Error codes
 */
{
    DLPSPEC_ERR_CODE ret_val = (DLPSPEC_PASS);
    
	if ((pCoeffs == NULL) || (pBuf == NULL))
		return (ERR_DLPSPEC_NULL_POINTER);

	ret_val = dlpspec_serialize(pCoeffs, pBuf, bufSize, CALIB_TYPE);

	return ret_val;
}


DLPSPEC_ERR_CODE dlpspec_calib_read_data(void *pBuf, size_t bufSize)
/**
 * Read calibration coefficients from a serialized data blob in place. Upon successful return, @p *pBuf will be a deserialized #calibCoeffs struct.
 *
 * @param[in,out]   pBuf    Serialized #calibCoeffs data blob, previously serilized with dlpspec_calib_write_data()
 * @param[in]       bufSize Size of @p *pBuf, in bytes
 * 
 * @return  Error codes
 */
{
    DLPSPEC_ERR_CODE ret_val = (DLPSPEC_PASS);
    	
    if (pBuf == NULL)
		return (ERR_DLPSPEC_NULL_POINTER);

	ret_val = dlpspec_deserialize(pBuf, bufSize, CALIB_TYPE);

	return ret_val;
}

DLPSPEC_ERR_CODE dlpspec_calib_interpret(const void *pBuf, const size_t bufSize, scanResults *pResults, const CALIB_SCAN_TYPES type)
/**
 * Interpret serialized #scanData serialized blob which was taken by a calibration scan
 *
 * @param[in]   pBuf        Serialized #scanData data blob
 * @param[in]   bufSize     Size of @p *pBuf, in bytes
 * @param[out]  pResults    Pointer where function will store interpreted #scanResults
 * @param[in]   type        Type of calibration scan this data blob refers to, as defined in #CALIB_SCAN_TYPES
 * 
 * @return  Error codes
 */
{
	int i;
	int offset = 0;
	int width_px = 5;
	int step = 1;
	scanData *pScanData;
	void *pCopyBuff = NULL;
 
    DLPSPEC_ERR_CODE ret_val = (DLPSPEC_PASS);    

	if ((pBuf == NULL) || (pResults == NULL))
		return (ERR_DLPSPEC_NULL_POINTER);

	pCopyBuff = (void *)malloc(bufSize);

	if(pCopyBuff == NULL)
		return (ERR_DLPSPEC_INSUFFICIENT_MEM);

	memcpy(pCopyBuff, pBuf, bufSize);

	ret_val = dlpspec_deserialize(pCopyBuff, bufSize, SCAN_DATA_TYPE);
    if (ret_val < 0)
    {
        return ret_val;
    }
    
	pScanData = (scanData *)pCopyBuff;
    /*
    Procedure:
    1. Compute DC detector level during black patterns (every 25th, as defined by sequence, #def.)
    2. Subtract found DC detector level from remaining measurements.
    3. Compute wavelength centers for the scanData configuration, using genPatDef
    4. Copy computed wavelengths, intensities, and header info from scanData to scanResults.
    */

	if (type == RIGHT_DMD_SCAN)
		offset = 430-5+2;

	dlpspec_copy_scanData_hdr_to_scanResults((const uScanData *)pScanData, pResults);
	dlpspec_subtract_remove_dc_level(pScanData, pResults);

	/* Other calib scans are done before calibCoeffs are computed 
	 * so the computation of wavelength is not possible for those */
    //if((type == LEFT_DMD_SCAN) || (type == RIGHT_DMD_SCAN))
	{
		for(i=0; i<pResults->length; i++)
		{
            if((type == LEFT_DMD_SCAN) || (type == RIGHT_DMD_SCAN))
            {
                ret_val = dlpspec_util_columnToNm((double)(offset + i*step + 
							width_px/2), 
						&(pScanData->calibration_coeffs.PixelToWavelengthCoeffs[0]), 
						&pResults->wavelength[i]);
                if (ret_val < 0)
                    goto cleanup_and_exit;
            }
            else
               pResults->wavelength[i] = offset + i;
		}
	}

    cleanup_and_exit:
    if(pCopyBuff != NULL)
	    free(pCopyBuff);

	return ret_val;
}

/** @} // group group_calib
 *
 */
