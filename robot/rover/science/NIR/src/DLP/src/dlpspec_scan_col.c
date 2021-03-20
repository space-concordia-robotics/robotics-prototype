/*****************************************************************************
**
**  Copyright (c) 2015 Texas Instruments Incorporated.
**
******************************************************************************
**
**  DLP Spectrum Library
**
*****************************************************************************/

#include <string.h>
#include <stdint.h>
#include "dlpspec_scan_col.h"
#include "dlpspec_util.h"
#include "dlpspec_helper.h"

/**
 * @addtogroup group_scan_col
 *
 * @{
 */

int32_t dlpspec_scan_col_genPatterns(const patDefCol *patDefCol,
	   	const FrameBufferDescriptor *pFB, uint32_t startPattern)
/**
 * @brief Function to generate patterns for a column scan.
 *
 * This function takes the column pattern definition an writes the described
 * patterns to the frame buffer described in the frame buffer descriptor.
 *
 * @param[in]   patDefCol		Pointer to column pattern definition
 * @param[in]	pFB				Pointer to frame buffer descriptor where the 
 *								patterns will be stored
 * @param[in]   startPattern	Pattern number at which to start drawing
 *
 * @return  >0  Number of binary patterns generated from the pattern definition
 * @return  ≤0  Error code as #DLPSPEC_ERR_CODE
 */
{
	int i;
	RectangleDescriptor rect;
	int curPattern;
	int patterns_per_image;
    uint32_t curBuffer=0;
	int frameBufferSz = (pFB->width * pFB->height * (pFB->bpp/8));
	FrameBufferDescriptor frameBuffer;
    
	if ((patDefCol == NULL) || (pFB == NULL))
		return (ERR_DLPSPEC_NULL_POINTER);

	memcpy(&frameBuffer, pFB, sizeof(FrameBufferDescriptor));

	if(frameBuffer.bpp == 16)
		patterns_per_image=16;
	else
		patterns_per_image=24;

	/* Depending on startPattern, skip N buffers */
	curBuffer = startPattern/patterns_per_image;
	frameBuffer.frameBuffer += ((frameBufferSz/4)*curBuffer);
	curPattern = startPattern - curBuffer*patterns_per_image;

	for(i=0; i < patDefCol->numPatterns; i++)
	{
		if(curPattern % patterns_per_image == 0)
		{
			//First clear the area of interest
			rect.startX = 0;
			rect.startY = 0;
			rect.height = frameBuffer.height;
			rect.width = frameBuffer.width;
			rect.pixelVal = 0;
			DrawRectangle(&rect, &frameBuffer, true);
		}
        
        //Guard against rectangles drawn out of the left bound of the frame
        if((patDefCol->colMidPix[i] - patDefCol->colWidth/2) < 0)
            rect.startX = 0;
        else
            rect.startX = patDefCol->colMidPix[i] - patDefCol->colWidth/2;

		rect.startY = 0;
		rect.height = frameBuffer.height;
        
        //Guard against rectangles drawn out of the right bound of the frame
        if((rect.startX + patDefCol->colWidth) > pFB->width)
            rect.width = pFB->width - rect.startX;
        else
		    rect.width = patDefCol->colWidth;
        
		rect.pixelVal = 1 << (curPattern%patterns_per_image);

		DrawRectangle(&rect, &frameBuffer, false);
		curPattern++;
		if(curPattern % patterns_per_image == 0)
		{
			//Advance frame buffer pointer
			frameBuffer.frameBuffer += frameBufferSz/4;
			curBuffer++;
			if(curBuffer == frameBuffer.numFBs)
				break;
		}
	}
	
	return (patDefCol->numPatterns);
}

DLPSPEC_ERR_CODE dlpspec_scan_col_genPatDef(const scanConfig *pScanConfig, 
		const calibCoeffs *pCoeffs, patDefCol *patDef)
/**
 * @brief Function to generate a column pattern definition.
 *
 * This function generates a column pattern definition from a scan configuration 
 * and calibration coefficients. The coefficients are necessary because the 
 * pattern definition includes information about the specific pixel centers for
 * each group of pixels that will be turned on.
 *
 * @param[in]   pScanConfig Pointer to the scan configuration
 * @param[in]   pCoeffs     Pointer to the calibration coefficients for the 
 *							unit in question
 * @param[out]  patDef      Pointer to the column pattern definition where the 
 *							definition will be stored
 *
 * @return  >0  Number of binary patterns generated from scan config
 * @return  ≤0  Error code
 */
{
	int i;
	double start_px;
	double end_px;
	double step_x = 0;
	double mid_x;
    
    DLPSPEC_ERR_CODE ret_val = (DLPSPEC_PASS);
    
	if ((pScanConfig == NULL) || (pCoeffs == NULL) || (patDef == NULL))
		return (ERR_DLPSPEC_NULL_POINTER);

	ret_val = dlpspec_util_nmToColumn(pScanConfig->wavelength_start_nm, 
			pCoeffs->PixelToWavelengthCoeffs, &start_px);
    if (ret_val < 0)
    {
        return (ERR_DLPSPEC_INVALID_INPUT);
    }
    
	ret_val = dlpspec_util_nmToColumn(pScanConfig->wavelength_end_nm, 
			pCoeffs->PixelToWavelengthCoeffs, &end_px);
    if (ret_val < 0)
    {
        return (ERR_DLPSPEC_INVALID_INPUT);
    }
    
    if ((pScanConfig->num_patterns-1) > 0)
	    step_x = (double)(end_px - start_px)/(double)(pScanConfig->num_patterns-1);
    /*
    From scan configuration, generate and return the following:
    - columns turned on for each pattern (used to actually generate patterns)
    */
	patDef->numPatterns = pScanConfig->num_patterns;
	patDef->colWidth = pScanConfig->width_px;

	mid_x = start_px;
	for(i=0; i<pScanConfig->num_patterns; i++)
	{
		patDef->colMidPix[i] = mid_x;
		mid_x += step_x;
	}
    
	return (DLPSPEC_PASS);
}

DLPSPEC_ERR_CODE dlpspec_scan_col_interpret(const uScanData *puScanData, 
		scanResults *pResults)
/**
 * @brief Function to interpret raw scan data from a column scan into a spectrum.
 *
 * This function transforms raw data from a scan made with the column method into
 * a processed intensity spectrum.
 *
 * @param[in]   puScanData   Pointer to the scan data
 * @param[out]  pResults    Pointer where the results will be stored
 *
 * @return      Error code
 */
{
	patDefCol patDef;
	int i;
    double mid_px_f;
    scanConfig cfg;
	const scanData *pScanData;
    
    DLPSPEC_ERR_CODE ret_val = (DLPSPEC_PASS);
    
    if ((pResults == NULL) || (puScanData == NULL))
    	return (ERR_DLPSPEC_NULL_POINTER);

	pScanData = &puScanData->data;
    /*
    Procedure:
    1. Compute DC detector level during black patterns (every 25th, as defined 
														by sequence, #def.)
    2. Subtract found DC detector level from remaining measurements.
    3. Compute wavelength centers for the scanData configuration, using genPatDef
    4. Copy computed wavelengths, intensities, and header info from scanData 
															to scanResults.
    */

	dlpspec_copy_scanData_hdr_to_scanResults(puScanData, pResults);
	
    dlpspec_subtract_remove_dc_level(pScanData, pResults);

    cfg.scan_type = pScanData->scan_type;
    cfg.scanConfigIndex = pScanData->scanConfigIndex;
    /* pResults->cfg.ScanConfig_serial_number
    pResults->cfg.config_name*/
    cfg.wavelength_start_nm = pScanData->wavelength_start_nm;
    cfg.wavelength_end_nm = pScanData->wavelength_end_nm;
    cfg.width_px =  pScanData->width_px;
    cfg.num_patterns = pScanData->num_patterns;
    cfg.num_repeats = pScanData->num_repeats;
    /* Compute wavelength centers for the scanData configuration, using genPatDef */
    ret_val = dlpspec_scan_col_genPatDef(&cfg, &(pScanData->calibration_coeffs),
		   	&patDef);
    if (ret_val < 0)
    {
        return ret_val;
    }
    
	for(i=0; i < pResults->length; i++)
	{
        if (pScanData->width_px % 2 != 0)
            mid_px_f = patDef.colMidPix[i];
        else
            mid_px_f = patDef.colMidPix[i] - 0.5;
        
		ret_val = dlpspec_util_columnToNm(mid_px_f, 
				&(pScanData->calibration_coeffs.PixelToWavelengthCoeffs[0]), 
				&pResults->wavelength[i]);
        if (ret_val < 0)
        {
            return ret_val;
        }
	}

	pResults->pga = pScanData->pga;
	return ret_val;

}

/** @} // group group_scan_col
 *
 */
