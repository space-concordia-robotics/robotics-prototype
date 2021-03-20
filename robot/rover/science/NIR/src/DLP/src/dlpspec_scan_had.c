/*****************************************************************************
**
**  Copyright (c) 2015 Texas Instruments Incorporated.
**
******************************************************************************
**
**  DLP Spectrum Library
**
*****************************************************************************/

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "dlpspec_scan_had.h"
#include "dlpspec_had_defs.h"
#include "dlpspec_util.h"
#include "dlpspec_helper.h"
#include "dlpspec_scan_col.h"

/**
 * @addtogroup group_scan_had
 *
 * @{
 */

DLPSPEC_ERR_CODE getSMatrix(const uint16_t order, double *unpackedMatrix)
/**
 * @brief Return an unpacked sparse matrix from the packed Hadamard matrix arrays defined in dlpspec_had_defs.h
 * 
 * The returned values in the @p unpackedMatrix array are '1' if that
 * pixel group should be on for the pattern, and '0' if that pixel group should 
 * remain off. Matrix is returned as a double because it will be used by
 * inverting and matrix multiplying to interpret the Hadamard scan data into a 
 * spectrum.
 * 
 * @param[in]   order           Hadamard matrix size requested
 * @param[out]  unpackedMatrix  Pointer to an array where the row will be stored. Array bounds should be of length (order * order) or larger.
 * 
 * @return      Error code
 */
{
    if (order > HAD_MATRIX_MAX_ORDER_AVAIL)
        return (ERR_DLPSPEC_INVALID_INPUT);

    if ((unpackedMatrix == NULL) || (g_matrix_lookup[order] == NULL))
    	return (ERR_DLPSPEC_NULL_POINTER);

    const uint8_t *packedMatrix = g_matrix_lookup[order];
    int i,j;
    double temp;

    for(i=0; i<((order*order+7)/8); i++)
    {
        for(j=0; j<8; j++)
        {
            temp = (double)((packedMatrix[i] >> j) & 1);
            unpackedMatrix[i*8+j] = temp;
        }
    }
    return DLPSPEC_PASS;
}

DLPSPEC_ERR_CODE getSMatrixRow(const uint16_t order, const int pattern, uint8_t *row)
/**
 * @brief Return a specified row from the packed Hadamard matrix arrays defined in dlpspec_had_defs.h
 * 
 * Each row in the Hadamard matrix maps to a single pattern dispalyed on the DMD.
 * Therefore, this function is for extracting a single row in order to derrive a
 * single binary pattern. The returned values in the @p row array are '1' if that
 * pixel group should be on for the pattern, and '0' if that pixel group should 
 * remain off.
 * 
 * @param[in]   order   Hadamard matrix size requested
 * @param[in]   pattern Which row of the matrix to extract.
 * @param[out]  row     Pointer to an array where the row will be stored. Array bounds should be of length @p order or larger.
 * 
 * @return      Error code
 */
{   
    int first_byte, last_byte, byte_bit;
    int curr_bit = 0;
    
    if (order > HAD_MATRIX_MAX_ORDER_AVAIL)
        return (ERR_DLPSPEC_INVALID_INPUT);
    
    if ((row == NULL) || (g_matrix_lookup[order] == NULL))
    	return (ERR_DLPSPEC_NULL_POINTER);
    
    first_byte = (order * pattern) / 8;
    last_byte = (order * (pattern + 1) + 7) / 8;
    byte_bit = order * pattern - (first_byte * 8);
    
    const uint8_t *packedMatrix = g_matrix_lookup[order];
    int i,j;
    
    for(i=first_byte; i<=last_byte; i++)
    {
        for(j=byte_bit; j<8; j++,curr_bit++)
        {
            row[curr_bit] = ((packedMatrix[i] >> j) & 1);
        }
        byte_bit = 0;
    }

    return DLPSPEC_PASS;
}

bool checkPrime(const int n)
/**
 * Check whether or not a number is prime
 * 
 * @param[in]   n   Number to check prime status
 *
 * @return      true, if prime. false, if not prime.
 */
{
    int i;
    bool isPrime = true;
    
    for(i=2;i<=n/2;++i)
    {
        if(n%i==0)
        {
            isPrime = false;
            break;
        }
    }

    return isPrime;
}

int getPaleyOrder(int n)
/**
 * Returns the next highest Hadamard S-matrix size possible with paley construction
 * 
 * @param[in]   n   Desired S-matrix size
 *
 * @return      Least possible S-matrix size that is greater than or equal to @p n
 */
{
    n++;
    if(n%4!=0)
        n += (4-(n%4));

    while(!checkPrime(n-1))
        n += 4;

    return n-1;
}

/*
* Review comment - PG
* Add Doxygen comments
*/
DLPSPEC_ERR_CODE dlpspec_scan_had_interpret(const uScanData *puScanData, scanResults *pResults)
/**
 * @brief Function to interpret raw scan data from a Hadamard scan into a spectrum.
 *
 * This function transforms raw data from a scan made with the Hadamard method into
 * a processed intensity spectrum.
 *
 * @param[in]   puScanData   Pointer to the scan data
 * @param[out]  pResults    Pointer where the results will be stored
 *
 * @return      Error code
 */
{
    double *pHadMatrix = NULL;
    double *pHadMatrixInv = NULL;
	patDefHad patDef;
	int i,j,adc_data_pos;
    int totalColGroups = 0;
    double mid_px_f;
    scanConfig cfg;
    double adc_buff[HAD_MATRIX_MAX_ORDER_AVAIL] = {0};
    double result_buff[HAD_MATRIX_MAX_ORDER_AVAIL] = {0};
    double adc_adjusted[ADC_DATA_LEN] = {0};
	const scanData *pScanData;

    DLPSPEC_ERR_CODE ret_val = DLPSPEC_PASS;

    if ((pResults == NULL) || (puScanData == NULL))
    	return (ERR_DLPSPEC_NULL_POINTER);

	pScanData = &puScanData->data;

    /*
    Procedure:
    1. Compute DC detector level during black patterns (every 25th, as defined by sequence, #def.)
    2. Subtract found DC detector level from remaining measurements.
    3. Compute wavelength centers for the scanData configuration, using genPatDef
    4. Copy computed wavelengths, intensities, and header info from scanData to scanResults.
    */

	dlpspec_copy_scanData_hdr_to_scanResults(puScanData, pResults);
	
    dlpspec_subtract_remove_dc_level(pScanData, pResults);

    for(i=0; i<pResults->length; i++)
        adc_adjusted[i] = pResults->intensity[i];

    //Copy config values in, because of TPL bug
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
    ret_val = dlpspec_scan_had_genPatDef(&cfg, &(pScanData->calibration_coeffs), &patDef);
    if (ret_val < 0)
        return ret_val;
    
    //Compute intensities
    adc_data_pos = 0;
    for(i=0; i<patDef.numSets; i++)
    {
        //Allocate for Hadamard matrices
        pHadMatrix = malloc(sizeof(double) * (patDef.set[i].hadOrder * patDef.set[i].hadOrder + 8));
        pHadMatrixInv = malloc(sizeof(double) * (patDef.set[i].hadOrder * patDef.set[i].hadOrder + 8));
        if((pHadMatrix == NULL) || (pHadMatrixInv == NULL))
        {
            ret_val = ERR_DLPSPEC_INSUFFICIENT_MEM;
            goto cleanup_and_exit;
        }
        
        ret_val = getSMatrix(patDef.set[i].hadOrder, pHadMatrix);
        if(ret_val < 0)
        {
            goto cleanup_and_exit;
        }
        
        //Get inverse of Hadamard s-matrix
        //dlpspec_matrix_transpose(pHadMatrix, pHadMatrixInv, patDef.set[i].hadOrder, patDef.set[i].hadOrder);
        for(j=0; j < patDef.set[i].hadOrder * patDef.set[i].hadOrder; j++)
        {
            pHadMatrixInv[j] = (pHadMatrix[j]-0.5)/((patDef.set[i].hadOrder+1)/4);
        }
        
        //Copy data to zero filled temporary buffer
        for(j=0; j < patDef.set[i].hadOrder; j++)
            adc_buff[j] = adc_adjusted[j + adc_data_pos];
        
        //Transform buffer data into spectrum intensity
        ret_val = dlpspec_matrix_mult(&adc_buff[0], pHadMatrixInv, &result_buff[0], 1, patDef.set[i].hadOrder, patDef.set[i].hadOrder);
        if(ret_val < 0)
        {
            goto cleanup_and_exit;
        }
        
        //Store spectrum intensity and wavelength in results struct
        for(j=0; j < patDef.set[i].numColGroups; j++)
        {
            //Intensity
            pResults->intensity[patDef.set[i].colGroupNum[j]] = result_buff[j];
            adc_buff[j] = 0;
            result_buff[j] = 0;
            
            //Wavelength
            if (pScanData->width_px % 2 != 0)
                mid_px_f = patDef.set[i].colMidPix[j];
            else
                mid_px_f = patDef.set[i].colMidPix[j] - 0.5;
            
            ret_val = dlpspec_util_columnToNm(mid_px_f, &(pScanData->calibration_coeffs.PixelToWavelengthCoeffs[0]), &pResults->wavelength[patDef.set[i].colGroupNum[j]]);
            if(ret_val < 0)
            {
                goto cleanup_and_exit;
            }
        }
        
        //Finish resetting buffer to zero
        for(j=patDef.set[i].numColGroups; j < patDef.set[i].hadOrder; j++)
        {
            adc_buff[j] = 0;
            result_buff[j] = 0;
        }
        
        totalColGroups += patDef.set[i].numColGroups;
        adc_data_pos += patDef.set[i].hadOrder;        

        free(pHadMatrix);
        free(pHadMatrixInv);
        pHadMatrix = NULL;
        pHadMatrixInv = NULL;
    }
    
    pResults->length = totalColGroups;
	
    cleanup_and_exit:
        if(pHadMatrix != NULL)
            free(pHadMatrix);
        if(pHadMatrixInv != NULL)
            free(pHadMatrixInv);
        
    return ret_val;
    
}

DLPSPEC_ERR_CODE dlpspec_scan_had_genPatDef(const scanConfig *pScanConfig, const calibCoeffs *pCoeffs, patDefHad *patDefH)
/**
 * @brief Function to generate a Hadamard pattern definition.
 *
 * This function generates a Hadamard pattern definition from a scan configuration 
 * and calibration coefficients. The coefficients are necessary because the 
 * pattern definition includes information about the specific pixel centers for
 * each group of pixels that will be turned on.
 *
 * @param[in]   pScanConfig Pointer to the scan configuration
 * @param[in]   pCoeffs     Pointer to the calibration coefficients for the unit in question
 * @param[out]  patDefH     Pointer to the Hadamard pattern definition where the definition will be stored
 *
 * @return  ≤0  Error code
 */
{
    int i,j,k;
    patDefCol patDefC;
    int this_edge_px;
    int next_edge_px;
    int px_between;
    
    DLPSPEC_ERR_CODE ret_val = DLPSPEC_PASS;

    if ((pScanConfig == NULL) || (pCoeffs == NULL) || (patDefH == NULL))
    	return (ERR_DLPSPEC_NULL_POINTER);
    
    ret_val = dlpspec_scan_col_genPatDef(pScanConfig, pCoeffs, &patDefC);
    if(ret_val < 0)
    {
        return ret_val;
    }
    
    // Determine how many Hadamard sets are necessary
    patDefH->numSets = 0;
    for(i=1; i<MAX_HAD_SETS; i++)
    {
        for(j=0; j<(pScanConfig->num_patterns-i-1); j++)
        {
            if(patDefC.colMidPix[j] < patDefC.colMidPix[j+i])
                {
                    this_edge_px = patDefC.colMidPix[j] - patDefC.colWidth/2 + patDefC.colWidth - 1;
                    next_edge_px = patDefC.colMidPix[j+i] - patDefC.colWidth/2;
                    px_between = next_edge_px - this_edge_px - 1;
                }
            else
                {
                    this_edge_px = patDefC.colMidPix[j] - patDefC.colWidth/2;
                    next_edge_px = patDefC.colMidPix[j+i] - patDefC.colWidth/2 + patDefC.colWidth - 1;
                    px_between = this_edge_px - next_edge_px - 1;
                }
                
            if(px_between < MIN_PX_BETWEEN_COL_GROUPS)
            {
                patDefH->numSets = 0;
                break;
            }
            else
                patDefH->numSets = i;
        }
        
        if(patDefH->numSets != 0)
            break;
    }

    // If a valid set of Hadamard sets was not found, we cannot create this pattern definition
    if(patDefH->numSets == 0)
    {
        return ERR_DLPSPEC_FAIL;
    }
    
    // Build each Hadamard set
    patDefH->numPatterns = 0;
    for(i=0; i<patDefH->numSets; i++)
    {
        
        patDefH->set[i].numColGroups = 0;
        for(k=i,j=0; k<pScanConfig->num_patterns; k+=patDefH->numSets,j++)
        {
            patDefH->set[i].colGroupNum[j] = k;
            patDefH->set[i].colMidPix[j] = patDefC.colMidPix[k];
            patDefH->set[i].numColGroups++;
        }
        patDefH->set[i].hadOrder = getPaleyOrder(patDefH->set[i].numColGroups);
        patDefH->numPatterns += patDefH->set[i].hadOrder;
    }
    
    // Set column width
    patDefH->colWidth = patDefC.colWidth;

    /*
    From scan configuration, compute:
    - How many scan 'groups' of hadamard scans are necessary to ensure two groups of pixels from adjacent wavelengths do not overlap or touch. For instance, if the scan has zero overlap, two different groups are required. Number of groups required should be: floor(1 + width_px / min_distance_between_adjacent_wavelengths_px)
    Then split wavelengths requested into necessary scan groups (e.g., 1 2 3 4 5 6 7 8 into [1 4 7], [2 5 8], [3 6]). For each group, do the following:
    1. Compute or lookup paley construction of Hadamard matrix N+1 size, where N is the number of wavelengths requested
    2. Remove first line and column (could be done beforehand if stored)
    3. Shuffle columns to randomize (could be done beforehand if stored)
    4. Output the following for each group:
        - columns turned on for each pattern (to actually generate patterns)
        - wavelength centers of each column region in this pattern group
        - Hadamard matrix or inverse matrix describing the pattern group structure of which wavelengths are turned on at the same time. Each column region with a center wavelength would be represented by column, and each pattern would be represented by a row (or transpose).
    */
    
	return ret_val;
}

int32_t dlpspec_scan_had_genPatterns(const patDefHad *patDefHad, 
		const FrameBufferDescriptor *pFB, uint32_t startPattern)
/**
 * @brief Function to generate patterns for a Hadamard scan.
 *
 * This function takes the Hadamard pattern definition an writes the described
 * patterns to the frame buffer described in the frame buffer descriptor.
 *
 * @param[in]   patDefHad		Pointer to Hadamard pattern definition
 * @param[out]  pFB				Pointer to frame buffer descriptor where the 
 *								patterns will be stored
 * @param[in]   startPattern	Pattern number at which to start drawing
 *
 * @return  >0  Number of binary patterns generated from scan the pattern definition
 * @return  ≤0  Error code as #DLPSPEC_ERR_CODE
 */
{
    uint8_t row[HAD_MATRIX_MAX_ORDER_AVAIL] = {0};
	int i,j;
	RectangleDescriptor rect;
    int set, pattern;
	int curPattern;
	int patterns_per_image;
    uint32_t curBuffer=0;
    
    DLPSPEC_ERR_CODE ret_val = DLPSPEC_PASS;
    
    if ((patDefHad == NULL) || (pFB == NULL))
    	return (ERR_DLPSPEC_NULL_POINTER);    
    
	int frameBufferSz = (pFB->width * pFB->height * (pFB->bpp/8));
	FrameBufferDescriptor frameBuffer;

	memcpy(&frameBuffer, pFB, sizeof(FrameBufferDescriptor));

	if(frameBuffer.bpp == 16)
		patterns_per_image=16;
	else
		patterns_per_image=24;

	/* Depending on startPattern, skip N buffers */
	curBuffer = startPattern/patterns_per_image;
	frameBuffer.frameBuffer += ((frameBufferSz/4)*curBuffer);
	curPattern = startPattern - curBuffer*patterns_per_image;
    
    set = 0;
    pattern = 0;

	for(i=0; i < patDefHad->numPatterns; i++)
	{
		if(curPattern % patterns_per_image == 0)
		{
			//First clear the area of interest
			rect.startX = 0;
			rect.startY = 0;
/* Writing frame buffer of height=1 for optimiation. This should be 
 * changed to frameBuffer.height if dlpspec_scan_bendPatterns() is not 
 * called after dlpspec_scan_genPatterns() */
			rect.height = 1;
			rect.width = frameBuffer.width;
			rect.pixelVal = 0;
			DrawRectangle(&rect, &frameBuffer, true);
		}
                
        //Lookup Hadamard s-matrix to see if this pattern should contain this column group
        ret_val = getSMatrixRow(patDefHad->set[set].hadOrder, pattern, &row[0]);
        if (ret_val < 0)
        {
            return ret_val;
        }
        
        for(j=0; j < patDefHad->set[set].numColGroups; j++)
        {
            if(row[j] == 1) //Hadamard s-matrix should contain this column group
            {
                //Guard against rectangles drawn out of the left bound of the frame
                if((patDefHad->set[set].colMidPix[j] - patDefHad->colWidth/2) < 0)
                    rect.startX = 0;
                else
                    rect.startX = patDefHad->set[set].colMidPix[j] - patDefHad->colWidth/2;

        		rect.startY = 0;
        		rect.height = 1; //frameBuffer.height;
        
                //Guard against rectangles drawn out of the right bound of the frame
                if((rect.startX + patDefHad->colWidth) > pFB->width)
                    rect.width = pFB->width - rect.startX;
                else
        		    rect.width = patDefHad->colWidth;
        
        		rect.pixelVal = 1 << (curPattern%patterns_per_image);

        		DrawRectangle(&rect, &frameBuffer, false);
            }
        }
        
        //Increment to next pattern and/or set
        if(pattern < (patDefHad->set[set].hadOrder - 1))
            {
                pattern++;
            }
        else
            {
                pattern = 0;
                set++;
            }        
        
        //Clean up
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
    
    if(ret_val == DLPSPEC_PASS)
    {
        return (patDefHad->numPatterns);
    }
    else
    {
        return ret_val;
    }
}

/** @} // group group_scan_had
 *
 */
