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
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "dlpspec_helper.h"
#include "dlpspec_scan.h"
#include "dlpspec_types.h"
#include "dlpspec_setup.h"
#include "dlpspec_scan_had.h"
#include "tpl.h"


void dlpspec_copy_scanData_hdr_to_scanResults(const uScanData *pScanData, 
		scanResults *pResults)
/**
 * @brief Copies all scan header information from @p pScanData to @p pResults.
 * 
 * This is typically used when interpretting scans, as the diagnostic information needs
 * to be copied from the original data to the results so the client of the caller
 * can optionally store only the results and still have all metadata.
 * 
 * @param[in]   pScanData       pointer to the scanData to copy data from
 * @param[out]  pResults        If overwrite_pixel = true, the pixel present 
 *								in the frame buffer will be 
 *                              overwritten with the specified pixel value. 
 *                              When false, the specified pixel
 *                              value will be appended or bitwise OR'ed to the 
 *                              existing pixel value. This option shall be used
 *                              to draw rectangles in a specific bit plane while 
 *                              leaving other bit planes unchanged.
 *
 */
{
	pResults->header_version = pScanData->data.header_version;
    pResults->year = pScanData->data.year;
    pResults->month = pScanData->data.month;
    pResults->day = pScanData->data.day;
    pResults->day_of_week = pScanData->data.day_of_week;
    pResults->hour = pScanData->data.hour;
    pResults->minute = pScanData->data.minute;
    pResults->second = pScanData->data.second;
    pResults->system_temp_hundredths = pScanData->data.system_temp_hundredths;
    pResults->detector_temp_hundredths = pScanData->data.detector_temp_hundredths;
    pResults->humidity_hundredths = pScanData->data.humidity_hundredths;
    pResults->lamp_pd = pScanData->data.lamp_pd;
	memcpy(&(pResults->calibration_coeffs), &(pScanData->data.calibration_coeffs), 
			sizeof(calibCoeffs));
    memcpy(pResults->serial_number, pScanData->data.serial_number, NANO_SER_NUM_LEN);
    memcpy(pResults->scan_name, pScanData->data.scan_name, SCAN_NAME_LEN);
    //memcpy(&(pResults->cfg), &(pScanData->cfg), sizeof(scanConfig));
    pResults->cfg.head.scan_type = SLEW_TYPE;
    pResults->cfg.head.scanConfigIndex = pScanData->data.scanConfigIndex;
	memcpy(pResults->cfg.head.ScanConfig_serial_number, 
			pScanData->data.ScanConfig_serial_number, NANO_SER_NUM_LEN);
	memcpy(pResults->cfg.head.config_name, pScanData->data.config_name, 
			SCAN_CFG_FILENAME_LEN);
	pResults->cfg.head.num_repeats = pScanData->data.num_repeats;
	pResults->scanDataIndex = pScanData->data.scanDataIndex;
	pResults->pga = pScanData->data.pga;
    if(dlpspec_scan_data_get_type(pScanData) != SLEW_TYPE)
	{
		pResults->cfg.head.num_sections = 1;
		pResults->cfg.section[0].wavelength_start_nm = pScanData->data.wavelength_start_nm;
		pResults->cfg.section[0].wavelength_end_nm = pScanData->data.wavelength_end_nm;
		pResults->cfg.section[0].width_px =  pScanData->data.width_px;
		pResults->cfg.section[0].num_patterns = pScanData->data.num_patterns;
		pResults->cfg.section[0].section_scan_type = pScanData->data.scan_type;
	}
	else
	{
		memcpy(&pResults->cfg, &pScanData->slew_data.slewCfg, sizeof(slewScanConfig));
	}
}


static void DrawRectangle32BPP(const RectangleDescriptor *r, 
		const FrameBufferDescriptor *fb, const bool overwrite_pixel)
/*
 * If overwrite_pixel = true, the pixel present in the frame buffer will be 
 * overwritten with the specified pixel value. When false, the specified pixel 
 * value will be appended or bitwise OR'ed to the existing pixel value. This option
 * shall be used to draw rectangles in a specific bit plane while leaving 
 * other bit planes unchanged.
 */
{
	uint32_t line = 0;
	uint32_t pixel = 0;
	uint32_t *linePtr;

	for(line=r->startY; line < r->startY+r->height; line++)
	{
		linePtr = fb->frameBuffer+line*fb->width;

		for(pixel=r->startX; pixel < r->startX+r->width; pixel++)
		{
			if(overwrite_pixel == false)
			{
				linePtr[pixel] |= r->pixelVal;
			}
			else
			{
				linePtr[pixel] = r->pixelVal;
			}
		}
	}
}


static void DrawRectangle16BPP(const RectangleDescriptor *r, 
		const FrameBufferDescriptor *fb, const bool overwrite_pixel)
/*
 * If overwrite_pixel = true, the pixel present in the frame buffer will be 
 * overwritten with the specified pixel value
 * When false, the specified pixel value will be appended or bitwise OR'ed 
 * to the existing pixel value. This option shall be used to draw rectangles 
 * in a specific bit plane while leaving other bit planes unchanged.
 */
{
	uint32_t line = 0;
	uint32_t pixel = 0;
	uint16_t *linePtr;

	for(line=r->startY; line < r->startY+r->height; line++)
	{
		linePtr = (uint16_t *)fb->frameBuffer;
		linePtr += line*fb->width;;

		for(pixel=r->startX; pixel < r->startX+r->width; pixel++)
		{
			if(overwrite_pixel == false)
			{
				linePtr[pixel] |= r->pixelVal;
			}
			else
			{
				linePtr[pixel] = r->pixelVal;
			}
		}
	}
}

static void DrawRectangle24BPP(const RectangleDescriptor *r, const 
		FrameBufferDescriptor *fb, const bool overwrite_pixel)
/*
 * If overwrite_pixel = true, the pixel present in the frame buffer will be 
 * overwritten with the specified pixel value
 * When false, the specified pixel value will be appended or bitwise OR'ed 
 * to the existing pixel value. This option shall be used to draw rectangles 
 * in a specific bit plane while leaving other bit planes unchanged.
 */
{
	uint32_t line = 0;
	uint32_t pixel = 0;
	uint32_t startX_burst = (((r->startX+3)/4)*4); //ceil(r->startX/4)
	uint32_t endX = r->startX+r->width;
	uint32_t endX_burst = ((endX/4)*4);
	uint32_t pixelWord1_2, pixelWord2_3, pixelWord3_4;
	uint32_t *linePtr;
	uint8_t red, blu, grn;

	blu = r->pixelVal >> 16;
	grn = r->pixelVal >> 8;
	red = r->pixelVal;

	pixelWord1_2 = (red << 24) | (blu << 16) | (grn << 8) | red; //32-bit word containing full pixel 1 and part of pixel 2
	pixelWord2_3 = (grn << 24) | (red << 16) | (blu << 8) | grn; //32-bit word containing part of pixel 2 and part of pixel 3
	pixelWord3_4 = (blu << 24) | (grn << 16) | (red << 8) | blu; //32-bit word containing part of pixel 3 and full pixel 4

	for(line=r->startY; line < r->startY+r->height; line++)
	{
		linePtr = fb->frameBuffer+line*fb->width*3/4;
		linePtr += startX_burst*3/4;

		if(startX_burst != r->startX)
			linePtr -= 3;

		if((startX_burst - r->startX) == 3)
		{
			if(overwrite_pixel == false)
			{
				*linePtr |= ( (*linePtr | pixelWord1_2 ) & 0xff000000);
				linePtr++;
				if(r->width > 1)
					*linePtr++ |= pixelWord2_3;
				else	//width = 1
				{
					*linePtr = (*linePtr) | (pixelWord2_3 & 0xffff);
					linePtr++;
				}
				if(r->width > 2)
					*linePtr++ |= pixelWord3_4;
				else if (r->width == 2)
				{
					*linePtr = (*linePtr) | (pixelWord3_4 & 0xff);
					linePtr++;
				}
			}
			else
			{
				*linePtr = (*linePtr & 0xffffff) | (pixelWord1_2 & 0xff000000);
				linePtr++;
				if(r->width > 1)
					*linePtr++ = pixelWord2_3;
				else	//width = 1
				{
					*linePtr = (*linePtr & 0xffff0000) | (pixelWord2_3 & 0xffff);
					linePtr++;
				}
				if(r->width > 2)
					*linePtr++ = pixelWord3_4;
				else if (r->width == 2)
				{
					*linePtr = (*linePtr & 0xffffff00) | (pixelWord3_4 & 0xff);
					linePtr++;
				}
			}
			if(r->width < 4)
				continue;
		}
		else if((startX_burst - r->startX) == 2)
		{
			linePtr++;
			if(overwrite_pixel == false)
			{
				*linePtr |= ( (*linePtr | pixelWord2_3) & 0xffff0000);
				linePtr++;
				if(r->width > 1)
					*linePtr++ |= pixelWord3_4;
				else	//width = 1
				{
					*linePtr = (*linePtr) | (pixelWord3_4 & 0xff);
					linePtr++;
				}
			}
			else
			{
				*linePtr = (*linePtr & 0xffff) | (pixelWord2_3 & 0xffff0000);
				linePtr++;
				if(r->width > 1)
					*linePtr++ = pixelWord3_4;
				else	//width = 1
				{
					*linePtr = (*linePtr & 0xffffff00) | (pixelWord3_4 & 0xff);
					linePtr++;
				}
			}
			if(r->width < 3)
				continue;
		}
		else if((startX_burst - r->startX) == 1)
		{
			linePtr++;
			linePtr++;
			if(overwrite_pixel == false)
			{
				*linePtr |= ( (*linePtr | pixelWord3_4) & 0xffffff00);
				linePtr++;
			}
			else
			{
				*linePtr = (*linePtr & 0xff) | (pixelWord3_4 & 0xffffff00);
				linePtr++;
			}
			if(r->width < 2)
				continue;
		}
		for(pixel=startX_burst; pixel < endX_burst; pixel+=4)
		{
			if(overwrite_pixel == false)
			{
				*linePtr++ |= pixelWord1_2;
				*linePtr++ |= pixelWord2_3;
				*linePtr++ |= pixelWord3_4;
			}
			else
			{
				*linePtr++ = pixelWord1_2;
				*linePtr++ = pixelWord2_3;
				*linePtr++ = pixelWord3_4;
			}
		}
		if((endX - endX_burst) == 1)
		{
			if(overwrite_pixel == false)
				*linePtr |= ( (*linePtr | pixelWord1_2) & 0xffffff);
			else
				*linePtr = (*linePtr & 0xff000000) | (pixelWord1_2 & 0xffffff);
		}
		else if((endX - endX_burst) == 2)
		{
			if(overwrite_pixel == false)
			{
				*linePtr++ |= pixelWord1_2;
				*linePtr |= ( (*linePtr | pixelWord2_3) & 0x0000ffff);
			}
			else
			{
				*linePtr++ = pixelWord1_2;
				*linePtr = (*linePtr & 0xffff0000) | (pixelWord2_3 & 0x0000ffff);
			}
		}
		else if((endX - endX_burst) == 3)
		{
			if(overwrite_pixel == false)
			{
				*linePtr++ |= pixelWord1_2;
				*linePtr++ |= pixelWord2_3;
				*linePtr |= ( (*linePtr | pixelWord3_4) & 0xff);
			}
			else
			{
				*linePtr++ = pixelWord1_2;
				*linePtr++ = pixelWord2_3;
				*linePtr = (*linePtr & 0xffffff00) | (pixelWord3_4 & 0xff);
			}
		}
	}
}


void DrawRectangle(const RectangleDescriptor *r, 
		const FrameBufferDescriptor *fb, const bool overwrite_pixel)
{
/**
 * @brief Draws a rectangle into the frame buffer specified by @p *fb.
 * 
 * This is used by various pattern generation functions to either clear the 
 * frame buffer or to write pixel data to the frame buffer in rectangles.
 * 
 * @param[in]   r               pointer to the rectangle descriptor which 
 *								defines what rectangle to draw
 * @param[in]   fb              frame buffer descriptor which describes the 
 *								location and properties of
 *                              the frame buffer to write into
 * @param[in]   overwrite_pixel If overwrite_pixel = true, the pixel present
 *								in the frame buffer will be 
 *                              overwritten with the specified pixel value. 
 *                              When false, the specified pixel
 *                              value will be appended or bitwise OR'ed to the 
 *                              existing pixel value. This option shall be used
 *                              to draw rectangles in a specific bit plane while 
 *                              leaving other bit planes unchanged.
 *
 */
	switch(fb->bpp)
	{
	case 24:
		DrawRectangle24BPP(r, fb, overwrite_pixel);
		break;
	case 32:
		DrawRectangle32BPP(r, fb, overwrite_pixel);
		break;
	case 16:
		DrawRectangle16BPP(r, fb, overwrite_pixel);
		break;
	}
}


static tpl_node *map_data_to_tplnode(const void *struct_p, 
		const BLOB_TYPES data_type)
/**
 * Prepares #tpl_node for serialization or deserialization
 * 
 * @param[in]   struct_p    pointer to the blob of data
 * @param[in]   data_type   blob type to be mapped
 *    
 * @return      No error: #tpl_node pointer
 * @return      Error:    NULL pointer
 *
 */
{
	char format_str[300];
	tpl_node *tn=NULL;

	switch (data_type)
	{
		case SCAN_DATA_TYPE:
			strcpy(format_str, "S(");
			strcat(format_str, SCAN_DATA_FORMAT);
			strcat(format_str, ")");
			tn = tpl_map(format_str, struct_p, 
					SCAN_NAME_LEN, NUM_SHIFT_VECTOR_COEFFS, NUM_PIXEL_NM_COEFFS,
                    NANO_SER_NUM_LEN, NANO_SER_NUM_LEN, SCAN_CFG_FILENAME_LEN, 
					ADC_DATA_LEN);
			break;

		case CFG_TYPE:
			strcpy(format_str, "S(");
			strcat(format_str, SCAN_CFG_FORMAT);
			strcat(format_str, ")");
			tn = tpl_map(format_str, struct_p, NANO_SER_NUM_LEN, 
					SCAN_CFG_FILENAME_LEN);
			break;

		case SLEW_CFG_HEAD_TYPE:
			strcpy(format_str, "S(");
			strcat(format_str, SLEW_SCAN_CFG_HEAD_FORMAT);
			strcat(format_str, ")");
			tn = tpl_map(format_str, struct_p, NANO_SER_NUM_LEN, 
					SCAN_CFG_FILENAME_LEN);
			break;

		case SLEW_CFG_SECT_TYPE:
			strcpy(format_str, "S(");
			strcat(format_str, SLEW_SCAN_CFG_SECT_FORMAT);
			strcat(format_str, ")#");
			tn = tpl_map(format_str, struct_p, SLEW_SCAN_MAX_SECTIONS);
			break;

		case CALIB_TYPE:
			strcpy(format_str, "S(");
			strcat(format_str, CALIB_COEFFS_FORMAT);
			strcat(format_str, ")");
			tn = tpl_map(format_str , struct_p , NUM_SHIFT_VECTOR_COEFFS,
					NUM_PIXEL_NM_COEFFS);
			break;
            
        case REF_CAL_MATRIX_TYPE:
			strcpy(format_str, "S(");
			strcat(format_str, REF_CAL_MATRIX_FORMAT);
			strcat(format_str, ")");
			tn = tpl_map(format_str, struct_p, 
					REF_CAL_INTERP_WIDTH, REF_CAL_INTERP_WAVELENGTH, 
					REF_CAL_INTERP_WIDTH,  
					REF_CAL_INTERP_WAVELENGTH);
			break;

		case SLEW_DATA_HEAD_TYPE:   
			strcpy(format_str, "S(");
			strcat(format_str, SLEW_SCAN_DATA_HEAD_FORMAT);
			strcat(format_str, ")");
			tn = tpl_map(format_str, struct_p, 
					SCAN_NAME_LEN, NUM_SHIFT_VECTOR_COEFFS, NUM_PIXEL_NM_COEFFS,
                    NANO_SER_NUM_LEN);
			break;

		case SLEW_DATA_ADC_TYPE:   
			strcpy(format_str, ADC_DATA_FORMAT);
			tn = tpl_map(format_str, struct_p, ADC_DATA_LEN);
			break;

		default: /* Unrecognized type, should not be possible with BLOB_TYPES type */
            tn = NULL;
			break;
	}

	return tn;
}

SCAN_TYPES dlpspec_scan_data_get_type(const uScanData *pData)
{
    if(pData->slew_data.slewCfg.head.scan_type == SLEW_TYPE)
        return SLEW_TYPE;
    else if(pData->data.scan_type == COLUMN_TYPE)
        return COLUMN_TYPE;
    else if(pData->data.scan_type == HADAMARD_TYPE)
        return HADAMARD_TYPE;
    else
        return ERR_DLPSPEC_ILLEGAL_SCAN_TYPE;
}

bool dlpspec_is_slewdatatype(void *struct_p, size_t buffer_size)
{
	char *fmt;
	bool ret;
    char std_cfg_format[] = "S(" SCAN_DATA_FORMAT ")";

	fmt = tpl_peek(TPL_MEM, struct_p, buffer_size);

	if(fmt == NULL)
		return false;

    if(strcmp(fmt, std_cfg_format) == 0)
        ret = false;
	else
        ret = true;

	free (fmt);
	return ret;
}

bool dlpspec_is_slewcfgtype(void *struct_p, size_t buffer_size)
{
	char *fmt;
	char scan_type;

	fmt = tpl_peek(TPL_MEM | TPL_DATAPEEK, struct_p, buffer_size, "c", &scan_type);
	free (fmt);

	if(scan_type == SLEW_TYPE)
		return true;
	else
		return false;
}


DLPSPEC_ERR_CODE dlpspec_get_scanData_from_slewScanData(const slewScanData *pSlew,
                                                scanData *pData, int section_index)
{
    int data_start_index = 0;
    uint16_t adc_data_length;
    uint16_t num_patterns;
    uint16_t num_black_patterns;

    if(section_index >= pSlew->slewCfg.head.num_sections)
        return ERR_DLPSPEC_INVALID_INPUT;

    dlpspec_scan_section_get_adc_data_range(pSlew, section_index, &data_start_index, &num_patterns, &num_black_patterns);
    adc_data_length = num_patterns + num_black_patterns;

    memcpy(pData, pSlew, sizeof(slewScanData) - sizeof(slewScanConfig) - sizeof(int32_t)*ADC_DATA_LEN);
    pData->scan_type = pSlew->slewCfg.section[section_index].section_scan_type;
    pData->scanConfigIndex = pSlew->slewCfg.head.scanConfigIndex;
    memcpy(pData->ScanConfig_serial_number, pSlew->slewCfg.head.ScanConfig_serial_number, NANO_SER_NUM_LEN);
    memcpy(pData->config_name, pSlew->slewCfg.head.config_name, SCAN_CFG_FILENAME_LEN);
    pData->wavelength_start_nm = pSlew->slewCfg.section[section_index].wavelength_start_nm;
    pData->wavelength_end_nm = pSlew->slewCfg.section[section_index].wavelength_end_nm;
    pData->width_px = pSlew->slewCfg.section[section_index].width_px;
    pData->num_patterns = pSlew->slewCfg.section[section_index].num_patterns;
    pData->num_repeats = pSlew->slewCfg.head.num_repeats;

    memcpy(&pData->adc_data[0], &pSlew->adc_data[data_start_index],
            sizeof(int32_t)*adc_data_length);

    pData->adc_data_length = adc_data_length;
    pData->black_pattern_first -= data_start_index % pData->black_pattern_period;

    return DLPSPEC_PASS;
}

DLPSPEC_ERR_CODE dlpspec_deserialize(void* struct_p, const size_t buffer_size,
	   	BLOB_TYPES data_type)
/**
 * Deserializes a blob in place for the passed in struct pointer @p struct_p. 
 * Upon return, @p struct_p holds a struct of the type determined by @p data_type.
 * 
 * @param[in,out] struct_p    pointer to the blob of data, returned as applicable struct.
 * @param[in]     buffer_size size allocated to blob
 * @param[in]     data_type   blob type to be deserialized in place
 *    
 * @return        Error code
 *
 */
{
	tpl_node *tn;
    DLPSPEC_ERR_CODE ret_val = DLPSPEC_PASS;
    int tpl_err = 0;

	tn = map_data_to_tplnode(struct_p, data_type);
	if(tn != NULL)
	{
#ifdef NANO_PRE_1_1_8_BLE_WORKAROUND
        // If 'tpl\0' was overwritten by BLE in DLP NIRscan Nano firmware 
		// ≤ v1.1.7, try unpacking anyway
        if ((data_type == REF_CAL_MATRIX_TYPE) || (data_type == SCAN_DATA_TYPE))
        {
            const char tpl_header[] = "tpl\0";
            memcpy(struct_p, tpl_header, sizeof(tpl_header));
        }
#endif
        
        // unpack data
        tpl_err = tpl_load(tn, TPL_MEM|TPL_EXCESS_OK, struct_p, buffer_size);
        if ( 0 != tpl_err) // TPL error in loading
        {
            ret_val = ERR_DLPSPEC_TPL;
        }
        else
        {
            tpl_err = tpl_unpack(tn, 0);
        }

        tpl_free(tn);
	}
    else
    {
        ret_val = ERR_DLPSPEC_TPL;
    }
  
    return ret_val;
}


DLPSPEC_ERR_CODE dlpspec_serialize(const void* struct_p, void *pBuffer,
	   	const size_t buffer_size, BLOB_TYPES data_type)
/**
 * Serializes a blob into @p pBuffer for the passed in struct pointer @p struct_p.
 * 
 * @param[in]     struct_p    pointer to the struct to be serialized.
 * @param[out]    pBuffer     pointer to the buffer 
 * @param[in]     buffer_size size allocated to @p pBuffer
 * @param[in]     data_type   blob type to be serialized
 * 
 * @return        Error code
 *
 */
{
    tpl_node *tn;
    DLPSPEC_ERR_CODE ret_val = DLPSPEC_PASS;
    int tpl_err = 0;

	tn = map_data_to_tplnode(struct_p, data_type);
	if(tn != NULL)
	{
		tpl_err = tpl_pack(tn, 0);
        if (0 == tpl_err)
        {
            tpl_err = tpl_dump(tn, TPL_MEM|TPL_PREALLOCD, pBuffer, buffer_size);
        }
		if (0 != tpl_err) // TPL error in dumping
        {
            ret_val = ERR_DLPSPEC_TPL;
        }
		tpl_free(tn);
	}
    else
    {
        ret_val = ERR_DLPSPEC_TPL;
    }

    return ret_val;
}

DLPSPEC_ERR_CODE dlpspec_get_serialize_dump_size(const void* struct_p, 
	   	size_t *pBufSize, BLOB_TYPES data_type)
/**
 * Serializes a blob into @p pBuffer for the passed in struct pointer @p struct_p.
 * 
 * @param[in]     struct_p    pointer to the struct to be serialized.
 * @param[out]    pBufSize	  size of buffer required to dump returned in this.
 * @param[in]     data_type   blob type to be serialized
 * 
 * @return        Error code
 *
 */
{
    tpl_node *tn;
    DLPSPEC_ERR_CODE ret_val = DLPSPEC_PASS;
    int tpl_err = 0;

	tn = map_data_to_tplnode(struct_p, data_type);
	if(tn != NULL)
	{
		tpl_err = tpl_pack(tn, 0);
        if (0 == tpl_err)
        {
            tpl_err = tpl_dump(tn, TPL_GETSIZE, pBufSize);
        }
		if (0 != tpl_err) // TPL error in dumping
        {
            ret_val = ERR_DLPSPEC_TPL;
        }
		tpl_free(tn);
	}
    else
    {
        ret_val = ERR_DLPSPEC_TPL;
    }

    return ret_val;
}

void dlpspec_subtract_dc_level(slewScanData *pScanData)
/**
 * Finds the average magnitude of the detector values stored during off patterns from
 * and subtracts that value from all values of the scan. The black values are not removed
 * just made zero and saved in the same place. This function will be used in case of slew
 * scans before it is split into multiple column and hadmard patterns and sent for interpret.
 *
 * @param[in]      pScanData    pointer to the scan data.
 *
 */
{
	int dc_level = 0;
	int num_black_patterns = 0;
	int scan_idx;
    int i;

    /* Compute DC detector level during black patterns */
    for(i=0; i<pScanData->adc_data_length; i++)
	{
        if((i-pScanData->black_pattern_first)%pScanData->black_pattern_period == 0)
        {
    		dc_level += pScanData->adc_data[i];
    		num_black_patterns++;
        }
	}
	if(num_black_patterns > 0)
		dc_level /= num_black_patterns;

    /* Subtract found DC detector level from remaining measurements */
    for(scan_idx=0; scan_idx < pScanData->adc_data_length; scan_idx++)
	{
		if((scan_idx-pScanData->black_pattern_first)
				%pScanData->black_pattern_period == 0)
		{
			pScanData->adc_data[scan_idx] = 0; //black pattern
		}
		else
		{
			pScanData->adc_data[scan_idx] = pScanData->adc_data[scan_idx]
				- dc_level;
		}
	}
}

void dlpspec_subtract_remove_dc_level(const scanData *pScanData, 
		scanResults *pResults)
/**
 * Subtracts the magnitude of the detector values stored during off patterns from 
 * the detector values for all other patterns of the scan. This is used while
 * interpreting scans for the purpose of correcting for scatter from the off-state
 * mirrors of the DMD. ADC data from @p *pScanData is processed and stored in  
 * @p pResults->intensity with the off pattern values removed.
 * 
 * @param[in]      pScanData    pointer to deserialized scan data.
 * @param[out]     pResults     pointer to the buffer 
 *
 */
{
	int dc_level = 0;
	int num_black_patterns = 0;
	int res_idx, scan_idx;
    int i;

    /* Compute DC detector level during black patterns */
    for(i=0; i<pScanData->adc_data_length; i++)
	{
        if((i-pScanData->black_pattern_first)%pScanData->black_pattern_period == 0)
        {
    		dc_level += pScanData->adc_data[i];
    		num_black_patterns++;
        }
	}
	if(num_black_patterns > 0)
		dc_level /= num_black_patterns;

    /* Subtract found DC detector level from remaining measurements */
    for(res_idx=0, scan_idx=0; scan_idx < pScanData->adc_data_length;)
	{
		if((scan_idx-pScanData->black_pattern_first)
				%pScanData->black_pattern_period == 0)
		{
			scan_idx++; //skip this adc_data (black level)
		}
		else
		{
			pResults->intensity[res_idx] = pScanData->adc_data[scan_idx++] 
				- dc_level;
			res_idx++;
		}
	}
    pResults->length = res_idx;
}

DLPSPEC_ERR_CODE dlpspec_interpolate_int_wavelengths(const double *desired_nm,  
		const int num_desired, double *reference_nm, int *reference_int, 
		const int num_reference)
/**
 * Function to interpolate/extrapolate and compute reference integer intensity values based on desired wavelengths
 *
 * @param[in]       desired_nm      Pointer to list of desired wavelength (for which intensities needs to be computed)
 * @param[in]       num_desired     Size of the desired wavelength list
 * @param[in,out]   reference_nm    Pointer to list of reference wavelengths; will be changed to desired_nm upon return
 * @param[in,out]   reference_int   Pointer to intensities correpsonding to reference wavelengths; will be udpated with output upon return
 * @param[in]       num_reference     Size of the reference wavelength/intensity lists
 *
 * @return  Error code
 *
 */
{
	int i = 0, j = 0;
	int *output_intensity;
	int prev_j = 0;
	int entry_found = 0;
    int value_found = 0;
    DLPSPEC_ERR_CODE ret_val = DLPSPEC_PASS;

    if ((desired_nm == NULL) || (reference_nm == NULL) || (reference_int == NULL))
        return (ERR_DLPSPEC_NULL_POINTER);
    
    if ((num_desired <= 0) || (num_desired > ADC_DATA_LEN) || 
			(num_reference <= 0) || (num_reference > ADC_DATA_LEN))
		return (ERR_DLPSPEC_INVALID_INPUT);

    output_intensity = (int *)malloc(sizeof(int) * num_desired);
	if (output_intensity == NULL)
		return (ERR_DLPSPEC_INSUFFICIENT_MEM);
    memset(output_intensity,0,sizeof(int) * num_desired);

	/* Logic used: For points outside the range of refernece wavelengths, 
	 * compute the slope from next/previous two points
	 * and extrapolate accordingly. Interpolate other points
	 */
    for (i = 0; i < num_desired; i++)
	{
        if (desired_nm[i] == 0) //Invalid entry
        {
            ret_val = ERR_DLPSPEC_INVALID_INPUT;
            goto cleanup_and_exit;
        }

        for (j = prev_j; j < num_reference; j++)
		{
            if (reference_nm[j] == 0) //Invalid entry
                break;

			if (reference_nm[j] < desired_nm[i])  
			// Skip the entry since it is smaller than what we are looking for
				continue;
			else if (reference_nm[j] == desired_nm[i])  
			// If equal, simple use the same value
            {
				output_intensity[i] = reference_int[j];
                value_found = 1;
            }
			else
				entry_found = 1;

			break;
		}

        if(value_found == 0)
        {
            if (entry_found == 1)
            {
                if (j == 0) 
				// This entry needs to be extrapolated using the first two entries
                {
                    if ((reference_nm[j+1] - reference_nm[j]) != 0)
                        output_intensity[i] = reference_int[j] - 
							(int)((reference_nm[j] - desired_nm[i]) * 
									(reference_int[j+1] - reference_int[j])/
									(reference_nm[j+1] - reference_nm[j]));
                    else
                        output_intensity[i] = reference_int[j];
                }
                else  // Compute intensity by interpolation
                {
                    if ((reference_nm[j] - reference_nm[j-1]) != 0)
                        output_intensity[i] = reference_int[j] - 
							(int)((reference_nm[j] - desired_nm[i])/
									(reference_nm[j] - reference_nm[j-1]) * 
									(reference_int[j] - reference_int[j-1]));
                    else
                        output_intensity[i] = reference_int[j];
                }
            }
            else 
			// This entry is past the sample range, so need to extrapolate using last two entries
            {
                if ((reference_nm[j-1] - reference_nm[j-2]) != 0)
                    output_intensity[i] = reference_int[j-1] + 
						(int)((desired_nm[i] - reference_nm[j-1]) * 
								(reference_int[j-1] - reference_int[j-2])/
								(reference_nm[j-1] - reference_nm[j-2]));
                else
                    output_intensity[i] = reference_int[j-1];
            }
        }

		prev_j = j;
		entry_found = 0;
        value_found = 0;
	}

    for (i = 0; i < num_desired; i++) // Copy output to relevant lists
	{
		if (output_intensity[i] > 0)
			reference_int[i] = output_intensity[i];
		else
			reference_int[i] = 0;

		reference_nm[i] = desired_nm[i];
	}
cleanup_and_exit:
	if (output_intensity != NULL)
		free(output_intensity);

    return ret_val;
}

DLPSPEC_ERR_CODE dlpspec_interpolate_double_wavelengths(const double *desired_nm,
	   	double *reference_nm, double *reference_int, const int num_entries)
/**
 * Function to interpolate/extrapolate & compute reference double intensity values based on desired wavelengths
 *
 * @param[in]       desired_nm	    Pointer to list of desired wavelength (for which intensities needs to be computed)
 * @param[in,out]   reference_nm    Pointer to list of reference wavelengths; will be changed to desired_nm upon return
 * @param[in,out]   reference_int   Pointer to intensities correpsonding to reference wavelengths; will be udpated with output upon return
 * @param[in]       num_entries     Size of wavelength/intensity lists
 *
 * @return  Error code
 *
 */
{
	int i = 0, j = 0;
	double *output_intensity;
	int prev_j = 0;
	int entry_found = 0;
	DLPSPEC_ERR_CODE ret_val = DLPSPEC_PASS;

	if ((desired_nm == NULL) || (reference_nm == NULL) || (reference_int == NULL))
		return (ERR_DLPSPEC_NULL_POINTER);
    
	if ((num_entries == 0))
		return (ERR_DLPSPEC_INVALID_INPUT);

	output_intensity = (double *)malloc(sizeof(double) * num_entries);
	if (output_intensity == NULL)
		return (ERR_DLPSPEC_INSUFFICIENT_MEM);

	/* Logic used: For points outside the range of refernece wavelengths, 
	 * compute the slope from next/previous two points
	 * and extrapolate accordingly. Interpolate for other points
	 */
	for (i = 0; i < num_entries; i++)
	{
		if (desired_nm[i] == 0) //Invalid entry
		{
			ret_val = ERR_DLPSPEC_INVALID_INPUT;
			goto cleanup_and_exit;
		}

        for (j = prev_j; j < REF_CAL_INTERP_WAVELENGTH; j++)
		{
            if (reference_nm[j] == 0) //Invalid entry
                break;

			if (reference_nm[j] < desired_nm[i])  
			// Skip the entry since it is smaller than what we are looking for
				continue;
			else if (reference_nm[j] == desired_nm[i])  
				// If equal, simple use the same value
				output_intensity[i] = reference_int[j];
			else
				entry_found = 1;

			break;
		}

		if (entry_found == 1)
		{
			if (j == 0) 
			// This entry needs to be extrapolated using the first two entries
			{
				if ((reference_nm[j+1] - reference_nm[j]) != 0.0)
					output_intensity[i] = reference_int[j] - ((reference_nm[j] 
								- desired_nm[i]) * (reference_int[j+1] - 
									reference_int[j])/(reference_nm[j+1] - 
										reference_nm[j]));
				else
					output_intensity[i] = reference_int[j];
			}
			else  // Compute intensity by interpolation
			{
				if ((reference_int[j] - reference_int[j-1]) != 0.0)
					output_intensity[i] = reference_int[j] - ((reference_nm[j] 
								- desired_nm[i])/(reference_nm[j] - 
									reference_nm[j-1]) * (reference_int[j] - 
										reference_int[j-1]));
				else
					output_intensity[i] = reference_int[j];
			}
		}
		else 
    // This entry is past the sample range, so need to extrapolate using last two entries
		{
			if ((reference_nm[j-1] - reference_nm[j-2]) != 0)
				output_intensity[i] = reference_int[j-1] + ((desired_nm[i] - 
							reference_nm[j-1]) * (reference_int[j-1] - 
								reference_int[j-2])/(reference_nm[j-1] - 
									reference_nm[j-2]));
			else
				output_intensity[i] = reference_int[j-1];
		}

		prev_j = j;
		entry_found = 0;
	}

	for (i = 0; i < num_entries; i++) // Copy output to relevant lists
	{
		if (output_intensity[i] > 0)
			reference_int[i] = output_intensity[i];
		else
			reference_int[i] = 0;

		reference_nm[i] = desired_nm[i];
	}

cleanup_and_exit:

	if (output_intensity != NULL)
		free(output_intensity);

	return ret_val;
}

DLPSPEC_ERR_CODE dlpspec_interpolate_double_positions(const double *desired_pos,
	   	double *reference_pos, double *modified_int, const int num_modified, const int num_reference)
/**
 * Function to interpolate/extrapolate & compute reference double intensity values based on desired wavelengths
 *
 * @param[in]       desired_pos	    Pointer to list of desired wavelength (for which intensities needs to be computed)
 * @param[in,out]   reference_pos   Pointer to list of reference wavelengths; will be changed to desired_pos upon return
 * @param[in,out]   modified_int   Pointer to intensities correpsonding to reference wavelengths; will be udpated with output upon return
 * @param[in]       num_modified     Size of modified position list
 * @param[in]       num_reference   Size of reference position list
 * 
 * @return  Error code
 *
 */
{
	int i = 0, j = 0;
	double *output_intensity;
	int prev_j = 0;
	int entry_found = 0;
	DLPSPEC_ERR_CODE ret_val = DLPSPEC_PASS;

	if ((desired_pos == NULL) || (reference_pos == NULL) || (modified_int == NULL))
		return (ERR_DLPSPEC_NULL_POINTER);
    
	if ((num_modified == 0))
		return (ERR_DLPSPEC_INVALID_INPUT);

	output_intensity = (double *)malloc(sizeof(double) * num_modified);
	if (output_intensity == NULL)
		return (ERR_DLPSPEC_INSUFFICIENT_MEM);

	/* Logic used: For points outside the range of refernece positions, 
	 * compute the slope from next/previous two points
	 * and extrapolate accordingly. Interpolate for other points
	 */
	for (i = 0; i < num_modified; i++)
	{
		if (desired_pos[i] == 0) //Invalid entry
		{
			ret_val = ERR_DLPSPEC_INVALID_INPUT;
			goto cleanup_and_exit;
		}

        for (j = prev_j; j < num_reference; j++)
		{
            if (reference_pos[j] == 0) //Invalid entry
                break;

			if (reference_pos[j] < desired_pos[i])  
			// Skip the entry since it is smaller than what we are looking for
				continue;
			else if (reference_pos[j] == desired_pos[i])  
				// If equal, simple use the same value
				output_intensity[i] = modified_int[j];
			else
				entry_found = 1;

			break;
		}

		if (entry_found == 1)
		{
			if (j == 0) 
			// This entry needs to be extrapolated using the first two entries
			{
				if ((reference_pos[j+1] - reference_pos[j]) != 0.0)
					output_intensity[i] = modified_int[j] - ((reference_pos[j] 
								- desired_pos[i]) * (modified_int[j+1] - 
									modified_int[j])/(reference_pos[j+1] - 
										reference_pos[j]));
				else
					output_intensity[i] = modified_int[j];
			}
			else  // Compute intensity by interpolation
			{
				if ((modified_int[j] - modified_int[j-1]) != 0.0)
					output_intensity[i] = modified_int[j] - ((reference_pos[j] 
								- desired_pos[i])/(reference_pos[j] - 
									reference_pos[j-1]) * (modified_int[j] - 
										modified_int[j-1]));
				else
					output_intensity[i] = modified_int[j];
			}
		}
		else 
    // This entry is past the sample range, so need to extrapolate using last two entries
		{
			if ((reference_pos[j-1] - reference_pos[j-2]) != 0)
				output_intensity[i] = modified_int[j-1] + ((desired_pos[i] - 
							reference_pos[j-1]) * (modified_int[j-1] - 
								modified_int[j-2])/(reference_pos[j-1] - 
									reference_pos[j-2]));
			else
				output_intensity[i] = modified_int[j-1];
		}

		prev_j = j;
		entry_found = 0;
	}

	for (i = 0; i < num_modified; i++) // Copy output to relevant lists
	{
		if (output_intensity[i] > 0)
			modified_int[i] = output_intensity[i];
		else
			modified_int[i] = 0;

		reference_pos[i] = desired_pos[i];
	}

cleanup_and_exit:

	if (output_intensity != NULL)
		free(output_intensity);

	return ret_val;
}

DLPSPEC_ERR_CODE dlpspec_compute_from_references(int ref_in1, int ref_in2, 
		int ref_out1, int ref_out2, int in, int *out)
/**
 * Function to estimate an integeer value by interpolation/extrapolation of
 * two input references and their corresponding output values
 *
 * @param[in]   ref_in1/ref_in2 	Reference inputs
 * @param[in]   ref_out1/ref_out2   Reference outputs
 * @param[in]   in  			    Input value to estimate
 * @param[out]  out				    Pointer to hold estimated output value
 *
 * @return      Error code
 *
 */
{
    DLPSPEC_ERR_CODE ret_val = (DLPSPEC_PASS);
    
    if((in < ref_in1) && (in < ref_in2))	//Interpolate using ref 1
    {
        if ((ref_in2 - ref_in1) != 0)
            *out = ref_out1 - (int)((ref_in1 - in) * (ref_out2 - ref_out1)/
					(ref_in2 - ref_in1));
        else
            ret_val = ERR_DLPSPEC_INVALID_INPUT;

    }
    else if ((in > ref_in1) && (in > ref_in2))
    {
        if ((ref_in2 - ref_in1) != 0)
            *out = ref_out2 + (int)((in - ref_in2) * (ref_out2 - ref_out1)/
					(ref_in2 - ref_in1));
        else
            ret_val = ERR_DLPSPEC_INVALID_INPUT;
    }
    else
    {
        if ((ref_in2 - ref_in1) != 0)
            *out = ref_out2 - (int)((ref_in2 - in) * (ref_out2 - ref_out1) / 
					(ref_in2 - ref_in1));
        else
            ret_val = ERR_DLPSPEC_INVALID_INPUT;
    }

    return ret_val;
}

DLPSPEC_ERR_CODE dlpspec_matrix_mult(const double *a, const double*b, 
		double*res, int p, int q, int r)
/**
 * Matrix multiplication function.
 *
 * @param[in]   a   multiplier matrix
 * @param[in]   b   multiplicand matrix
 * @param[out]  res product matrix
 * @param[in]   p   number of rows in matrix a
 * @param[in]   q   number of columns in matrix a
 * @param[in]   r   number of columns in matrix b
 *
 * @return      Error code
 *
 */
{
    double sum=0;
    int i,j,k;

	if ((a == NULL) || (b == NULL) || (res == NULL))
		return (ERR_DLPSPEC_NULL_POINTER);

    for ( i = 0 ; i < p ; i++ )
    {
        for ( j = 0 ; j < r ; j++ )
        {
            for ( k = 0 ; k < q ; k++ )
            {
                sum = sum + a[i*q+k]*b[k*r+j];
            }

            res[i*r+j] = sum;
            sum = 0;
        }
    }

    return (DLPSPEC_PASS);
}


DLPSPEC_ERR_CODE dlpspec_matrix_transpose(double *a, double *res, int row,int col)
/**
 * Matrix transpose function.
 *
 * @param[in]   a       input matrix
 * @param[out]  res     transposed matrix
 * @param[in]   row     number of rows in matrix a
 * @param[in]   col     number of columns in matrix a
 *
 * @return  Error code
 *
 */
{
    int i,j;

    if ((a == NULL) || (res == NULL))
    	return (ERR_DLPSPEC_NULL_POINTER);

    for (i=0;i<row;++i)
        for (j=0;j<col;++j)
            res[j*row+i]=a[i*col + j];

    return (DLPSPEC_PASS);
}

int32_t dlpspec_scan_cfg_compare(const slewScanConfig *pCfg1, 
		const slewScanConfig *pCfg2)
/**
 * Compares two slewScanConfig structures and retuns DLPSPEC_PASS if they are the same and 
 * ERR_DLPSPEC_FAIL if they differ.
 *
 * @param[in]   pCfg1   config 1
 * @param[in]   pCfg2   config 2
 *
 * @return DLPSPEC_PASS if same or ERR_DLPSPEC_FAIL if different
 *
 */
{
	int i;

	if(pCfg1->head.num_sections != pCfg2->head.num_sections)
		return ERR_DLPSPEC_FAIL;

	for(i=0; i<pCfg1->head.num_sections; i++)
	{
		if(pCfg1->section[i].section_scan_type != pCfg2->section[i].section_scan_type)
			return ERR_DLPSPEC_FAIL;
		if(pCfg1->section[i].width_px != pCfg2->section[i].width_px)
			return ERR_DLPSPEC_FAIL;
		if(pCfg1->section[i].wavelength_start_nm != pCfg2->section[i].wavelength_start_nm)
			return ERR_DLPSPEC_FAIL;
		if(pCfg1->section[i].wavelength_end_nm != pCfg2->section[i].wavelength_end_nm)
			return ERR_DLPSPEC_FAIL;
		if(pCfg1->section[i].num_patterns != pCfg2->section[i].num_patterns)
			return ERR_DLPSPEC_FAIL;
	}

	return DLPSPEC_PASS;
}

DLPSPEC_ERR_CODE dlpspec_valid_configs_to_interp(const slewScanConfig *pCfg1, 
		const slewScanConfig *pCfg2)
/**
 * Checks if pCfg1 structure is fully covered by a pCfg2 and ensures pCfg2 must be a single sectioned scan
 * used to ensure that reference interpolation will be valid when the reference scan was taken with pCfg2
 * and the sample scan was taken with pCfg1. 
 *
 * @param[in]   pCfg1   config 1
 * @param[in]   pCfg2   config 2
 *
 * @return DLPSPEC_PASS if it is valid
 * @return ERR_DLPSPEC_FAIL if invalid
 *
 */
{
	int i;

	if(pCfg2->head.num_sections != 1)
		return ERR_DLPSPEC_FAIL;

	for(i=0; i<pCfg1->head.num_sections; i++)
	{
		if(pCfg1->section[i].wavelength_start_nm < pCfg2->section[0].wavelength_start_nm)
			return ERR_DLPSPEC_FAIL;
		if(pCfg1->section[i].wavelength_end_nm > pCfg2->section[0].wavelength_end_nm)
			return ERR_DLPSPEC_FAIL;
	}

	return DLPSPEC_PASS;
}
