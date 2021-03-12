/**
 *
 * This class provides EVM specific functions.
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
*/

#include <qdatetime.h>
#include "mainwindow.h"
#include "API.h"
#include "refCalMatrix.h"
#include "evm.h"

extern FilePath filepath;

EVM::EVM()
{
    m_RefCalMatrixBlob = (void *)malloc(REF_CAL_MATRIX_BLOB_SIZE);
    m_RefCalDataBlob = (void *)malloc(SCAN_DATA_BLOB_SIZE);
}

EVM::~EVM()
{
    if(m_RefCalMatrixBlob  != NULL)
    {
        free(m_RefCalMatrixBlob);
        m_RefCalMatrixBlob = NULL;
    }
    if(m_RefCalDataBlob != NULL)
    {
        free(m_RefCalDataBlob);
        m_RefCalDataBlob = NULL;
    }
}

int EVM::ApplyScanCfgtoDevice(uScanConfig *pCfg)
	/**
	 * This function applies the input scanConfig to the TIVA
	 * @param pCfg - I - current scan configuration which should be applied to the TIVA
	 * @return  < 0 = FAIL; else number of patterns generated for this scan config.
	 *
	 */
{
    size_t bufferSize;
    void *pBuffer;
    int ret;

    ret = dlpspec_get_scan_config_dump_size(pCfg, &bufferSize);
    if(ret == DLPSPEC_PASS)
    {
        pBuffer = malloc(bufferSize);
        if(pBuffer == NULL)
        {
            return FAIL;
        }
        ret = dlpspec_scan_write_configuration(pCfg, pBuffer, bufferSize);

        if(ret == DLPSPEC_PASS)
            ret = NNO_ApplyScanConfig(pBuffer, bufferSize);

        free(pBuffer);
    }

    return ret;
}

int EVM::GenCalibPatterns(CALIB_SCAN_TYPES scan_type)
/**
 * This function commands TIVA to generate calibration patterns
 * @param scan_type - I - type of calibration
 * @return  < 0 = FAIL;
 *
 */
{
    int ret_val;

    if((scan_type < SLIT_ALIGN_SCAN) || (scan_type > CALIB_SCAN_TYPES_MAX))
            return FAIL;

    ret_val = NNO_GenCalibPatterns(scan_type);

    if(ret_val <= 0)
        return FAIL;

    if( (scan_type == LEFT_DMD_TOP_SCAN)  || (scan_type == RIGHT_DMD_TOP_SCAN))
        NNO_setScanSubImage(DMD_TOP_SCAN_START_Y, DMD_TOP_MID_BOT_SCAN_HEIGHT);
    else if( (scan_type == LEFT_DMD_MID_SCAN)  || (scan_type == RIGHT_DMD_MID_SCAN))
        NNO_setScanSubImage(DMD_MID_SCAN_START_Y, DMD_TOP_MID_BOT_SCAN_HEIGHT);
    else if( (scan_type == LEFT_DMD_BOT_SCAN)  || (scan_type == RIGHT_DMD_BOT_SCAN))
        NNO_setScanSubImage(DMD_BOT_SCAN_START_Y, DMD_TOP_MID_BOT_SCAN_HEIGHT);

    return ret_val;

}

int EVM::FetchRefCalData(void)
/**
 * This function gets the reference calibration data stored in the EVM
 * @return  < 0 = FAIL;
 *
 */
{
    int refCalSize = NNO_GetFileSizeToRead(NNO_FILE_REF_CAL_DATA);

    if(NNO_GetFile((unsigned char *)m_RefCalDataBlob, refCalSize) == refCalSize)
    {
        return PASS;
    }
    else
    {
        return FAIL;
    }
}

int EVM::FetchRefCalMatrix(void)
/**
 * This function gets the Reference Calibration Matrix stored in the TIVA
 * @return  < 0 = FAIL;
 *
 */
{
    char ser_num[NANO_SER_NUM_LEN+1];
    QString refCalMatrixFileName = "refCalMatrix_";
    QString refCalMatrixFileNameFull;
    int refCalSize = NNO_GetFileSizeToRead(NNO_FILE_REF_CAL_MATRIX);

    if(NNO_GetFile((unsigned char *)m_RefCalMatrixBlob, refCalSize) == refCalSize)
    {
        /* Save refcal data in the PC tool's config directory for later use when not connected
         * to the EVM */
        NNO_GetSerialNumber(ser_num);
        refCalMatrixFileName.append(ser_num);
        refCalMatrixFileName.append(".dat");
        refCalMatrixFileNameFull = filepath.GetconfigDir().absoluteFilePath(refCalMatrixFileName);

        if (QFile::exists(refCalMatrixFileNameFull))
        {
            QFile::remove(refCalMatrixFileNameFull);
        }
        QFile file(refCalMatrixFileNameFull);
        file.open(QIODevice::ReadWrite);
        QDataStream out_data(&file);

        out_data.writeRawData((const char *)m_RefCalMatrixBlob, REF_CAL_MATRIX_BLOB_SIZE);
        file.close();
        return PASS;
    }
    else
    {
        return FAIL;
    }
}

void *EVM::GetRefCalMatrixBlob(char *p_ser_num_str)
{
    QString refCalMatrixFileName = "refCalMatrix_";
    QString refCalMatrixFileNameFull;
    refCalMatrix tmpRefCalMatrix;

    refCalMatrixFileName.append(p_ser_num_str);
    refCalMatrixFileName.append(".dat");
    refCalMatrixFileNameFull = filepath.GetconfigDir().absoluteFilePath(refCalMatrixFileName);

    if (QFile::exists(refCalMatrixFileNameFull)) //read from file
    {
        QFile file(refCalMatrixFileNameFull);
        if(file.open(QIODevice::ReadOnly))
        {
            file.read((char *)m_RefCalMatrixBlob, REF_CAL_MATRIX_BLOB_SIZE);
            file.close();
        }
    }
    else    //give default ref cal matrix
    {
        memcpy(tmpRefCalMatrix.width, refCalMatrix_widths, sizeof(uint8_t)*REF_CAL_INTERP_WIDTH);
        memcpy(tmpRefCalMatrix.wavelength, refCalMatrix_wavelengths, sizeof(double)*REF_CAL_INTERP_WAVELENGTH);
        memcpy(tmpRefCalMatrix.ref_lookup, refCalMatrix_intensities, sizeof(uint16_t)*REF_CAL_INTERP_WIDTH*REF_CAL_INTERP_WAVELENGTH);

        dlpspec_calib_write_ref_matrix(&tmpRefCalMatrix, m_RefCalMatrixBlob, REF_CAL_MATRIX_BLOB_SIZE);

    }

    return m_RefCalMatrixBlob;
}
