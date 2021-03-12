/*****************************************************************************
**
**  Copyright (c) 2015 Texas Instruments Incorporated.
**  ALL RIGHTS RESERVED
**
******************************************************************************/

#include "spectrum.h"
#include <math.h>
#include <QString>
#include "dlpspec_scan.h"
#include "string.h"
#include "API.h"
#include "dlpspec_calib.h"
#include "dlpspec_util.h"
#include "version.h"
#include "evm.h"
#include <QTimer>
#include <QTime>
#include <QThread>
#include <QFile>
#include <QFileInfo>
#include "dlpspec_helper.h"
#include "mainwindow.h"

extern EVM evm;
extern FilePath filepath;
extern qint64 lastScanTimeMS;

QVector<double> Spectrum::GetWavelengths()
/**
 * This function is for accessing the wavelength vector from data last
 * set using SetData()
 *
 */
{
    QVector<double> out;
    for (int i = 0; i < m_scanResults.length; ++i)
    {
        out.append(m_scanResults.wavelength[i]);
    }
    return out;
}

QVector<double> Spectrum::GetIntensities()
/**
 * This function is for accessing the intensity vector from data last
 * set using SetData()
 */
{
    QVector<double> out;
    for (int i = 0; i < m_scanResults.length; ++i)
    {
        out.append(m_scanResults.intensity[i]);
    }
    return out;
}

QVector<double> Spectrum::GetRefIntensities()
/**
 * This function is for accessing the reference intensity vector from data last
 * set using SetData()
 */
{
    QVector<double> out;
    for (int i = 0; i < m_referenceResults.length; ++i)
    {
        out.append(m_referenceResults.intensity[i]);
    }
    return out;
}

QVector<double> Spectrum::GetAbsorbance()
/**
 * This function is for accessing the absorbance vector from data last
 * set using SetData()
 */
{
    QVector<double> out;
    double absorbance;

    m_maxAbsorbance = 0;
    for(int i = 0; i < m_scanResults.length; i++ )
    {
        if(m_referenceResults.length > i)
        {
            absorbance = COMPUTE_ABSORPTION_VAL(m_scanResults.intensity[i], m_referenceResults.intensity[i]);
            out.append(absorbance);
            if(absorbance > m_maxAbsorbance)
                m_maxAbsorbance = absorbance;
        }
    }
    return out;
}

//get reflectance
QVector<double> Spectrum::GetReflectance()
/**
 * This function is for accessing the absorbance vector from data last
 * set using SetData()
 */
{
    QVector<double> out;
    double reflectance;

    //m_maxReflectance = 0;
    for(int i = 0; i < m_scanResults.length; i++ )
    {
        if(m_referenceResults.length > i)
        {
            if( m_referenceResults.intensity[i]==0)
            {
                reflectance=0;
            }
            else
            {
                reflectance = COMPUTE_REFLECTANCE_VAL(m_scanResults.intensity[i], m_referenceResults.intensity[i]);
            }

            out.append(reflectance);
            //if(reflectance > m_maxReflectance)
            //  m_maxReflectance = reflectance;
        }
    }
    return out;
}
QString Spectrum::GetScanTimeStamp()
/**
 * This function returns the timestamp of scan data last set using SetData()
 */
{
    QString time_stamp_str = "";

    time_stamp_str.sprintf("20%02d%02d%02d_%02d%02d%02d", m_scanResults.year, m_scanResults.month,
            m_scanResults.day, m_scanResults.hour, m_scanResults.minute, m_scanResults.second);

    return time_stamp_str;
}

QString Spectrum::GetReferenceTimeStamp()
/**
 * This function returns the timestamp of reference data last set using SetData()
 *
 */
{
    QString time_stamp_str = "";

    time_stamp_str.sprintf("20%02d/%02d/%02d @ %02d:%02d:%02d", m_referenceResults.year, m_referenceResults.month,
            m_referenceResults.day, m_referenceResults.hour, m_referenceResults.minute, m_referenceResults.second);

    return time_stamp_str;
}

int Spectrum::ReadReferenceFromFile(void *pRefDataBlob, const uScanConfig *pCfg)
/**
 * This function reads the Reference Data form the specified file and saves it to the class variable
 *
 * @param pRefDataBlob - O - reference data blob read from the file
 * @param pCfg - I - scan config whose corresponding reference data has to be found and read
 *
 * @return PASS or FAIL
 */
{
    QString refFileName = GetReferenceFileName(pCfg);
    refFileName.append(".dat");
    QString refFileNameFull = filepath.GetconfigDir().absoluteFilePath(refFileName);
    QFile file(refFileNameFull);

    if(QFile::exists(refFileNameFull) )
    {
        if(file.open(QIODevice::ReadOnly))
        {
            file.read((char *)pRefDataBlob, SCAN_DATA_BLOB_SIZE);
            file.close();
            return PASS;
        }
    }
    return FAIL;
}

int Spectrum::ReadSavedScanFromFile(QString FileName)
/**
 * This function reads the scan Data form the specified file and saves it to the class variable
 *
 * @param FileName - I - the name of the file from which the scan Data has to be read
 */
{
    QFile file(FileName);
    QFileInfo finfo(FileName);
    int fsize = finfo.size();
    int ret_val = PASS;
    size_t blob_size;

    if(fsize ==( 2 * SCAN_DATA_BLOB_SIZE) )
        blob_size = SCAN_DATA_BLOB_SIZE;
    else if(fsize ==( 2 * OLD_SCAN_DATA_BLOB_SIZE) )
        blob_size = OLD_SCAN_DATA_BLOB_SIZE;
    else
        return FAIL;


    void *pBuffer = malloc(blob_size);
    if(pBuffer == NULL)
    {
        return FAIL;
    }
    void *pBufferRef = malloc(blob_size);
    if(pBufferRef == NULL)
    {
        free(pBuffer);
        return FAIL;
    }
    //read the reference if it is appended to the output file
    if(file.open(QIODevice::ReadOnly))
    {
        file.read((char *)pBuffer, blob_size);
        file.read((char *)pBufferRef, blob_size);

        if(SetData(pBuffer, pBufferRef) != PASS)
        {
            ret_val = FAIL;
        }
        file.close();
    }
    free(pBuffer);
    free(pBufferRef);
    return ret_val;


}
void Spectrum::ResetPlot()
/**
 * This function clears the vector of the y-axis points in the CustomePlot instance
 * whenever overlay check-box in GUI is checked, all the different y-axis will ve stored in a vector
 * this function clears that vector
 *
 */
{
    m_plot.ResetValues();
}

bool Spectrum::Plot(PLOT_TYPE type, QGraphicsScene *scene)
/**
 * This function sets the Current Refrence Data blob to the Spectrum class variable
 *
 * @param type - I - the PLOT_TYPE indicating whether it is intensity/absorbance/reflectance
 * @param scene - I - QGraphicsScene where the points are marked and lines are drawn
 */
{
    scene->clear();
    QVector<double> vectorIntensity;
    QVector<double> vectorSectionBordersStart;
    QVector<double> vectorSectionBordersEnd;
    for(int i = 0; i < m_scanResults.cfg.head.num_sections; i++)
    {
        vectorSectionBordersStart.push_back(m_scanResults.cfg.section[i].wavelength_start_nm);
        vectorSectionBordersEnd.push_back(m_scanResults.cfg.section[i].wavelength_end_nm);
    }

    if(type == PLOT_INTENSITY)
    {
        double maxIntensity = 0;
        for(int i = 0; i < m_scanResults.length; i++ )
        {
            if(m_scanResults.intensity[i] > maxIntensity)
                maxIntensity = m_scanResults.intensity[i];

            vectorIntensity.append(m_scanResults.intensity[i]);
        }

        m_plot.DemarcateSections(vectorSectionBordersStart,vectorSectionBordersEnd,m_scanResults.cfg.head.num_sections);
        return m_plot.Plot(GetWavelengths(),vectorIntensity, maxIntensity, scene);

    }

    else if(type == PLOT_ABSORBANCE)
    {
        QVector<double> vectorAbsorbance = GetAbsorbance();
        m_plot.DemarcateSections(vectorSectionBordersStart,vectorSectionBordersEnd,m_scanResults.cfg.head.num_sections);
        return m_plot.Plot(GetWavelengths(),vectorAbsorbance, m_maxAbsorbance, scene);
    }

    else if(type == PLOT_REFLECTANCE)
    {

        QVector<double> vectorReflectance;
        double maxReflectance = 0;
        double reflectance;

        for(int i = 0; i < m_scanResults.length; i++ )
        {
            if(m_referenceResults.length > i)
            {
                if(m_referenceResults.intensity[i] != 0)
                 reflectance = COMPUTE_REFLECTANCE_VAL(m_scanResults.intensity[i], m_referenceResults.intensity[i]);
                else
                   reflectance = 0;

                vectorReflectance.append(reflectance);

                if(reflectance > maxReflectance)
                    maxReflectance = reflectance;
            }
        }
        m_plot.DemarcateSections(vectorSectionBordersStart,vectorSectionBordersEnd,m_scanResults.cfg.head.num_sections);
        return m_plot.Plot(GetWavelengths(),vectorReflectance, maxReflectance, scene);
    }

    return false;
}

QString Spectrum::GetReferenceFileName(const uScanConfig *pCfg)
{
    char name[50];
    QString scanName = "ref_";
    snprintf(name, NANO_SER_NUM_LEN, pCfg->scanCfg.ScanConfig_serial_number);
    char type_id;
    uint16 start_nm;
    uint16 end_nm;
    uint16 num_patterns = 0;
    uint8 width_px;
    int i;

    scanName.append(name);

    if(pCfg->scanCfg.scan_type == SLEW_TYPE)
    {
        if(pCfg->slewScanCfg.head.num_sections == 1)
        {
            type_id = (pCfg->slewScanCfg.section[0].section_scan_type == COLUMN_TYPE) ? 'c' : 'h';
            start_nm = pCfg->slewScanCfg.section[0].wavelength_start_nm;
            end_nm = dlpspec_scan_slew_get_end_nm(&pCfg->slewScanCfg);
            width_px = pCfg->slewScanCfg.section[0].width_px;
            num_patterns = dlpspec_scan_slew_get_num_patterns(&pCfg->slewScanCfg);
            sprintf(name, "_%04d_%04d_%03d_%03d_%c",
                    start_nm,
                    end_nm,
                    num_patterns,
                    width_px,
                    type_id);
        }
        else
        {
            for(i=0; i<pCfg->slewScanCfg.head.num_sections; i++)
            {
                start_nm = pCfg->slewScanCfg.section[i].wavelength_start_nm;
                end_nm = pCfg->slewScanCfg.section[i].wavelength_end_nm;
                width_px = pCfg->slewScanCfg.section[i].width_px;
                num_patterns = pCfg->slewScanCfg.section[i].num_patterns;
                sprintf(name, "_%04d_%04d_%03d_%03d",
                        start_nm,
                        end_nm,
                        num_patterns,
                        width_px);
                scanName.append(name);
            }
            sprintf(name, "_s");
        }
    }
    else
    {
        type_id = (pCfg->scanCfg.scan_type == COLUMN_TYPE) ? 'c' : 'h';
        sprintf(name, "_%04d_%04d_%03d_%03d_%c",
                pCfg->scanCfg.wavelength_start_nm,
                pCfg->scanCfg.wavelength_end_nm,
                pCfg->scanCfg.num_patterns,
                pCfg->scanCfg.width_px,
                type_id);
    }
    scanName.append(name);
    return scanName;
}

void Spectrum::SaveReferenceToFile()
/**
     *This function saves the current Reference Data from the spectrum into the
     * corresponding reference file in both .dat and .csv formats
    */
{
    char ser_num[NANO_SER_NUM_LEN];

    NNO_GetSerialNumber(ser_num);
    strncpy(m_referenceResults.cfg.head.ScanConfig_serial_number, ser_num, NANO_SER_NUM_LEN);
    QString scanName = GetReferenceFileName((const uScanConfig *)&m_referenceResults.cfg);

    // scanName.append(GetScanTimeStamp());
    QString refFileNameRel =  QString("%1.csv").arg(scanName);
    QString refFileName = filepath.GetconfigDir().absoluteFilePath(refFileNameRel);
    QString refFileNameBinaryRel =  QString("%1.dat").arg(scanName);
    QString refFileNameBinary = filepath.GetconfigDir().absoluteFilePath(refFileNameBinaryRel);

    if (QFile::exists(refFileName) || QFile::exists(refFileNameBinary) )
    {
        QFile::remove(refFileName);
        QFile::remove(refFileNameBinary);
    }

    //save the refernece in binary file
    QFile file(refFileNameBinary);
    file.open(QIODevice::ReadWrite);
    QDataStream out_data(&file);   // we will serialize the data into the file

    out_data.writeRawData((const char *)m_curReferenceDataBlob, SCAN_DATA_BLOB_SIZE);
    file.close();

    //saving to csv file
    QFile RefFile(refFileName);

    RefFile.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&RefFile);

    //setting the timeStamp on reference file
    int year,month, day;
    int hour,minute, second;

    QDateTime current = QDateTime::currentDateTime();

    year = current.date().year();
    month = current.date().month();
    day = current.date().day();

    hour = current.time().hour();
    minute = current.time().minute();
    second = current.time().second();

    QString line;

    line.sprintf("%d/%d/%d @ %d:%d:%d,,",day,month,year,hour,minute,second);
    out << line << "\n";

    QVector<double> intensity;// = this->GetAveragedIntensities();

    for(int i = 0; i < m_referenceResults.length; i++ )
        intensity.push_back(m_referenceResults.intensity[i]);

    for(int i = 0 ; i < intensity.size(); i++)
    {
        out << intensity[i] << "\n" ;
    }

    RefFile.close();
}

int Spectrum::SetData(void* pScanDataBlob, void *pRefDataBlob)
/**
 * This function takes scan data and reference data as serialzed blobs, interprets them
 * using spectrum library APIs and saves for future use.
 *
 * @param pScanDataBlob - I - pointer to scanData blob; if NULL it will continue to use the last set value
 * @param pRefDataBlob - I - pointer to reference data blob; if NULL it will continue to use the last set value
 */
{
    uScanData *pData;
    DLPSPEC_ERR_CODE ret_val;
    void *pCopyBuff;

    if(pScanDataBlob != NULL)
    {
        memcpy(m_curScanDataBlob, pScanDataBlob, SCAN_DATA_BLOB_SIZE);
        if(dlpspec_scan_interpret(pScanDataBlob, SCAN_DATA_BLOB_SIZE, &m_scanResults)!= PASS)
            return FAIL;
    }

    if(pRefDataBlob != NULL)
    {
        memcpy(m_curReferenceDataBlob, pRefDataBlob, SCAN_DATA_BLOB_SIZE);

        pCopyBuff = (void *)malloc(SCAN_DATA_BLOB_SIZE);

        if(pCopyBuff == NULL)
            return (ERR_DLPSPEC_INSUFFICIENT_MEM);

        memcpy(pCopyBuff, pRefDataBlob, SCAN_DATA_BLOB_SIZE);

        ret_val = dlpspec_scan_read_data(pCopyBuff, SCAN_DATA_BLOB_SIZE);
        if(ret_val < 0)
        {
            free(pCopyBuff);
            return ret_val;
        }

        pData = (uScanData *)pCopyBuff;
        if(PASS != dlpspec_scan_interpReference(m_curReferenceDataBlob, SCAN_DATA_BLOB_SIZE,
                                                evm.GetRefCalMatrixBlob(pData->data.serial_number),
                                                REF_CAL_MATRIX_BLOB_SIZE, &m_scanResults, &m_referenceResults))
        {
            free(pCopyBuff);
            return FAIL;
        }
        free(pCopyBuff);
    }
    return PASS;
}

int Spectrum::SetInterpretData(void* pScanDataBlob)
/**
 * This function takes scan interpreted data as serialzed blobs and saves for future use.
 *
 * @param pScanDataBlob - I - pointer to scanInterpretData blob; if NULL it will continue to use the last set value
 */
{
	int i, length;
    unsigned char *pScanDataBlobAddition;
	
    if(pScanDataBlob != NULL)
    {
       length = *((int *) pScanDataBlob);
        pScanDataBlobAddition = (unsigned char *) pScanDataBlob + sizeof(int);
        pScanDataBlob = pScanDataBlobAddition;
		for ( i = 0; i < length; i++ ) {
            m_scanResults.wavelength[i] = *((double *) pScanDataBlob);
            pScanDataBlobAddition = (unsigned char *) pScanDataBlob + sizeof(double);
            pScanDataBlob = pScanDataBlobAddition;
            m_scanResults.intensity[i] = *((int *) pScanDataBlob);
            pScanDataBlobAddition = (unsigned char *) pScanDataBlob + sizeof(int);
            pScanDataBlob = pScanDataBlobAddition;
        }
    }

    return PASS;
}

int Spectrum::SaveToFile()
/**
 * This function saves the current ScanData and RefernceData in a file
 * RefernceData is appended at the end of the file
 *
 */
{
    QString scanName;
    if(m_fileName == "")
    {
        scanName = m_prefix;
        uint32 scanIndexCounter;
        uint32 sessionScanCounter;
        char id_str[7];

    // make sure name string terminates at SCAN_NAME_TAG_SIZE and ignores any stray characters beyond this
    m_scanResults.scan_name[SCAN_NAME_TAG_SIZE] = '\0';

        scanName.append(m_scanResults.scan_name);
        scanIndexCounter = m_scanResults.scanDataIndex >> 16;
        sessionScanCounter = m_scanResults.scanDataIndex & 0xffff;
        snprintf(id_str, 7, "%03d%03d", scanIndexCounter, sessionScanCounter);
        scanName.append("_");
        scanName.append(id_str);
        scanName.append("_");
        scanName.append(GetScanTimeStamp());
    }
    else
        scanName = m_fileName;
    if(savebinary)
    {
        QString fileNameRelBinary = QString("%1.dat").arg(scanName);
        QString fileNameBinary = filepath.GetsaveScanSettingsDir().absoluteFilePath(fileNameRelBinary);

        QFile file(fileNameBinary);
        file.open(QIODevice::ReadWrite);
        QDataStream out_data(&file);   // we will serialize the data into the file

        out_data.writeRawData((const char *)m_curScanDataBlob, SCAN_DATA_BLOB_SIZE);
        out_data.writeRawData((const char *)m_curReferenceDataBlob, SCAN_DATA_BLOB_SIZE);

        file.close();
    }




    //saving to CSV file

    SaveToCSV(scanName);
    SavetoJCamp(scanName);
    return PASS;
}

//save csv header
//ifJCAMP value is directly passed while calling this function, as true or false  for JCAMP files or other(CSV) files respectively
void Spectrum::Saveheader(QTextStream *out,bool ifJCAMP)
{
    int year,month, day;
    int hour,minute, second;
    year = m_scanResults.year;
    month = m_scanResults.month;
    day = m_scanResults.day;
    hour = m_scanResults.hour;
    minute = m_scanResults.minute;
    second = m_scanResults.second;
    QString line;
    uint32 num_patterns = 0;
    QString a;
    if(ifJCAMP)
    {
        a="##";
    }
    else
    {
        a="";
    }

    line.sprintf("Method:,");
    *out <<a<< line << m_scanResults.cfg.head.config_name << ",,\n";

    line.sprintf("Host Date-Time:,%d/%d/%d @ %d:%d:%d,,",day,month,year,hour,minute,second);
    *out << a<< line << "\n";

    //CR 18551 - KVs
    if(m_scanResults.length > 0)
    {

        for(int i=0; i < m_scanResults.cfg.head.num_sections; i++)
            num_patterns += m_scanResults.cfg.section[i].num_patterns;


        *out << a<<"Header Version:," << m_scanResults.header_version << ",,\n";

        *out << a<<"System Temp (C):," << m_scanResults.system_temp_hundredths/100.0 << ",,\n";

        *out << a<<"Detector Temp (C):," << m_scanResults.detector_temp_hundredths/100.0 << ",,\n";

        *out << a<<"Humidity (%):," << m_scanResults.humidity_hundredths/100.0 << ",,\n";

        *out <<a<< "Lamp PD:," << m_scanResults.lamp_pd << ",,\n";

        *out << a<<"Shift Vector Coefficients:," << m_scanResults.calibration_coeffs.ShiftVectorCoeffs[0] << "," << m_scanResults.calibration_coeffs.ShiftVectorCoeffs[1] << "," << m_scanResults.calibration_coeffs.ShiftVectorCoeffs[2] << ",,\n";

        *out <<a<< "Pixel to Wavelength Coefficients:,"  << m_scanResults.calibration_coeffs.PixelToWavelengthCoeffs[0] << "," << m_scanResults.calibration_coeffs.PixelToWavelengthCoeffs[1] << "," << m_scanResults.calibration_coeffs.PixelToWavelengthCoeffs[2] << ",,\n";

        *out << a<<"Serial Number:," << m_scanResults.serial_number << ",,\n";

        *out << a<<"Scan Config Name:," << m_scanResults.cfg.head.config_name << ",,\n";


        if (m_scanResults.cfg.head.scan_type == 0)
        {

            *out << a<<"Scan Config Type:," << "Column" << ",,\n";

            *out << a<<"Start wavelength (nm):,"  << m_scanResults.cfg.section[0].wavelength_start_nm << ",,\n";

            *out <<a<< "End wavelength (nm):,"  << m_scanResults.cfg.section[m_scanResults.cfg.head.num_sections-1].wavelength_end_nm << ",,\n";

            line.sprintf("Pattern Pixel Width (nm):,%7.2f,,",m_scanResults.cfg.section[0].width_px  * PIXEL_WIDTH);
            *out << a<< line << ",,\n";

            switch (m_scanResults.cfg.section[0].exposure_time)
            {
            case 0:
                line.sprintf("Exposure (ms):,%7.3f,,", 0.635);
                break;
            case 1:
                line.sprintf("Exposure (ms):,%7.3f,,", 1.270);
                break;
            case 2:
                line.sprintf("Exposure (ms):,%7.3f,,", 2.450);
                break;
            case 3:
                line.sprintf("Exposure (ms):,%7.3f,,", 5.08);
                break;
            case 4:
                line.sprintf("Exposure (ms):,%7.3f,,", 15.24);
                break;
            case 5:
                line.sprintf("Exposure (ms):,%7.3f,,", 30.48);
                break;
            case 6:
                line.sprintf("Exposure (ms):,%7.3f,,", 60.96);
                break;
            default:
                line.sprintf("Exposure (ms):, unknown");
                break;
            }
            *out << a<< line << ",,\n";
        }

        if (m_scanResults.cfg.head.scan_type == 1)
        {

            *out << a<<"Scan Config Type:," << "Hadamard" << ",,\n";

            *out << a<<"Start wavelength (nm):,"  << m_scanResults.cfg.section[0].wavelength_start_nm << ",,\n";

            *out <<a<< "End wavelength (nm):,"  << m_scanResults.cfg.section[m_scanResults.cfg.head.num_sections-1].wavelength_end_nm << ",,\n";

            line.sprintf("Pattern Pixel Width (nm):,%7.2f,,",m_scanResults.cfg.section[0].width_px  * PIXEL_WIDTH);
            *out << a<< line << ",,\n";

            switch (m_scanResults.cfg.section[0].exposure_time)
            {
            case 0:
                line.sprintf("Exposure (ms):,%7.3f,,", 0.635);
                break;
            case 1:
                line.sprintf("Exposure (ms):,%7.3f,,", 1.270);
                break;
            case 2:
                line.sprintf("Exposure (ms):,%7.3f,,", 2.450);
                break;
            case 3:
                line.sprintf("Exposure (ms):,%7.3f,,", 5.08);
                break;
            case 4:
                line.sprintf("Exposure (ms):,%7.3f,,", 15.24);
                break;
            case 5:
                line.sprintf("Exposure (ms):,%7.3f,,", 30.48);
                break;
            case 6:
                line.sprintf("Exposure (ms):,%7.3f,,", 60.96);
                break;
            default:
                line.sprintf("Exposure (ms):, unknown");
                break;
            }
            *out << a<< line << ",,\n";
        }

        if (m_scanResults.cfg.head.scan_type == 2)
        {
            *out << a<<"Scan Config Type:," << "Slew" << ",,\n";

            for(int l=0; l < m_scanResults.cfg.head.num_sections; l++)
            {
                *out << a<<"Section "  << l+1 << ",,\n";

                *out << a<<"Start wavelength (nm):,"  << m_scanResults.cfg.section[l].wavelength_start_nm << ",,\n";

                *out <<a<< "End wavelength (nm):,"  << m_scanResults.cfg.section[m_scanResults.cfg.head.num_sections-1].wavelength_end_nm << ",,\n";

                line.sprintf("Pattern Pixel Width (nm):,%7.2f,,",m_scanResults.cfg.section[l].width_px  * PIXEL_WIDTH);
                *out << a<< line << ",,\n";

                switch (m_scanResults.cfg.section[l].exposure_time)
                {
                case 0:
                    line.sprintf("Exposure (ms):,%7.3f,,", 0.635);
                    break;
                case 1:
                    line.sprintf("Exposure (ms):,%7.3f,,", 1.270);
                    break;
                case 2:
                    line.sprintf("Exposure (ms):,%7.3f,,", 2.450);
                    break;
                case 3:
                    line.sprintf("Exposure (ms):,%7.3f,,", 5.08);
                    break;
                case 4:
                    line.sprintf("Exposure (ms):,%7.3f,,", 15.24);
                    break;
                case 5:
                    line.sprintf("Exposure (ms):,%7.3f,,", 30.48);
                    break;
                case 6:
                    line.sprintf("Exposure (ms):,%7.3f,,", 60.96);
                    break;
                default:
                    line.sprintf("Exposure (ms):, unknown");
                    break;
                }
                *out << a<< line << ",,\n";
            }
        }

        *out << a<<"Digital Resolution:,"  << num_patterns << ",,\n";

        *out << a<<"Num Repeats:,"  << m_scanResults.cfg.head.num_repeats << ",,\n";

        *out << a<<"PGA Gain:," << m_scanResults.pga << ",,\n";
    }

    line.sprintf("Total Measurement Time in sec:,%7.3f,,",lastScanTimeMS/1000.0);
    *out << a<< line << "\n";

}


//SavetoJCamp
void Spectrum::SavetoJCamp(QString scanName)
/**
 * This function saves the current scan data in a JCamp file
 * @param scanName - I - scanName to be used in the filename while saving
 *
 *
 */
{
    QString line;
    QVector<double> wavelengths = GetWavelengths();
    QVector<double> intensity = GetIntensities();
    QVector<double> absorbance = GetAbsorbance();
    QVector<double> reflectance = GetReflectance();

    QString scanName_a=scanName;
    QString scanName_i=scanName;
    QString scanName_r=scanName;
    scanName_a.append("_a");
    scanName_i.append("_i");
    scanName_r.append("_r");
    QString fileNameRe1 = QString("%1.jdx").arg(scanName_i);
    QString fileNameRe2 = QString("%1.jdx").arg(scanName_a);
    QString fileNameRe3 = QString("%1.jdx").arg(scanName_r);
    QString fileName_i = filepath.GetsaveScanSettingsDir().absoluteFilePath(fileNameRe1);
    QString fileName_a = filepath.GetsaveScanSettingsDir().absoluteFilePath(fileNameRe2);
    QString fileName_r = filepath.GetsaveScanSettingsDir().absoluteFilePath(fileNameRe3);

     if(save_JCAMP)
     {
            //save for absorbance
         if(absorbance_JCAMP)
         {
             if (QFile::exists(fileName_a))
             {
                 QFile::remove(fileName_a);
             }
             QFile adcReadingsFile_a(fileName_a);
             if(adcReadingsFile_a.open(QIODevice::WriteOnly | QIODevice::Text) != true)
             {
                 return;
             }
             QTextStream out_a(&adcReadingsFile_a);
            Saveheader(&out_a,true);

             line.sprintf("##XUNITS=wavelength(nm)");
              out_a << line << "\n";
             line.sprintf("##YUNITS=Absorbance");
             out_a << line << "\n";
             line.sprintf("##PEAK TABLE= X+(Y..Y)");
             out_a << line << "\n";
             for(int i=0; i < m_scanResults.length; i++)
             {
                 line.sprintf("%1f,%lf",wavelengths[i],absorbance[i]);
                  out_a << line << "\n";
             }

             adcReadingsFile_a.close();

         }

             //save for intensity
         if(intensity_JCAMP)
         {
             if (QFile::exists(fileName_i))
             {
                 QFile::remove(fileName_i);
             }
             QFile adcReadingsFile_i(fileName_i);
             if(adcReadingsFile_i.open(QIODevice::WriteOnly | QIODevice::Text) != true)
             {
                 return;
             }
             QTextStream out_i(&adcReadingsFile_i);
             Saveheader(&out_i,true);
             line.sprintf("##XUNITS=wavelength(nm)");
              out_i << line << "\n";
             line.sprintf("##YUNITS=intensity");
             out_i << line << "\n";
             line.sprintf("##PEAK TABLE= X+(Y..Y)");
             out_i << line << "\n";
             for(int i=0; i < m_scanResults.length; i++)
             {
                 line.sprintf("%1f,%lf",wavelengths[i],intensity[i]);
                  out_i << line << "\n";
             }

             adcReadingsFile_i.close();
         }

        if(reflectance_JCAMP)
        {
            if (QFile::exists(fileName_r))
            {
                QFile::remove(fileName_r);
            }
            QFile adcReadingsFile_r(fileName_r);
            if(adcReadingsFile_r.open(QIODevice::WriteOnly | QIODevice::Text) != true)
            {
                return;
            }
            QTextStream out_r(&adcReadingsFile_r);

            Saveheader(&out_r,true);
            line.sprintf("##XUNITS=wavelength(nm)");
             out_r << line << "\n";
            line.sprintf("##YUNITS=reflectance");
            out_r << line << "\n";
            line.sprintf("##PEAK TABLE= X+(Y..Y)");
            out_r << line << "\n";
            for(int i=0; i < m_scanResults.length; i++)
            {
                line.sprintf("%1f,%lf",wavelengths[i],reflectance[i]);
                 out_r << line << "\n";
            }

            adcReadingsFile_r.close();
         }
}



}

void Spectrum::SaveCombinedCSV(QString fileNameRe)
{
    QVector<double> wavelengths = GetWavelengths();
    QVector<double> intensity = GetIntensities();
    QVector<double> absorbance = GetAbsorbance();
    QVector<double> reference = GetRefIntensities();
    QVector<double> reflectance = GetReflectance();

    QString line;
    QString fileName = filepath.GetsaveScanSettingsDir().absoluteFilePath(fileNameRe);

    if (QFile::exists(fileName))
    {
        QFile::remove(fileName);
    }
    QFile adcReadingsFile(fileName);
    if(adcReadingsFile.open(QIODevice::WriteOnly | QIODevice::Text) != true)
    {
        return;
    }
    QTextStream out(&adcReadingsFile);
    Saveheader(&out,false);

    line.sprintf("Wavelength (nm),Absorbance (AU),Reference Signal (unitless),Sample Signal (unitless)");
    out << line << "\n";

    for(int i=0; i < m_scanResults.length; i++)
    {
        line.sprintf("%1f,%lf,%lf,%lf",wavelengths[i],absorbance[i],reference[i],intensity[i]);
        out << line << "\n";
    }

    adcReadingsFile.close();

}

void Spectrum::SaveToCSV(QString scanName)
/**
 * This function saves the current scan data in a csv file
 * @param scanName - I - scanName to be used in the file name while saving
 *
 */
{


    QVector<double> wavelengths = GetWavelengths();
    QVector<double> intensity = GetIntensities();
    QVector<double> absorbance = GetAbsorbance();
    QVector<double> reference = GetRefIntensities();
    QVector<double> reflectance = GetReflectance();

    QString line;
    QString scanName_a=scanName;
    QString scanName_i=scanName;
    QString scanName_r=scanName;
    scanName_a.append("_a");
    scanName_i.append("_i");
    scanName_r.append("_r");
    QString fileNameRel = QString("%1.csv").arg(scanName);
    QString fileNameRe2 = QString("%1.csv").arg(scanName_a);
    QString fileNameRe3 = QString("%1.csv").arg(scanName_i);
    QString fileNameRe4 = QString("%1.csv").arg(scanName_r);

    //QString fileName = filepath.GetsaveScanSettingsDir().absoluteFilePath(fileNameRel);
    QString fileName_a = filepath.GetsaveScanSettingsDir().absoluteFilePath(fileNameRe2);
    QString fileName_i = filepath.GetsaveScanSettingsDir().absoluteFilePath(fileNameRe3);
    QString fileName_r = filepath.GetsaveScanSettingsDir().absoluteFilePath(fileNameRe4);

    if(saveone_csv)
    {
        SaveCombinedCSV(fileNameRel);
    }
    if(saveseperate_csv)
    {
        // save intensity
        if(intensity_csv)
        {
            if (QFile::exists(fileName_i))
            {
                QFile::remove(fileName_i);
            }
            QFile adcReadingsFile_i(fileName_i);
            if(adcReadingsFile_i.open(QIODevice::WriteOnly | QIODevice::Text) != true)
            {
                return;
            }
            QTextStream out_i(&adcReadingsFile_i);

            line.sprintf("Wavelength (nm),Sample Signal (unitless)");
            out_i << line << "\n";
            for(int i=0; i < m_scanResults.length; i++)
            {
                line.sprintf("%1f,%lf",wavelengths[i],intensity[i]);
                 out_i << line << "\n";
            }

            adcReadingsFile_i.close();
        }


        //save absorbance
        if(absorbance_csv)
        {
            if (QFile::exists(fileName_a))
            {
                QFile::remove(fileName_a);
            }
            QFile adcReadingsFile_a(fileName_a);
            if(adcReadingsFile_a.open(QIODevice::WriteOnly | QIODevice::Text) != true)
            {
                return;
            }
            QTextStream out_a(&adcReadingsFile_a);

            line.sprintf("Wavelength (nm),Absorbance (AU)");
            out_a << line << "\n";
            for(int i=0; i < m_scanResults.length; i++)
            {
                line.sprintf("%1f,%lf",wavelengths[i],absorbance[i]);
                 out_a << line << "\n";
            }

            adcReadingsFile_a.close();
        }
        //save reflectance
        if(reflectance_csv)
        {
            if (QFile::exists(fileName_r))
            {
                QFile::remove(fileName_r);
            }
            QFile adcReadingsFile_r(fileName_r);
            if(adcReadingsFile_r.open(QIODevice::WriteOnly | QIODevice::Text) != true)
            {
                return;
            }
            QTextStream out_r(&adcReadingsFile_r);

            line.sprintf("Wavelength (nm),Reflectance (AU)");
            out_r << line << "\n";
            for(int i=0; i < m_scanResults.length; i++)
            {
                line.sprintf("%1f,%lf",wavelengths[i],reflectance[i]);
                 out_r << line << "\n";
            }

            adcReadingsFile_r.close();
        }
    }
}
slewScanConfig Spectrum::GetScanConfig(void)
/**
 * Retuns the scanConfig structure embedded in the scan data last set using SetData().
 */
{
    return m_scanResults.cfg;
}

