#include "Spectrometer.h"
#include "refCalMatrix.h"

#include <iostream>
using namespace std;

bool Spectrometer::init()
{


    if (USB_Init() != 0)
    {
        cout << "Cannot init" << endl;
        return false;
    }

    if (USB_Open() != 0)
    {
        cout << "Cannot open" << endl;
        return false;
    }

    /*if(createScanConfiguration() != PASS){
        cout << "Error creating configuration" << endl;
    } */

    m_ref_selection = SCAN_REF_FACTORY;

//    printConfigurations();

    NNO_SetHibernate(false);

    FetchRefCalData();
    FetchRefCalMatrix();
    return true;
}
int Spectrometer::createScanConfiguration(){
    activeConfiguration.scanCfg.wavelength_start_nm = 420;
    activeConfiguration.scanCfg.wavelength_end_nm = 420;

    uScanConfig cfg;
    size_t bufferSize;
    void *pBuffer;
    memcpy(&cfg, &activeConfiguration, sizeof(cfg));


    if( dlpspec_get_scan_config_dump_size(&cfg, &bufferSize) != DLPSPEC_PASS) {
        cout << "Error getting size of configuration" << endl;
        return FAIL;
    }
    pBuffer =  malloc(bufferSize);
    if(pBuffer == NULL)
    {
        cout << "pBuffer null" << endl;
        return FAIL;
    }
    if(dlpspec_scan_write_configuration(&cfg, pBuffer, bufferSize) != DLPSPEC_PASS)
    {
        free(pBuffer);
    }
    if(NNO_SaveScanCfgInEVM(0, pBuffer, bufferSize) != PASS)
    {
        free(pBuffer);
    }
    if( getAllConfigurations() != PASS){
        cout << "Error getting configurations" << endl;
        return FAIL;
    }
    return PASS;
}
void Spectrometer::deinit()
{

    USB_Close();
    USB_Exit();
}

int Spectrometer::scan(Spectrum &spectrum)
{
    uint8_t pDataBlob[SCAN_DATA_BLOB_SIZE];
    uScanData *pData = (uScanData *)pDataBlob;
    int fileSize;
    if(runScan(pDataBlob,&fileSize) != PASS){
        cout << "Error running scan" << endl;
        return FAIL;
    }else{
        cout << "Succesfully ran scan" << endl;
    }
    if(m_ref_selection == SCAN_REF_FACTORY) {
        cout << "Using factory scan reference" << endl;
        if (SetData(pData, GetRefCalDataBlob()) != PASS) {
            cout << "scan or reference data interpret failed" << endl;
            free(pData);
            return FAIL;
        }
        cout << "Scan data saved in EVM" << endl;

        vector<double> wavelengths = GetWavelengths();
        vector<double> intensity = GetIntensities();
        vector<double> absorbance = GetAbsorbance();
        vector<double> reference = GetRefIntensities();
        vector<double> reflectance = GetReflectance();

        std::vector<std::pair<std::string,std::vector<double>>> dataset = {{"Wavelengths",wavelengths},{"Absorbances",absorbance}};
        SaveToCSV("absorbance_scan.csv", dataset);
    }

    return PASS;
}
void Spectrometer::SaveToCSV(string filename,std::vector<std::pair<std::string, std::vector<double>>> dataset)
{
    std::ofstream myFile(filename,std::ofstream::out);

    for(int j = 0; j < dataset.size(); ++j)
    {
        myFile << dataset.at(j).first;
        if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
    }
    myFile << "\n";

    // Send data to the stream
    for(int i = 0; i < dataset.at(0).second.size(); ++i)
    {
        for(int j = 0; j < dataset.size(); ++j)
        {
            myFile << dataset.at(j).second.at(i);
            if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
        }
        myFile << "\n";
    }

    cout << "Created the csv ! " << endl;
    myFile.close();
}

vector<double> Spectrometer::GetWavelengths()
/**
 * This function is for accessing the wavelength vector from data last
 * set using SetData()
 *
 */
{

    vector<double> out;
    for (int i = 0; i < m_scanResults.length; ++i)
    {
        out.push_back(m_scanResults.wavelength[i]);
    }
    return out;
}

vector<double> Spectrometer::GetIntensities()
/**
 * This function is for accessing the intensity vector from data last
 * set using SetData()
 */
{
    vector<double> out;
    for (int i = 0; i < m_scanResults.length; ++i)
    {
        out.push_back(m_scanResults.intensity[i]);
    }
    return out;
}

vector<double> Spectrometer::GetRefIntensities()
/**
 * This function is for accessing the reference intensity vector from data last
 * set using SetData()
 */
{
    vector<double> out;
    for (int i = 0; i < m_referenceResults.length; ++i)
    {
        out.push_back(m_referenceResults.intensity[i]);
    }
    return out;
}

vector<double> Spectrometer::GetAbsorbance()
/**
 * This function is for accessing the absorbance vector from data last
 * set using SetData()
 */
{
    vector<double> out;
    double absorbance;

    m_maxAbsorbance = 0;
    for(int i = 0; i < m_scanResults.length; i++ )
    {
        if(m_referenceResults.length > i)
        {
            absorbance = COMPUTE_ABSORPTION_VAL(m_scanResults.intensity[i], m_referenceResults.intensity[i]);
            out.push_back(absorbance);
            if(absorbance > m_maxAbsorbance)
                m_maxAbsorbance = absorbance;
        }
    }
    return out;
}

//get reflectance
vector<double> Spectrometer::GetReflectance()
/**
 * This function is for accessing the absorbance vector from data last
 * set using SetData()
 */
{
    vector<double> out;
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

            out.push_back(reflectance);
            //if(reflectance > m_maxReflectance)
            //  m_maxReflectance = reflectance;
        }
    }
    return out;
}

int Spectrometer::fetchScan(unsigned char *buffer)
{
    size_t fileSize = NNO_GetFileSizeToRead(NNO_FILE_SCAN_DATA);
    return NNO_GetFile(buffer, fileSize);
}

int Spectrometer::getAllConfigurations(void)
{
    int num_records = NNO_GetNumScanCfg();
    uint8 i;
    void *buf;
    uScanConfig *pCfg;
    uint32 bufSize;
    int ret_val;
    cout << "Num records : " << num_records << endl;
    if(num_records <= 0)
    {
        cout << "No configurations saved on the EVM" << endl;
        return FAIL;
    }

    buf = (void *)malloc(NNO_DATA_MAX_SIZE);
    if(buf == NULL)
    {
        return FAIL;
    }
    pCfg = (uScanConfig *)buf;
    cfg_list.clear();

    for(i=0; i<num_records; i++)
    {
        ret_val = NNO_GetScanCfg(i, buf, &bufSize);
        if((ret_val == PASS) && (bufSize != 0))
        {
            dlpspec_scan_read_configuration(buf, bufSize);
            cfg_list.push_back(*pCfg);
        }
        else
        {
            break;
        }
    }
    cout << "Number of configurations : " << cfg_list.size() << endl;
    free(buf);
    m_activeindex = NNO_GetActiveScanIndex();

    return ret_val;
}
void Spectrometer::printConfigurations() {
    for(int i = 0 ; i < cfg_list.size() ; i++){

        cout << "Scan config : " << cfg_list[i].scanCfg.config_name << endl;
        cout << "Scan type : " << (int)cfg_list[i].scanCfg.scan_type << endl;
        cout << "Start wavelength : " << (int) cfg_list[i].scanCfg.wavelength_start_nm << endl;
        cout << "End wavelength : " << (int) cfg_list[i].scanCfg.wavelength_end_nm << endl;
        cout << "Width : " << (int) cfg_list[i].scanCfg.width_px<< endl;
    }
}
int Spectrometer::runScan(void *pData, int *pBytesRead)
{
    int size;
    unsigned int devStatus;

    NNO_SetScanNumRepeats(6);

    int scanTime = NNO_GetEstimatedScanTime();
    NNO_PerformScan(false);
    if(NNO_ReadDeviceStatus(&devStatus) == PASS)
    {
        do
        {
            if((devStatus & NNO_STATUS_SCAN_IN_PROGRESS) != NNO_STATUS_SCAN_IN_PROGRESS)
                break;
         }while(NNO_ReadDeviceStatus(&devStatus) == PASS);
    }
    else
    {
        cout << "Reading device status for scan completion failed" << endl;
        return FAIL;
    }
    *pBytesRead = NNO_GetFileSizeToRead(NNO_FILE_SCAN_DATA);

    if((size = NNO_GetFile((unsigned char *)pData, *pBytesRead)) != *pBytesRead)
    {
        cout << "Size issue" << endl;
        *pBytesRead = size;
       return FAIL;
    }
    return PASS;
}

void *Spectrometer::GetRefCalMatrixBlob()
{
    refCalMatrix tmpRefCalMatrix;

    memcpy(tmpRefCalMatrix.width, refCalMatrix_widths, sizeof(uint8_t) * REF_CAL_INTERP_WIDTH);
    memcpy(tmpRefCalMatrix.wavelength, refCalMatrix_wavelengths, sizeof(double) * REF_CAL_INTERP_WAVELENGTH);
    memcpy(tmpRefCalMatrix.ref_lookup, refCalMatrix_intensities, sizeof(uint16_t) * REF_CAL_INTERP_WIDTH * REF_CAL_INTERP_WAVELENGTH);

    dlpspec_calib_write_ref_matrix(&tmpRefCalMatrix, refCalMatrixBlob, REF_CAL_MATRIX_BLOB_SIZE);

    return refCalMatrixBlob;
}

void *Spectrometer::GetRefCalDataBlob()
{
    return refCalDataBlob;
}

int Spectrometer::FetchRefCalData()
{
    int refCalSize = NNO_GetFileSizeToRead(NNO_FILE_REF_CAL_DATA);

    if (NNO_GetFile((unsigned char *)refCalDataBlob, refCalSize) == refCalSize)
        return PASS;
    else
        return FAIL;
}

int Spectrometer::FetchRefCalMatrix()
{
    int refCalSize = NNO_GetFileSizeToRead(NNO_FILE_REF_CAL_MATRIX);

    if (NNO_GetFile((unsigned char *)refCalMatrixBlob, refCalSize) == refCalSize)
        return PASS;
    else
        return FAIL;
}
int Spectrometer::SetData(void* pScanDataBlob, void *pRefDataBlob)
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
        if(dlpspec_scan_interpret(pScanDataBlob, SCAN_DATA_BLOB_SIZE, &m_scanResults)!= PASS) {
            cout << "Failed scan interpret " << endl;
            return FAIL;
        }
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
                                                GetRefCalMatrixBlob(),
                                                REF_CAL_MATRIX_BLOB_SIZE, &m_scanResults, &m_referenceResults))
        {
            free(pCopyBuff);
            return FAIL;
        }
        free(pCopyBuff);
    }
    return PASS;
}
