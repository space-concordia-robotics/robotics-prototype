#ifndef _SPECTROMETER_H_
#define _SPECTROMETER_H_

#include <unistd.h>
#include <string>
#include <vector>
#include "API.h"
#include "usb.h"
#include <dlpspec_scan_col.h>
#include <cmath>
#include "fstream"
#define COMPUTE_ABSORPTION_VAL(S , R)	((double)-1.0 * (double)std::log10( (double) ((double) (S) / (double)(R) ) ) )
#define COMPUTE_REFLECTANCE_VAL(S , R)	((double) (S) / (double)(R))

struct Spectrum{

};

class Spectrometer
{
public:
  bool init();
  void deinit();

  int scan(Spectrum &spectrum);

private:
  int runScan(void *pData, int *pBytesRead);
  int fetchActiveConfig();
  int fetchScan(unsigned char *buffer);
  int createScanConfiguration();
    std::vector<double> GetWavelengths();
    std::vector<double> GetIntensities();
    std::vector<double> GetAbsorbance();
    std::vector<double> GetReflectance();
    std::vector<double> GetRefIntensities();

    void *GetRefCalMatrixBlob();
  void *GetRefCalDataBlob();
  int FetchRefCalData();
  int FetchRefCalMatrix();
  void printConfigurations();
  int getAllConfigurations(void);
  uint8_t refCalMatrixBlob[REF_CAL_MATRIX_BLOB_SIZE];
  uint8_t refCalDataBlob[SCAN_DATA_BLOB_SIZE];
    int m_activeindex;
  uint8_t activeConfigurationBuffer[SCAN_DATA_BLOB_SIZE];
  uScanConfig activeConfiguration;
  std::vector<uScanConfig> cfg_list;
    typedef enum
    {
        SCAN_REF_FACTORY,
        SCAN_REF_PREV,
        SCAN_REF_NEW,
        SCAN_REF_MAX
    }SCAN_REF_TYPE;
    SCAN_REF_TYPE m_ref_selection;
    std::string m_fileName;
    double m_maxAbsorbance;
    uint8  m_curScanDataBlob[SCAN_DATA_BLOB_SIZE];
    uint8  m_curReferenceDataBlob[SCAN_DATA_BLOB_SIZE];
    scanResults m_scanResults;
    scanResults m_referenceResults;
    int SetData(void* pScanDataBlob, void *pRefDataBlob);
    void SaveToCSV(std::string scanName,std::vector<std::pair<std::string, std::vector<double>>> dataset);
};

#endif // _SPECTROMETER_H_
