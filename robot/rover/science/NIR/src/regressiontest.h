/*****************************************************************************
 *
 * This module provides function handlers and supporting functions for regression tests.
 *
 * Copyright (c) 2015 Texas Instruments Incorporated.
 * ALL RIGHTS RESERVED
 *
 ******************************************************************************/
#ifndef REGRESSIONTEST_H
#define REGRESSIONTEST_H

class regressiontest
{

public:
    regressiontest();
    ~regressiontest();

    bool scan_config_good;
    int ADC_Max;


    /* Test Function Declaration */
    void Regtest_NNODLPCEnable(void);
    void Regtest_ApplyScanConfig(void);
    void Regtest_PerformScan(void);
    void Regtest_DLPCLampControl(void);
    void Regtest_ScanConfiguration(void);
    int  Regtest_read_scan_cfg_index_to_device(void);
    void Regtest_sensortest(void);
    void Regtest_DateTime(void);
    void Regtest_PGA(void);
    void Regtest_SerialNumber(void);
    void Regtest_GeneratePatterns(void);
    void Regtest_SaveRefCal(void);
    void Regtest_SNRTest(QString);
    void Regtest_NNODLPCReg(void);
    void Regtest_NNOSDCard(void);
    void Regtest_ScanSubImage(void);
    void Regtest_ScanConfigCosineSimilarity(void);


    /* Utility Function Declaration */
     int      RegtestUtil_apply_scan_cfg_to_device(uScanConfig *pCfg);
     uint32_t RegtestUtil_find_median(int *val_array, uint32_t num_vals);
     int      RegtestUtil_apply_scan_cfg_index_to_device(uScanConfig *pCfg , int index);
     int      RegtestUtil_find_max(int* val_array, uint32 num_vals);
     void     RegtestUtil_generatehtmlreport(void);
     void     RegtestUtil_validateMax(int adc_max);
     void     RegtestUtil_PerformScan(void);
     void     RegtestUtil_compare_spectrum(int *testAbsorbance, int *refAbsorbance, int vectorLength, double simThreshold, bool *isSimilar, double *simNumber);
     int      RegtestUtil_cosineSimilarity(int* sample);
     void     RegtestUtil_ValidateScanResults(scanResults* scan_result);
     int      Regtest_UtilVerifySimilarityOne(int* max);
     int      Regtest_UtilVerifySimilarityTwo(int* max);

};


#endif // REGRESSIONTEST_H
