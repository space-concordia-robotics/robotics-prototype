/*****************************************************************************
 *
 * This module provides function handlers and supporting functions for regression tests.
 *
 * Copyright (c) 2015 Texas Instruments Incorporated.
 * ALL RIGHTS RESERVED
 *
 ******************************************************************************/

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "dlpspec_scan.h"
#include "regressiontest.h"
#include <string.h>
#include <QTimer>
#include <QMessageBox>


#define NNO_CORRECT_FILE_SIZE 3822

int knownSample[228]={
    629123,
    627697,
    627217,
    626694,
    626644,
    626348,
    626312,
    625584,
    624787,
    624676,
    624561,
    623074,
    621082,
    620645,
    617720,
    617131,
    616731,
    616152,
    613884,
    613518,
    609255,
    608892,
    608006,
    606709,
    605543,
    605515,
    602014,
    601514,
    598785,
    597864,
    596474,
    593435,
    593042,
    590986,
    586568,
    585202,
    582179,
    581878,
    578210,
    575195,
    574421,
    569696,
    568854,
    567063,
    565788,
    562181,
    561406,
    560497,
    559338,
    559334,
    559287,
    556062,
    555248,
    554340,
    552952,
    550030,
    548263,
    547988,
    545838,
    542972,
    540166,
    539661,
    534265,
    533589,
    531107,
    526801,
    524526,
    521650,
    517840,
    515555,
    511600,
    506902,
    505961,
    501683,
    497474,
    495826,
    487884,
    486561,
    482325,
    476059,
    474554,
    469342,
    468158,
    460632,
    460153,
    452466,
    449923,
    446683,
    443788,
    439945,
    435549,
    432081,
    427814,
    420694,
    419261,
    413906,
    413876,
    406732,
    404875,
    402944,
    396865,
    396331,
    394536,
    394462,
    394064,
    393972,
    393689,
    393086,
    392848,
    392726,
    391959,
    391687,
    390586,
    390154,
    389559,
    389357,
    388387,
    387498,
    387046,
    386961,
    386488,
    386050,
    386004,
    384678,
    384538,
    384203,
    382908,
    382830,
    381929,
    381261,
    381251,
    381011,
    380803,
    380794,
    379760,
    379569,
    379313,
    378874,
    377232,
    374939,
    374339,
    372347,
    371846,
    369348,
    369186,
    367656,
    366286,
    364244,
    361721,
    360771,
    358142,
    356481,
    349823,
    349590,
    343715,
    341296,
    336611,
    332647,
    329120,
    324084,
    314437,
    312404,
    305541,
    301820,
    297271,
    288655,
    288564,
    279306,
    274313,
    269315,
    261549,
    255818,
    254300,
    245754,
    239940,
    233921,
    227465,
    223013,
    219563,
    211374,
    208572,
    200828,
    194187,
    192948,
    186037,
    178119,
    171834,
    168483,
    162225,
    154599,
    152643,
    146346,
    140045,
    138817,
    131418,
    125431,
    123668,
    118739,
    112263,
    108183,
    104225,
    98709,
    98354,
    91894,
    89535,
    85776,
    83132,
    76726,
    76226,
    70109,
    67614,
    63147,
    54941,
    43680,
    36846,
    30554,
    23924,
    18012,
    14860,
    12302,
    9591,
    7906,
    6411,
    3350,
    3036,
    2823,
    2728,
    2333,

};

regressiontest::regressiontest()
{

}

regressiontest::~regressiontest()
{

}



void MainWindow::on_pushButton_regtest_clicked()
{


    QString labeltext;
    QString path = QDir::currentPath();
    QString msg = QString("Test report will be generated at %1/ regression_test_report.txt").arg(path);

    QMessageBox reply;
    reply.setText(msg);
    QAbstractButton *myYesButton = reply.addButton(trUtf8("Proceed"), QMessageBox::YesRole);
    QAbstractButton *myNoButton = reply.addButton(trUtf8("Cancel"), QMessageBox::NoRole);
    reply.setIcon(QMessageBox::Question);
    reply.exec();


    if (reply.clickedButton() == myNoButton)
    {
       return;
    }

    else if(reply.clickedButton() == myYesButton)
     {

    //to indicate that the test is started
    ui->progressBar_RegTest->setValue(5);
    QApplication::processEvents();
    regtest.scan_config_good = false;
    ui->label_RegTest->setText("Start Test");

    regtest.Regtest_SerialNumber();
    regtest.Regtest_NNODLPCEnable();

    regtest.Regtest_ScanConfigCosineSimilarity();
    regtest.Regtest_ApplyScanConfig();

    ui->label_RegTest->setText("Scan Tests");
    ui->progressBar_RegTest->setValue(10);
    regtest.Regtest_PerformScan();
    regtest.Regtest_ScanSubImage();
    regtest.Regtest_DLPCLampControl();
    regtest.Regtest_ScanConfiguration();
    regtest.Regtest_read_scan_cfg_index_to_device();

    ui->label_RegTest->setText("Sensor Tests");
    ui->progressBar_RegTest->setValue(20);
    regtest.Regtest_sensortest();
    regtest.Regtest_DateTime();
    regtest.Regtest_PGA();
    regtest.Regtest_GeneratePatterns();


    ui->label_RegTest->setText("Reference calibration Tests");
    ui->progressBar_RegTest->setValue(30);
    /* Reference Calibration Test */
    {
        void *pData = malloc(SCAN_DATA_BLOB_SIZE);
        if(pData == NULL)
        {
            showError("Out of Memory");
            return;
        }
        PerformRefCalScan(pData);
        NNO_SaveRefCalPerformed();
        free(pData);
    }

    regtest.Regtest_SaveRefCal();

    /* SNR Tests */
    ui->label_RegTest->setText("SNR Tests");
    ui->progressBar_RegTest->setValue(40);
    DoSNRComputation( 0 );
    labeltext = ui->label_cal->text();
    regtest.Regtest_SNRTest(labeltext);

    regtest.Regtest_NNODLPCReg();
    regtest.Regtest_NNOSDCard();

    ui->progressBar_RegTest->setValue(90);
    ui->label_RegTest->setText("Generate Report");
    regtest.RegtestUtil_generatehtmlreport();
    ui->progressBar_RegTest->setValue(100);

    msg = QString("Test report generated at %1/ regression_test_report.txt").arg(path);
    showError(msg.toLatin1().data());

    ui->progressBar_RegTest->setValue(0);
    ui->label_RegTest->setText(" ");
  }
}

void regressiontest::Regtest_NNODLPCEnable(void)
{
    FILE* reg_test;
    uint32 value;

    /* Test for DLPC150 Enable */
    reg_test = fopen("regression_test_report.txt" , "w");
    if(reg_test == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open regression_test_report.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }

    NNO_DLPCEnable(true, false);
    NNO_GetDLPCReg(0x40015000, &value);

    if(reg_test != NULL)
    {
        if(1 == value)
        {
            fprintf(reg_test , "NNO_DLPCEnable-PASS\n");
        }
        else
        {
            fprintf(reg_test , "NNO_DLPCEnable-FAIL-Get values Does not match 1\n");
        }
        fclose(reg_test);
    }
}

void regressiontest::Regtest_ScanConfigCosineSimilarity(void)
{
    FILE* system_validation;
    calibCoeffs calData;
    int retval;
    int max;

    system_validation = fopen("system_validation_data.txt" , "w");
    if(system_validation == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open system_validation_data.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }
    fprintf(system_validation,"=============================Cosine Similarity For baked in scan Configurations==============================\n");
    /*Scan Config 1*/
    if (0 > NNO_GetCalibStruct(&calData))
    {
        fprintf(system_validation, "Calib Coefficients not valid\n");
        fclose(system_validation);
        return;
    }

    uScanConfig config;
    config.scanCfg.scan_type = COLUMN_TYPE;
    config.scanCfg.num_patterns = 100; //length is 100
    config.scanCfg.num_repeats = 10;
    config.scanCfg.wavelength_start_nm = MIN_WAVELENGTH;
    config.scanCfg.wavelength_end_nm = MAX_WAVELENGTH;
    config.scanCfg.width_px = 60;
    retval = RegtestUtil_apply_scan_cfg_to_device(&config);

    if(retval < 0)
    {
        fprintf(system_validation, "Apply config 1 failed\n");
        fclose(system_validation);
        return;
    }

    RegtestUtil_PerformScan();
    retval = Regtest_UtilVerifySimilarityOne(&max);

    if(-1 == retval)
    {
        fprintf(system_validation, "config.scan_type = COLUMN_TYPE\n");
        fprintf(system_validation, "config.num_patterns = 100\n");
        fprintf(system_validation, "config.num_repeats = 10\n");
        fprintf(system_validation, "config.wavelength_start_nm = 900\n");
        fprintf(system_validation, "config.wavelength_end_nm = 1700\n");
        fprintf(system_validation, "config.width_px = 60\n");
        fprintf(system_validation,"Not Cosine similar\n");
    }
    else
    {
        fprintf(system_validation, "config.scan_type = COLUMN_TYPE\n");
        fprintf(system_validation, "config.num_patterns = 100\n");
        fprintf(system_validation, "config.num_repeats = 10\n");
        fprintf(system_validation, "config.wavelength_start_nm = 900\n");
        fprintf(system_validation, "config.wavelength_end_nm = 1700\n");
        fprintf(system_validation, "config.width_px = 60\n");
        fprintf(system_validation,"Cosine similar\n");
    }
    fclose(system_validation);
    RegtestUtil_validateMax(max);

    system_validation = fopen("system_validation_data.txt" , "a");
    if(system_validation == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open system_validation_data.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }
    config.scanCfg.scan_type = COLUMN_TYPE;
    config.scanCfg.num_patterns = 624;
    config.scanCfg.num_repeats = 10;
    config.scanCfg.wavelength_start_nm = MIN_WAVELENGTH;
    config.scanCfg.wavelength_end_nm = MAX_WAVELENGTH;
    config.scanCfg.width_px = 2;

    retval = RegtestUtil_apply_scan_cfg_to_device(&config);

    if(retval < 0)
    {
        fprintf(system_validation, "Apply config 2 failed\n");
        fclose(system_validation);
        return;
    }

    RegtestUtil_PerformScan();
    retval = Regtest_UtilVerifySimilarityTwo(&max);

    if(-1 == retval)
    {
        fprintf(system_validation, "config.scan_type = COLUMN_TYPE\n");
        fprintf(system_validation, "config.num_patterns = 624\n");
        fprintf(system_validation, "config.num_repeats = 10\n");
        fprintf(system_validation, "config.wavelength_start_nm = 900\n");
        fprintf(system_validation, "config.wavelength_end_nm = 1700\n");
        fprintf(system_validation, "config.width_px = 2\n");
        fprintf(system_validation,"Not Cosine similar\n");

    }
    else
    {
        fprintf(system_validation, "config.scan_type = COLUMN_TYPE\n");
        fprintf(system_validation, "config.num_patterns = 624\n");
        fprintf(system_validation, "config.num_repeats = 10\n");
        fprintf(system_validation, "config.wavelength_start_nm = 900\n");
        fprintf(system_validation, "config.wavelength_end_nm = 1700\n");
        fprintf(system_validation, "config.width_px = 2\n");
        fprintf(system_validation,"Cosine similar\n");

    }
    fclose(system_validation);
    RegtestUtil_validateMax(max);

}

void regressiontest::Regtest_ApplyScanConfig(void)
{
    FILE* reg_test;
    FILE* system_validation;
    calibCoeffs calData;
    double minWavelength=0, maxWavelength=0;
    int retval;

    /* Test the scan config API */

    reg_test = fopen("regression_test_report.txt" , "a");

    if(reg_test == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open regression_test_report.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }

    system_validation= fopen("system_validation_data.txt" , "a");
    if(system_validation == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open system_validation_data.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }
    if (0 > NNO_GetCalibStruct(&calData))
    {
        fprintf(reg_test , "NNO_GetCalibStruct-FAIL-API Fails with negative return\n");
    }
    else
    {

        dlpspec_util_columnToNm(0, calData.PixelToWavelengthCoeffs, &maxWavelength);
        dlpspec_util_columnToNm(853, calData.PixelToWavelengthCoeffs, &minWavelength);

        fprintf(system_validation,"=============================Calibration Coefficient validation==============================\n");

        fprintf(system_validation , "wavelength corresponding to column 0 is Maxwavelength -%f\n", maxWavelength);
        fprintf(system_validation , "wavelength corresponding to column 853 is Minwavelength-%f\n", minWavelength);
        if ((maxWavelength < MAX_WAVELENGTH) || (minWavelength > MIN_WAVELENGTH))
        {
            fprintf(reg_test , "NNO_GetCalibStruct-FAIL-Min and Max wavelength beyond range[900,1700]\n");
        }
        else
        {
            fprintf(reg_test , "NNO_GetCalibStruct-PASS\n");
        }

    }


    /* Dump the coefficients */
    fprintf(system_validation , "ShiftCoeff 0 [%f] , ShiftCoeff 1 [%f] , ShiftCoeff 2 [%f]\
    , PixWaveCoeff 0 [%f] ,PixWaveCoeff 1 [%f] , PixWaveCoeff 2 [%f] \n",\
    calData.ShiftVectorCoeffs[0] ,calData.ShiftVectorCoeffs[1] ,calData.ShiftVectorCoeffs[2],\
    calData.PixelToWavelengthCoeffs[0] ,calData.PixelToWavelengthCoeffs[1] ,calData.PixelToWavelengthCoeffs[2]);

    fclose(system_validation);

    uScanConfig config;
    config.scanCfg.scan_type = COLUMN_TYPE;
    config.scanCfg.num_patterns = 228;
    config.scanCfg.num_repeats = 30;
    config.scanCfg.wavelength_start_nm = MIN_WAVELENGTH;
    config.scanCfg.wavelength_end_nm = MAX_WAVELENGTH;
    config.scanCfg.width_px = 6;
    retval = RegtestUtil_apply_scan_cfg_to_device(&config);
    if( config.scanCfg.num_patterns == retval)/* REVIEW ALL THE VALUES CHECK */
    {
        scan_config_good = true;
        fprintf(reg_test , "NNO_ApplyScanConfig-PASS\n");
    }
    else if(-1 == retval)
        fprintf(reg_test , "Malloc fail in ApplyScanConfig\n");
    else
        fprintf(reg_test , "NNO_ApplyScanConfig-FAIL-Number of Patterns %d does not match 228\n", retval);

    fclose(reg_test);
}

void regressiontest::Regtest_PerformScan(void)
{
    FILE* reg_test;
    FILE* system_validation;
    uint32 size;
    uint32 median;
    size_t fileSize;
    scanData *pData;
    scanResults scan_results;
    uint32 red , green , blue;
    int max;


    reg_test = fopen("regression_test_report.txt" , "a");
    if(reg_test == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open regression_test_report.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }
    system_validation= fopen("system_validation_data.txt" , "a");
    if(system_validation == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open system_validation_data.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }
    if(false == scan_config_good)
    {
        fprintf(reg_test , "No Scan Allowed as scan config failed\n");
    }
    else
    {   NNO_SetScanNumRepeats(10);

        RegtestUtil_PerformScan();

        fileSize = NNO_GetFileSizeToRead(NNO_FILE_SCAN_DATA);

        if(NNO_CORRECT_FILE_SIZE == fileSize)
            fprintf(reg_test , "NNO_GetFileSizeToRead-PASS \n");
        else
            fprintf(reg_test , "NNO_GetFileSizeToRead-FAIL-fileSize %d does not match 3822\n", fileSize);
        pData = (scanData *)malloc(fileSize);

        if(pData == NULL)
        {
            fprintf(reg_test , "Malloc Fails in Perform scan\n");
            fclose(reg_test);
            fclose(system_validation);
            return;
        }
        else
        {
            if((size = NNO_GetFile((unsigned char *)pData, fileSize)) == fileSize)
            {
                dlpspec_scan_interpret(pData, fileSize, &scan_results);
            }
            else
            {
                fprintf(reg_test , "NNO_GetFile-FAIL-size %d does not match fileSize %d\n", size , fileSize);
                fclose(reg_test);
                fclose(system_validation);
                free(pData);
                return;
            }
            free(pData);
            median = RegtestUtil_find_median((int*)&scan_results.intensity[0], scan_results.length);
        }
        if(10 == scan_results.cfg.head.num_repeats)
        {
            fprintf(reg_test , "NNO_SetScanNumRepeats-PASS\n");
        }
        else
        {
            fprintf(reg_test , "NNO_SetScanNumRepeats-FAIL-num repeats not equal to set value 10\n");
        }/* REVIEW SCAN RESULTS EVERYTHING */

        if(median > 10000)
         {
            fprintf(reg_test , "NNO_PerformScan-PASS\n");
            fprintf(reg_test , "NNO_GetFile-PASS\n");
            fprintf(reg_test , "NNO_GetScanComplete-PASS\n");
         }
        else
         {
            fprintf(reg_test , "NNO_PerformScan-FAIL-Signal strength is very low\n");
            fprintf(reg_test , "NNO_GetFile-PASS\n");
            fprintf(reg_test , "NNO_GetScanComplete-PASS\n");
         }
        NNO_GetPhotoDetector(&red, &green , &blue);

        if(green > 3000)
            fprintf(reg_test , " NNO_GetPhotoDetector-PASS\n");
        else
            fprintf(reg_test , " NNO_GetPhotoDetector-FAIL-PD value less than 3000\n" );

        max = RegtestUtil_find_max((int*)&scan_results.intensity[0], scan_results.length);

        fprintf(system_validation,"=============================MAX ADC validation for Reference Cal configuration==============================\n");
        fclose(system_validation);
        RegtestUtil_validateMax(max);
        RegtestUtil_ValidateScanResults(&scan_results);

        if(-1 == RegtestUtil_cosineSimilarity((int*)&scan_results.intensity[0]))
        {
            fprintf(reg_test , "NNO_PerformScan-FAIL-Cosine similarity fails\n");
        }
        else
        {
            fprintf(reg_test , "NNO_PerformScan-PASS-Cosine similarity Passes\n");
        }
        ADC_Max = max;

    }
    fclose(reg_test);
    fclose(system_validation);
}

void regressiontest::Regtest_ScanSubImage(void)
{
    FILE* reg_test;
    uint32 size;
    uint32 median;
    size_t fileSize;
    scanData *pData;
    scanResults scan_results;
    FILE* system_validation;
    int max;

    system_validation= fopen("system_validation_data.txt" , "a");
    if(system_validation == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open system_validation_data.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }
    reg_test = fopen("regression_test_report.txt" , "a");
    if(reg_test == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open regression_test_report.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }

    /* One more scan for different API */
    NNO_setScanSubImage(0, 240);
    RegtestUtil_PerformScan();

    fileSize = NNO_GetFileSizeToRead(NNO_FILE_SCAN_DATA);

    pData = (scanData *)malloc(fileSize);

    if(pData == NULL)
    {
        fprintf(reg_test , "Malloc Fails in Perform scan\n");
        fclose(reg_test);
        fclose(system_validation);
        return;
    }
    else
    {
        if((size = NNO_GetFile((unsigned char *)pData, fileSize)) == fileSize)
        {
            dlpspec_scan_interpret(pData, fileSize, &scan_results);
        }
        else
        {
            fprintf(reg_test , "NNO_GetFile-FAIL-size %d does not match fileSize %d\n", size , fileSize);
            fclose(reg_test);
            fclose(system_validation);
            free(pData);
            return;
        }

        median = RegtestUtil_find_median((int*)&scan_results.intensity[0], scan_results.length);
    }

    max = RegtestUtil_find_max((int*)&scan_results.intensity[0], scan_results.length);

    float divisor = max / (float)ADC_Max;

    fprintf(system_validation,"=============================SubImage effect on ADC validation==============================\n");

    fprintf(system_validation,"Ratio of ADC MAX at half image to ADC Max at Full Image %f \n", divisor);

    if(divisor > 0.7 )
    {
        fprintf(system_validation,"Max ADC value for half image is not half of full image scan \n");
    }
    else
    {
        fprintf(system_validation,"Max ADC value for half image is half of full image scan \n");
    }

    /* One more scan for different API */
    NNO_setScanSubImage(475, 4);
    RegtestUtil_PerformScan();

    fileSize = NNO_GetFileSizeToRead(NNO_FILE_SCAN_DATA);

    if((size = NNO_GetFile((unsigned char *)pData, fileSize)) == fileSize)
    {
        dlpspec_scan_interpret(pData, fileSize, &scan_results);
    }

    median = RegtestUtil_find_median((int*)&scan_results.intensity[0], scan_results.length);
    if(median > 10000)
    {
        fprintf(reg_test , "NNO_setScanSubImage-FAIL-Full pattern scan happening though it should not happen\n");
    }
    else
    {
        fprintf(reg_test , "NNO_setScanSubImage-PASS\n");
    }

    NNO_setScanSubImage(0 , 479);
    fclose(system_validation);
    fclose(reg_test);
    free(pData);
}

void regressiontest::Regtest_DLPCLampControl(void)
{
    FILE* reg_test;
    uint32 red , green , blue;

    reg_test = fopen("regression_test_report.txt" , "a");
    if(reg_test == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open regression_test_report.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }
    if(false == scan_config_good)
    {
        fprintf(reg_test , "No Scan Allowed as scan config failed\n");
    }
    else
    {
        NNO_SetScanControlsDLPCOnOff(false);

        NNO_DLPCEnable(true, false);

        RegtestUtil_PerformScan();
        NNO_GetPhotoDetector(&red,&green,&blue);

        if(green>3000)
            fprintf(reg_test , "NNO_SetScanControlsDLPCOnOff-FAIL-lamp is on though switched off in code\n");
        else
            fprintf(reg_test , "NNO_SetScanControlsDLPCOnOff-PASS\n");

        NNO_DLPCEnable(false, false);
        NNO_SetScanControlsDLPCOnOff(true);

    }

    fclose(reg_test);
}

/* Test All the Scan Configuration APIs */
void regressiontest::Regtest_ScanConfiguration(void)
{
    FILE* reg_test;


    reg_test = fopen("regression_test_report.txt" , "a");
    if(reg_test == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open regression_test_report.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }
    NNO_EraseAllScanCfg();

    if(1 == NNO_GetNumScanCfg())
    {
        fprintf(reg_test , " NNO_GetNumScanCfg-PASS\n" );
        fprintf(reg_test , " NNO_EraseAllScanCfg-PASS\n" );
    }
    else
    {
        fprintf(reg_test , " NNO_GetNumScanCfg-FAIL-Returned scan config greater then 1 even when scan configs have been erased\n" );
        fprintf(reg_test , " NNO_EraseAllScanCfg-FAIL-Returned scan config greater then 1 even when scan configs have been erased\n" );
    }

    uScanConfig config;
    config.scanCfg.scan_type = COLUMN_TYPE;
    config.scanCfg.num_patterns = 213;
    config.scanCfg.num_repeats = 20;
    config.scanCfg.wavelength_start_nm = MIN_WAVELENGTH;
    config.scanCfg.wavelength_end_nm = MAX_WAVELENGTH;
    config.scanCfg.width_px = 6;
    strcpy(config.scanCfg.config_name, "testcfg1");
    RegtestUtil_apply_scan_cfg_index_to_device(&config,1);

    if(2 == NNO_GetNumScanCfg())
    {
        fprintf(reg_test , " NNO_SaveScanCfgInEVM-PASS\n" );
    }
    else
    {
        fprintf(reg_test , " NNO_SaveScanCfgInEVM-FAIL-Value returned is not equal to 1\n" );
    }

    config.scanCfg.scan_type = COLUMN_TYPE;
    config.scanCfg.num_patterns = 228;
    config.scanCfg.num_repeats = 20;
    config.scanCfg.wavelength_start_nm = MIN_WAVELENGTH;
    config.scanCfg.wavelength_end_nm = MAX_WAVELENGTH;
    config.scanCfg.width_px = 6;
    strcpy(config.scanCfg.config_name, "testcfg2");
    RegtestUtil_apply_scan_cfg_index_to_device(&config,2);

    if(3 == NNO_GetNumScanCfg())
    {
        NNO_SetActiveScanIndex(1);/* REVIEW starting check getactivescanindex */
        if(1 == NNO_GetActiveScanIndex())
        {
            fprintf(reg_test , " NNO_GetActiveScanIndex-PASS\n" );
            fprintf(reg_test , " NNO_SetActiveScanIndex-PASS\n" );
        }
        else
        {
            fprintf(reg_test , " NNO_GetActiveScanIndex-FAIL-Value is equal to %d, though set to 1\n",NNO_GetActiveScanIndex() );
            fprintf(reg_test , " NNO_SetActiveScanIndex FAIL-Value is equal to %d, though set to 1\n",NNO_GetActiveScanIndex());
        }
    }
    fclose(reg_test);
}

int regressiontest::Regtest_read_scan_cfg_index_to_device(void)
{
    FILE* reg_test;
    scanConfig* pCfg;
    size_t bufferSize = sizeof(uScanConfig)*4;
    void *pBuffer = malloc(bufferSize);
    int ret;

    if(pBuffer ==NULL)
    {
        return -1;
    }
    reg_test = fopen("regression_test_report.txt" , "a");
    if(reg_test == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open regression_test_report.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return -1;
    }
    ret = NNO_GetScanCfg( 2 , pBuffer , (uint32*)&bufferSize);
    dlpspec_scan_read_configuration(pBuffer, bufferSize);
    pCfg = (scanConfig*)(pBuffer);
    if(228 == pCfg->num_patterns)
    {
        fprintf(reg_test , " NNO_GetScanCfg-PASS\n" );
    }
    else
    {
        fprintf(reg_test , " NNO_GetScanCfg-FAIL-Returned number of patterns %d not equal to 228\n",pCfg->num_patterns );
    }
    free(pBuffer);
    fclose(reg_test);
    return ret;
}

/* Sensor API testing */

void regressiontest::Regtest_sensortest(void)
{
    FILE* reg_test;
    int ambient , detector , hdc_temp , tiva_temp;
    uint32 humidity , pBatt_Volt;

    reg_test = fopen("regression_test_report.txt" , "a");
    if(reg_test == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open regression_test_report.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }

    NNO_ReadTemp(&ambient, &detector);
    NNO_ReadHum(&humidity, &hdc_temp);
    NNO_ReadTivaTemp(&tiva_temp);
    NNO_ReadBattVolt(&pBatt_Volt);


    pBatt_Volt = pBatt_Volt / 100.0;
    ambient = ambient/100;
    detector = detector/100;
    humidity = humidity/100;
    hdc_temp = hdc_temp/100;
    tiva_temp = tiva_temp/100;

    if((ambient > 20 && ambient < 70)&&(detector > 20 && detector < 70))
    {
        fprintf(reg_test , " NNO_ReadTemp-PASS\n" );
    }
    else
    {
        fprintf(reg_test , " NNO_ReadTemp-FAIL-Ambient temperature is not in range [20-70C]\n" );
    }
    if((hdc_temp > 20 && hdc_temp < 70)&&(humidity > 10 && humidity < 80))
    {
        fprintf(reg_test , " NNO_ReadHum-PASS\n" );
    }
    else
    {
        fprintf(reg_test , " NNO_ReadHum-FAIL-Humidity is not in range [10-80%%]\n" );
    }
    if((tiva_temp > 20 && tiva_temp < 70))
    {
        fprintf(reg_test , " NNO_ReadTivaTemp-PASS\n" );
    }
    else
    {
        fprintf(reg_test , " NNO_ReadTivaTemp-FAIL-Tiva Temperature not in range [20-70C]\n" );
    }
    if((pBatt_Volt < 10))
    {
        fprintf(reg_test , " NNO_ReadBattVolt-PASS\n" );
    }
    else
    {
        fprintf(reg_test , " NNO_ReadBattVolt-FAIL-Battery voltage is greater than 10\n" );
    }
    fclose(reg_test);
}

void regressiontest::Regtest_DateTime(void)
{
    FILE* reg_test;

    QDate currentDate = QDate::currentDate();
    QTime currentTime = QTime::currentTime();
    uint8 year, month, day, wday;
    uint8 hour, minute, second;

    uint8 gyear, gmonth, gday, gwday;
    uint8 ghour, gminute, gsecond;

    reg_test = fopen("regression_test_report.txt" , "a");
    if(reg_test == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open regression_test_report.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }

    year = currentDate.year() - 2000;
    month = currentDate.month();
    day = currentDate.day();
    wday = currentDate.dayOfWeek();

    hour = currentTime.hour();
    minute = currentTime.minute();
    second = currentTime.second();

    NNO_SetDateTime( year, month, day, wday, hour, minute, second );/* not required */
    NNO_GetDateTime( &gyear, &gmonth, &gday, &gwday, &ghour, &gminute, &gsecond );

    if((year == gyear)&&(month == gmonth)&&(day == gday)&&(wday==gwday)&&(hour==ghour)&&(minute==gminute)&&(second== gsecond))
    {
        fprintf(reg_test , " NNO_SetDateTime-PASS\n" );
        fprintf(reg_test , " NNO_GetDateTime-PASS\n" );
    }
    else
    {
        fprintf(reg_test , " NNO_SetDateTime-FAIL-Retrieved Date is not equal to current date\n" );
        fprintf(reg_test , " NNO_GetDateTime-FAIL-Retrieved Date is not equal to current date\n" );
    }

    fclose(reg_test);
}

void regressiontest::Regtest_PGA(void)
{
    FILE* reg_test;
    uint8 pga;

    NNO_SetPGAGain(32);
    pga = NNO_GetPGAGain();
    reg_test = fopen("regression_test_report.txt" , "a");
    if(reg_test == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open regression_test_report.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }

    if(32 == pga)
    {
        fprintf(reg_test , " NNO_SetPGAGain-PASS\n" );
        fprintf(reg_test , " NNO_GetPGAGain-PASS\n" );
    }
    else
    {
        fprintf(reg_test , " NNO_SetPGAGain-FAIL-Retrieved Gain is not equal to set PGA Gain 32\n" );
        fprintf(reg_test , " NNO_GetPGAGain-FAIL-Retrieved Gain is not equal to set PGA Gain 32\n" );
    }
    fclose(reg_test);
}

void regressiontest::Regtest_SerialNumber(void)
{
    FILE* reg_test;
    char sernum[NANO_SER_NUM_LEN+1];
    char testsernum[NANO_SER_NUM_LEN+1];
    char test_ser_num[NANO_SER_NUM_LEN+1];

    reg_test = fopen("regression_test_report.txt" , "a");
    if(reg_test == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open regression_test_report.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }


    test_ser_num[0] = '5';
    test_ser_num[1] = '0';
    test_ser_num[2] = '5';
    test_ser_num[3] = '0';
    test_ser_num[4] = '0';
    test_ser_num[5] = '0';
    test_ser_num[6] = '1';
    test_ser_num[7] = '\0';
    test_ser_num[8] = '\0';

    sernum[NANO_SER_NUM_LEN] = '\0';
    testsernum[NANO_SER_NUM_LEN] = '\0';

    if(NNO_GetSerialNumber(sernum) != PASS)
    {
        fprintf(reg_test , " NNO_GetSerialNumber-FAIL-API Failed\n" );
        fclose(reg_test);
        return;
    }
    if(NNO_SetSerialNumber(test_ser_num)!= PASS)
    {
        fprintf(reg_test , " NNO_SetSerialNumber-FAIL-API Failed\n" );
        fclose(reg_test);
        return;
    }
    if(NNO_GetSerialNumber(testsernum) != PASS)
    {
        fprintf(reg_test , " NNO_GetSerialNumber-FAIL-API Failed\n" );
        fclose(reg_test);
        return;
    }
    if(0 == strcmp(testsernum, test_ser_num))
    {
        fprintf(reg_test , " NNO_GetSerialNumber-PASS\n" );
        fprintf(reg_test , " NNO_SetSerialNumber-PASS\n" );
    }
    else
    {
        fprintf(reg_test , " NNO_GetSerialNumber-FAIL-Retrieved Serial number is not equal to set Serial number 5050001\n" );
        fprintf(reg_test , " NNO_SetSerialNumber-FAIL-Retrieved Serial number is not equal to set Serial number 5050001\n" );
    }
    if(NNO_SetSerialNumber(sernum)!= PASS)
     {
        fclose(reg_test);
        return;
    }

    fclose(reg_test);
}

void regressiontest::Regtest_GeneratePatterns(void)
{
    FILE* reg_test;
    int ret;

    reg_test = fopen("regression_test_report.txt" , "a");
    if(reg_test == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open regression_test_report.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }
    ret = NNO_GenCalibPatterns(SLIT_ALIGN_SCAN);

    if(213 == ret)
    {
        fprintf(reg_test , " NNO_GenCalibPatterns-PASS\n" );
    }
    else
    {
        fprintf(reg_test , " NNO_GenCalibPatterns-FAIL-Generated Patterns not equal to 213\n" );
    }
    fclose(reg_test);
}

void regressiontest::Regtest_SaveRefCal(void)
{
    FILE* reg_test;
    size_t fileSize;
    scanData *pData;
    uint32 size;
    scanResults scan_results;

    reg_test = fopen("regression_test_report.txt" , "a");
    if(reg_test == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open regression_test_report.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }
    fileSize = NNO_GetFileSizeToRead(NNO_FILE_SCAN_DATA);

    pData = (scanData *)malloc(fileSize); /* PERFORM REF CAL */

    if(pData == NULL)
    {
        fprintf(reg_test , "Malloc Fails in Perform scan\n");
    }
    else
    {
        if((size = NNO_GetFile((unsigned char *)pData, fileSize)) == fileSize)
        {
            dlpspec_scan_interpret(pData, fileSize, &scan_results);
        }
        else
        {
            fprintf(reg_test , "NNO_GetFile-FAIL-size %d does not match fileSize %d\n", size , fileSize);
            fclose(reg_test);
            free(pData);
            return;
        }

        if(228 == scan_results.length)
        {
            fprintf(reg_test , " NNO_SaveRefCalPerformed-PASS\n" );
        }
        else
        {
            fprintf(reg_test , " NNO_SaveRefCalPerformed-FAIL-Ref cal scan results show patterns as %d which should be 228\n",scan_results.length );
        }
        free(pData);
    }
    fclose(reg_test);
}

void regressiontest::Regtest_SNRTest(QString str)
{
    FILE* reg_test;
    int snr17 , snr133 , snr600;
    QString snrhadpeak;

    if("" == str)
        return;
    QStringList pieces = str.split("=");
    snrhadpeak = pieces.at(2);

    reg_test = fopen("regression_test_report.txt" , "a");
    if(reg_test == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open regression_test_report.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }
    int snrhad = snrhadpeak.toInt();
    NNO_GetSNRData(&snr17 , &snr133 , &snr600);

    if((snr17 > 100)&&(snr133 > 100)&&(snr600 > 100)&&(snrhad > 100))
    {
        fprintf(reg_test , " NNO_StartSNRScan-PASS\n" );
        fprintf(reg_test , " NNO_StartHadSNRScan-PASS\n" );
        fprintf(reg_test , " NNO_GetSNRData-PASS\n" );
    }
    else
    {
        fprintf(reg_test , " NNO_StartSNRScan-FAIL-SNR Values less than 100\n" );
        fprintf(reg_test , " NNO_StartHadSNRScan-FAIL-SNR Values less than 100\n" );
        fprintf(reg_test , " NNO_GetSNRData-FAIL-SNR Values less than 100\n" );
    }
    fclose(reg_test);
}

void regressiontest::Regtest_NNODLPCReg(void)
{
    FILE* reg_test;
    uint32 value;

    /* Test for DLPC150 Register */
    reg_test = fopen("regression_test_report.txt" , "a");
    if(reg_test == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open regression_test_report.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }
    NNO_DLPCEnable(true, false);
    NNO_SetDLPCReg(0x40015004, 2);
    NNO_GetDLPCReg(0x40015004, &value);
    if(reg_test != NULL)
    {
        if(2 == value)
        {
            fprintf(reg_test , "NNO_SetDLPCReg-PASS\n");
            fprintf(reg_test , "NNO_GetDLPCReg-PASS\n");
        }
        else
        {
            fprintf(reg_test , "NNO_SetDLPCReg-FAIL-Returned register value not equal to 2\n");
            fprintf(reg_test , "NNO_GetDLPCReg-FAIL-Returned register value not equal to 2\n");
        }
        fclose(reg_test);
    }
}

void regressiontest::Regtest_NNOSDCard(void)
{
    FILE* reg_test;
    int valpre , valpost;
    uint32 device_status;

    reg_test = fopen("regression_test_report.txt" , "a");
    if(reg_test == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open regression_test_report.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }
    NNO_ReadDeviceStatus(&device_status);

    if((device_status & 0x00000001) == 1)
    {
        fprintf(reg_test, "NNO_ReadDeviceStatus-PASS\n");
    }
    else
    {
        fprintf(reg_test, "NNO_ReadDeviceStatus-FAIL\n");
    }

    if(((device_status & 0x00000004)>>2) == 0)
    {
        fprintf(reg_test, "NNO_GetNumScanFilesInSD-NOSDCARD\n");
        fprintf(reg_test, "NNO_DeleteLastScanFileInSD-NOSDCARD\n");
        fclose(reg_test);
        return;
    }

    valpre = NNO_GetNumScanFilesInSD();/* RETURN DIFFERENT FOR NO CARD */

    if(0 == valpre)
    {
        fprintf(reg_test, "NNO_GetNumScanFilesInSD-PASS-NOSCANS\n");
        fprintf(reg_test, "NNO_DeleteLastScanFileInSD-PASS-NOSCANS\n");
    }
    else if(valpre > 0)
    {
        fprintf(reg_test, "NNO_GetNumScanFilesInSD-PASS\n");
        NNO_DeleteLastScanFileInSD();
        valpost = NNO_GetNumScanFilesInSD();
        if( valpost == (valpre -1))
        {
            fprintf(reg_test, "NNO_DeleteLastScanFileInSD-PASS\n");
        }
        else
        {
            fprintf(reg_test, "NNO_DeleteLastScanFileInSD-FAIL-After deleting one scan file the number of scan files is not less by 1\n");
        }
    }
    else
    {
        fprintf(reg_test, "NNO_GetNumScanFilesInSD-FAIL-The number of scan returned less than 0\n");
    }
    fclose(reg_test);
}



//Do for send calibrate function
/* Utility functions */
void regressiontest::RegtestUtil_generatehtmlreport(void)
{
    QString apiname;
    QString result;
    QString remark;
    QFile filet("regression_test_report.txt");
    QFile fileh("regression_test_report.html");
    if (!filet.open(QIODevice::ReadOnly | QIODevice::Text))
            return;

    if (!fileh.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open regression_test_report.html.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;

    }
    QTextStream in(&filet);
    QTextStream out(&fileh);

    out << "<table border = 1 rules = rows>" << "\n";
    out << "<tr>" << "\n";
    out << "<th width='10%' align=justify bgcolor = '#d3d3d3'>API Name</th>" << "\n";
    out << "<th width='10%' align=justify bgcolor = '#d3d3d3'>Result</th>" << "\n";
    out << "<th width='10%' align=justify bgcolor = '#d3d3d3'>Remark</th>" << "\n";
    out << "</tr>" << endl;
    while (!in.atEnd()) {
            QString line = in.readLine();
            QStringList line_piece = line.split("-");



            apiname = line_piece.at(0);
            result  = line_piece.at(1);

            if(result == "FAIL")
                remark  = line_piece.last();
            else
                remark = "";

            out << "<tr>" << "\n" ;
            out << "<td width='10%' align=justify>" <<apiname << "</td>" << endl;
            out << "<td width='10%' align=justify>" <<result<< "</td>" << "\n";
            out << "<td width='10%' align=justify>" <<remark<< "</td>" << "\n";
            out << "</tr>" << "\n";

    }
    out << "</table>" << "\n";
    filet.close();
    fileh.close();
}

int regressiontest::RegtestUtil_apply_scan_cfg_to_device(uScanConfig *pCfg)
{
    size_t bufferSize;
    void *pBuffer;
    int ret;

    ret = dlpspec_get_scan_config_dump_size(pCfg, &bufferSize);
    if(ret != DLPSPEC_PASS)
        return FAIL;

    pBuffer = malloc(bufferSize);

    if(pBuffer == NULL)
    {
        return FAIL;
    }

    if(dlpspec_scan_write_configuration(pCfg, pBuffer, bufferSize) != DLPSPEC_PASS)
        return FAIL;
    ret = NNO_ApplyScanConfig(pBuffer, bufferSize);
    free(pBuffer);
    return ret;
}

int regressiontest::RegtestUtil_apply_scan_cfg_index_to_device(uScanConfig *pCfg , int index)
{
    size_t bufferSize;
    void *pBuffer;
    int ret;

    ret = dlpspec_get_scan_config_dump_size(pCfg, &bufferSize);
    if(ret != DLPSPEC_PASS)
        return FAIL;

    pBuffer = malloc(bufferSize);

    if(pBuffer == NULL)
    {
        return FAIL;
    }

    if(dlpspec_scan_write_configuration(pCfg, pBuffer, bufferSize) != DLPSPEC_PASS)
        return FAIL;
    ret = NNO_SaveScanCfgInEVM(index, pBuffer, bufferSize);
    free(pBuffer);
    return ret;
}


uint32 regressiontest::RegtestUtil_find_median(int* val_array, uint32 num_vals)
{
    uint32 i, j, median, temp;

    /* Sorting begins */
    for (i = 0 ; i <= num_vals-1 ; i++)
    {     /* Trip-i begins  */
        for (j = 0 ; j <= num_vals-i ; j++)
        {
          if (val_array[j] <= val_array[j+1])
          { /* Interchanging values */

            temp = val_array[j];
            val_array[j] = val_array[j+1];
            val_array[j+1] = temp;
          }
       }
   } /* sorting ends */

    /* calculation of median  */
    if ( num_vals % 2 == 0)
      median = (val_array[num_vals/2] + val_array[num_vals/2+1])/2 ;
   else
      median = val_array[num_vals/2 + 1];

    return median;
}

int regressiontest::RegtestUtil_find_max(int* val_array, uint32 num_vals)
{
    uint32 i;
    int max = 0;


    for (i = 0 ; i <= num_vals-1 ; i++)
    {
        if(val_array[i] > max)
            max = val_array[i];
    }
    return max;
}

#define MAX_RESOLUTION_LESS_TENPERCENT 7549747
void regressiontest::RegtestUtil_validateMax(int adc_max)
{
    FILE* system_validation;
    float percent;

    system_validation= fopen("system_validation_data.txt" , "a");
    if(system_validation == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open system_validation_data.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }
    percent = (adc_max * 100)/MAX_RESOLUTION_LESS_TENPERCENT;


    fprintf(system_validation,"MAX ADC value is %d percentage of known max 7549747[90percent of 2^23] is %f \n", adc_max , percent);
    if(percent < 90)
    {
        fprintf(system_validation,"MAX ADC value is less than 90percent of known max \n");
    }
    else
    {
        fprintf(system_validation,"MAX ADC value is greater than 90percent of known max \n");
    }
    fclose(system_validation);
}

void regressiontest::RegtestUtil_PerformScan(void)
{
    int scanStatus;
    FILE* reg_test;

    reg_test = fopen("regression_test_report.txt" , "a");
    if(reg_test == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open regression_test_report.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }

        NNO_PerformScan(0);

        //Wait for scan completion
        do
        {
            scanStatus = NNO_GetScanComplete();
        }while(scanStatus == 0);

        if(scanStatus != 1)
        {
            fprintf(reg_test , "USB Communication fail\n");
            fclose(reg_test);
            return;
        }

    fclose(reg_test);
}

int regressiontest::RegtestUtil_cosineSimilarity(int* sample)
{
    FILE* system_validation;
    bool isSimilar ;
    double simVal;
    int ret;


    system_validation= fopen("system_validation_data.txt" , "a");
    if(system_validation == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open system_validation_data.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return -1;
    }

    RegtestUtil_compare_spectrum((int*)sample, (int*)knownSample, 228, 0.99, &isSimilar, &simVal);

    fprintf(system_validation,"=======================Cosine similarity validation for Reference Cal configuration==================\n");

    fprintf(system_validation, "config.scan_type = COLUMN_TYPE\n");
    fprintf(system_validation, "config.num_patterns = 228\n");
    fprintf(system_validation, "config.num_repeats = 10\n");
    fprintf(system_validation, "config.wavelength_start_nm = 900\n");
    fprintf(system_validation, "config.wavelength_end_nm = 1700\n");
    fprintf(system_validation, "config.width_px = 6\n");
    fprintf(system_validation, "Cosine similarity with golden unit = %1.5f\n",simVal);

    fclose(system_validation);
    if(isSimilar)
        ret = 0;
    else
        ret = -1;

    return ret;
}

void regressiontest::RegtestUtil_compare_spectrum(int *testAbsorbance, int *refAbsorbance, int vectorLength, double simThreshold, bool *isSimilar, double *simNumber)
{
    unsigned long sumAiXBi;
    unsigned long sumAiXAi;
    unsigned long sumBiXBi;
    int i;
    double similarityNum = -1.0; //Cos(theta) = -1 to 1

    //http://en.wikipedia.org/wiki/Cosine_similarity => equation used to compare two vectors
    sumAiXBi = 0;
    sumAiXAi = 0;
    sumBiXBi = 0;
    for(i=0; i<vectorLength; i++)
    {
        sumAiXBi +=  (testAbsorbance[i] * refAbsorbance[i]);
        sumAiXAi +=  (testAbsorbance[i] * testAbsorbance[i]);
        sumBiXBi +=  (refAbsorbance[i] * refAbsorbance[i]);
    }

    similarityNum = (double) ( sumAiXBi / ( (sqrt(sumAiXAi) * (sqrt(sumBiXBi)) ) ) );

    if(similarityNum <= simThreshold)
        *isSimilar = FALSE;
    else
        *isSimilar = TRUE;

    if(std::isnan(similarityNum) == true)
        *isSimilar = FALSE;

    printf("Computed Cosine Similarity - %f\n",similarityNum);

    *simNumber = similarityNum;

    return;
}

void regressiontest::RegtestUtil_ValidateScanResults(scanResults* scan_result)
{
    FILE* system_validation;
    double minWavelength = 0, maxWavelength =0;
    system_validation= fopen("system_validation_data.txt" , "a");
    if(system_validation == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open system_validation_data.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return;
    }
    fprintf(system_validation,"=============================Scan Results validation Reference Cal configuration==============================\n");

    /* Validate length */
    if(228 == scan_result->length)
    {
        fprintf(system_validation,"scan_result.length is %d which matches 228\n", scan_result->length);
    }
    else
    {
        fprintf(system_validation,"scan_result.length is %d which does not match 228\n", scan_result->length);
    }

    /* Validate sensor readings */
    if((((scan_result->humidity_hundredths)/100) > 20)||(((scan_result->humidity_hundredths)/100) < 70))
    {
        fprintf(system_validation,"scan_result.humidity is %d within range [20,70]\n", scan_result->humidity_hundredths);
    }
    else
    {
        fprintf(system_validation,"scan_result.humidity is %d not within range [20,70]\n", scan_result->humidity_hundredths);
    }
    if((((scan_result->detector_temp_hundredths)/100) > 20)||(((scan_result->detector_temp_hundredths)/100) < 70))
    {
        fprintf(system_validation,"detector_temp_hundredths is %d within range [20,70]\n", scan_result->detector_temp_hundredths);
    }
    else
    {
        fprintf(system_validation,"detector_temp_hundredths is %d not within range [20,70]\n", scan_result->detector_temp_hundredths);
    }
    if((((scan_result->system_temp_hundredths)/100) > 20)||(((scan_result->system_temp_hundredths)/100) < 70))
    {
        fprintf(system_validation,"system_temp_hundredths is %d within range [20,70]\n", scan_result->system_temp_hundredths);
    }
    else
    {
        fprintf(system_validation,"system_temp_hundredths is %d not within range [20,70]\n", scan_result->system_temp_hundredths);
    }
    if(scan_result->lamp_pd > 3000)
    {
        fprintf(system_validation,"scan_result->lamp_pd is %d which is above 3000\n", scan_result->lamp_pd);
    }
    else
    {
        fprintf(system_validation,"scan_result->lamp_pd is %d which is not above 3000\n", scan_result->lamp_pd);
    }

    /* Validate Scan Configuration */
    if(228 == scan_result->cfg.section[0].num_patterns)
    {
        fprintf(system_validation,"scan_result->cfg.num_patterns is %d which matches 228\n", scan_result->cfg.section[0].num_patterns);
    }
    else
    {
        fprintf(system_validation,"scan_result->cfg.num_patterns is %d which does not match 228\n", scan_result->cfg.section[0].num_patterns);
    }
    if(10 == scan_result->cfg.head.num_repeats)
    {
        fprintf(system_validation,"scan_result->cfg.num_repeats is %d which matches 10\n", scan_result->cfg.head.num_repeats);
    }
    else
    {
        fprintf(system_validation,"scan_result->cfg.num_repeats is %d which does not match 10\n", scan_result->cfg.head.num_repeats);
    }
    if(0 == scan_result->cfg.head.scan_type)
    {
        fprintf(system_validation,"scan_result->cfg.scan_type is %d which matches column type\n", scan_result->cfg.head.scan_type);
    }
    else
    {
        fprintf(system_validation,"scan_result->cfg.scan_type is %d which does not column type\n", scan_result->cfg.head.scan_type);
    }
    if(MAX_WAVELENGTH == scan_result->cfg.section[0].wavelength_end_nm)
    {
        fprintf(system_validation,"scan_result->cfg.wavelength_end_nm is %d which matches 1700\n", scan_result->cfg.section[0].wavelength_end_nm);
    }
    else
    {
        fprintf(system_validation,"scan_result->cfg.wavelength_end_nm is %d which does not match 1700\n", scan_result->cfg.section[0].wavelength_end_nm);
    }
    if(MIN_WAVELENGTH == scan_result->cfg.section[0].wavelength_start_nm)
    {
        fprintf(system_validation,"scan_result->cfg.wavelength_start_nm is %d which matches 900\n", scan_result->cfg.section[0].wavelength_start_nm);
    }
    else
    {
        fprintf(system_validation,"scan_result->cfg.wavelength_start_nm is %d which does not match 900\n", scan_result->cfg.section[0].wavelength_start_nm);
    }
    if(6 == scan_result->cfg.section[0].width_px)
    {
        fprintf(system_validation,"scan_result->cfg.width_px is %d which matches 6\n", scan_result->cfg.section[0].width_px);
    }
    else
    {
        fprintf(system_validation,"scan_result->cfg.width_px is %d which does not match 6\n", scan_result->cfg.section[0].width_px);
    }

    /* Write the pga setting */
    fprintf(system_validation,"scan_result->pga is %d\n", scan_result->pga);

    /* Validate Calibration Coefficients */
    dlpspec_util_columnToNm(0, scan_result->calibration_coeffs.PixelToWavelengthCoeffs, &maxWavelength);
    dlpspec_util_columnToNm(853,scan_result->calibration_coeffs.PixelToWavelengthCoeffs, &minWavelength);

    fprintf(system_validation , "Scan Results wavelength corresponding to column 0 is Maxwavelength -%f\n", maxWavelength);
    fprintf(system_validation , "Scan Results wavelength corresponding to column 853 is Minwavelength-%f\n", minWavelength);
    if ((maxWavelength < MAX_WAVELENGTH) || (minWavelength > MIN_WAVELENGTH))
    {
        fprintf(system_validation, "Scan Results calibration coefficient Min and Max wavelength beyond range[900,1700]\n");
    }
    else
    {
        fprintf(system_validation, "Scan Results calibration coefficient Min and Max wavelength within range[900,1700]\n");
    }
    fclose(system_validation);
}


int regressiontest::Regtest_UtilVerifySimilarityOne(int* max)
{

    uint32 size;
    size_t fileSize;
    scanData *pData;
    scanResults scan_results;
    bool isSimilar ;
    double simVal;
    int ret;
    int scanIntensity[624];
    FILE* reg_test;

    reg_test = fopen("regression_test_report.txt" , "a");
    if(reg_test == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open regression_test_report.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return -1;
    }
    QFile file("KnownTestSample_1.txt");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        fclose(reg_test);
        return -2;
    }
    QTextStream out(&file);

    fileSize = NNO_GetFileSizeToRead(NNO_FILE_SCAN_DATA);
    pData = (scanData *)malloc(fileSize);
    if(pData == NULL)
    {
        fprintf(reg_test , "Malloc Fails in Regtest_UtilVerifySimilarityOne\n");
        fclose(reg_test);
        return FAIL;
    }

    if((size = NNO_GetFile((unsigned char *)pData, fileSize)) == fileSize)
    {
        dlpspec_scan_interpret(pData, fileSize, &scan_results);
    }
    else
    {
        fprintf(reg_test , "NNO_GetFile-FAIL-size %d does not match fileSize %d\n", size , fileSize);
        fclose(reg_test);
        free(pData);
        return FAIL;
    }
    for(int i=0; i<624 ; i++)
    {
        QString line = out.readLine();
        scanIntensity[i] = line.toInt();
    }

    *max = RegtestUtil_find_max((int*)&scan_results.intensity[0], scan_results.length);

    RegtestUtil_compare_spectrum((int*)&scan_results.intensity[0], (int*)&scanIntensity[0], 624, 0.99, &isSimilar, &simVal);


    if(isSimilar)
        ret = 0;
    else
        ret = -1;


    free(pData);
    fclose(reg_test);

    return ret;
}


int regressiontest::Regtest_UtilVerifySimilarityTwo(int* max)
{

    uint32 size;
    size_t fileSize;
    scanData *pData;
    scanResults scan_results;
    bool isSimilar ;
    double simVal;
    int ret;
    int scanIntensity[624];
    FILE* reg_test;

    reg_test = fopen("regression_test_report.txt" , "a");
    if(reg_test == NULL)
    {
        QString text;
        QString title("Error Message");
        text.sprintf("Unable to open regression_test_report.txt.\n Please close if the file is open in other editor.");
        QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton );
        msgBox.exec();
        return -1;
    }
    QFile file("KnownTestSample_2.txt");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        fclose(reg_test);
        return -2;
    }

    QTextStream out(&file);

    fileSize = NNO_GetFileSizeToRead(NNO_FILE_SCAN_DATA);
    pData = (scanData *)malloc(fileSize);
    if(pData == NULL)
    {
        fprintf(reg_test , "Malloc Fails in Regtest_UtilVerifySimilarityOne\n");
        fclose(reg_test);
        return FAIL;
    }

    if((size = NNO_GetFile((unsigned char *)pData, fileSize)) == fileSize)
    {
        dlpspec_scan_interpret(pData, fileSize, &scan_results);
    }
    else
    {
        fprintf(reg_test , "NNO_GetFile-FAIL-size %d does not match fileSize %d\n", size , fileSize);
        fclose(reg_test);
        free(pData);
        return FAIL;
    }
    for(int i=0; i<624 ; i++)
    {
        QString line = out.readLine();
        scanIntensity[i] = line.toInt();
    }

    RegtestUtil_compare_spectrum((int*)&scan_results.intensity[0], (int*)&scanIntensity[0], 624, 0.99, &isSimilar, &simVal);

    *max = RegtestUtil_find_max((int*)&scan_results.intensity[0], scan_results.length);
    if(isSimilar)
        ret = 0;
    else
        ret = -1;


    fclose(reg_test);
    free(pData);

    return ret;
}

