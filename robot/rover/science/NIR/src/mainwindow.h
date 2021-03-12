/*****************************************************************************
**
**  Copyright (c) 2015 Texas Instruments Incorporated.
**  ALL RIGHTS RESERVED
**
******************************************************************************/
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#pragma once

#include <QMainWindow>
#include <QSettings>
#include <QProgressBar>
#include <QTextStream>
#include <QFileDialog>
#include <QFile>
#include <QDir>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsTextItem>
#include <QDesktopServices>
#include <QMessageBox>
#include <QThread>

#include "spectrum.h"
#include "dlpspec_scan.h"
#include "regressiontest.h"

#include "ui_scanconfigdialog.h"
#include "lmdfu.h"
#include "Common.h"
#include "qgraphicsitem.h"
#include "verticallabel.h"
#include "math.h"
#include "dlpspec_calib.h"
#include "dlpspec_util.h"
#include "dlpspec_version.h"
#include "dlpspec_helper.h"
#include "dlpspec_setup.h"
#include <fstream>
#include <iostream>
#include "lmusbdll.h"
#include "firmware.h"
#include "NNOSNRDefs.h"
#include "usb.h"
#include "API.h"
#include "filepath.h"
#include "scanconfiglist.h"
#include "scanconfigdialog.h"
#include "evm.h"
#include "filesettings_window.h"
#define USB_POLL_ENABLED
#undef PATTERN_WIDTH_CONTROL

#define MAX_SCAN_PEAKS 24
#define SLIT_ALIGN_NUM_SCANS_AVG 6
#define NUM_REPEATS_CALIBRATION 50
#define SCAN_RANGE 3
#define MAX_NUM_COEFF 3  /* in the order c , b , a */
#define FULL_DMD_SCAN 2

#define DET_ALIGN_MIN1 74000
#define DET_ALIGN_MIN2 281000
#define DET_ALIGN_MIN3 12000

#define SLIT_ALIGN_MIN1 4200
#define SLIT_ALIGN_MIN2 7500
#define SLIT_ALIGN_MIN3 8500

#define SNR_THRESHOLD1 800
#define SNR_THRESHOLD2 1200
#define SNR_THRESHOLD3 2600

#define PIXEL_WIDTH  (MAX_WAVELENGTH - MIN_WAVELENGTH)/( 854 * 0.8)
#define NUM_WIDTH_ITEMS 60
#define DEFAULT_WIDTH_INDEX 5
#define MIN_PIXEL_INDEX 2

/**
 * @brief Maximum for production.
 */
#define WAVELENGTH_ERROR_LIMIT 2

/**
 * @brief Maximum FWHM for production.
 */
#define FWHM_PASS_MAX_THRESHOLD 12

/**
 * @brief Minimum FWHM for production, to guard against errant noisy data being
 * counted as passing.
 */
#define FWHM_PASS_MIN_THRESHOLD 5

#undef SWEEP_TEST
#undef HADAMARD


typedef enum
{
    SCAN_REF_FACTORY,
    SCAN_REF_PREV,
    SCAN_REF_NEW,
    SCAN_REF_MAX
}SCAN_REF_TYPE;


class ErrorPushButton: public QPushButton
{

  QString errMesg;

  void mousePressEvent(QMouseEvent *e);

public:

  void setErrorMessage(QString text){errMesg = text;}

};

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();


public slots:
    void PopulateScanCfgListComboBox();
     int readVersionAndUpdate(void);

signals:
    void SendScanCfgList(QList<scanConfig> &);
    
private slots:
    void on_pushButton_dlpc_fw_browse_clicked();

    void on_pushButton_dlpc_fw_update_clicked();

    int on_pushButton_test_eeprom_clicked();

    void on_pushButton_test_adc_clicked();

    int on_pushButton_test_bq_clicked();

    int on_pushButton_test_battery_clicked();

    int on_pushButton_test_sdram_clicked();

    void on_pushButton_test_tmp_clicked();

    void on_pushButton_test_hdc_clicked();

    int on_pushButton_test_bt_clicked();

    int on_pushButton_test_sdc_clicked();

    void timerTimeout(void);

    void on_pushButton_scan_clicked();

    void check_USB_Connection();

    void on_pushButton_startDetCal_clicked();

    void on_pushButton_stopDet_cal_clicked();

    void on_pushButton_startSlit_Cal_clicked();

    void on_pushButton_stopSlit_Cal_clicked();

    void on_pushButton_reg_set_clicked();

    void on_pushButton_reg_get_clicked();

    void on_comboBox_scanID_currentIndexChanged(int index);

    int on_pushButton_test_dlpc150_clicked();

    void on_pushButton_runAll_tests_Tiva_clicked();

    void on_pushButton_runAll_tests_DLPC_clicked();

    void on_pushButton_runAll_tests_Detector_clicked();

    void on_pushButton_CalStart_clicked();

    void on_comboBox_calID_currentIndexChanged(int index);

    void on_pushButton_edit_scan_config_clicked();

    void on_pushButton_tiva_fw_browse_clicked();
#ifdef Q_OS_WIN
    int DownloadImage(tLMDFUHandle hHandle);
#endif
    void on_pushButton_tiva_fw_usb_update_clicked();

    void Populate_ScanList();

    void on_pushButton_dir_change_clicked();

    void on_pushButton_customSaveScanDir_clicked();

    void on_pushButton_ScanTab_clicked();

    void on_pushButton_UtilsTab_clicked();

    void on_pushButton_FactoryTab_clicked();

    void on_pushButton_TestTab_clicked();

    void on_pushButton_readSensors_clicked();

    //added the Information tab - KV
    void on_pushButton_InformationTab_clicked();

    void on_ti_e2e_pushButton_clicked();

    void on_pushButton_SyncDateTime_clicked();

    void on_pushButton_GetDateTime_clicked();

    void on_pushButton_import_scanData_clicked();

    void on_pushButton_SetSerNumber_clicked();

    void on_pushButton_GetSerNumber_clicked();

    void on_pushButton_SetModelName_clicked();

    void on_pushButton_GetModelName_clicked();

    int on_pushButton_eeprom_mass_erase_clicked();

    int on_pushButton_eeprom_write_generic_clicked();

    int on_pushButton_eeprom_write_clicked();

    int on_pushButton_eeprom_read_clicked();

    int on_pushButton_test_led_clicked();

    int on_pushButton_reset_tiva_clicked();

    void on_pushButton_hibernate_clicked();

    void on_checkBox_hibernateEnabled_clicked();

    void SetGUIControls(bool);

    void on_pushButton_dumpreg_clicked();

    void on_pushButton_sidebar_hide_clicked();

    void on_radioButton_ref_factory_clicked(bool checked);

    void on_radioButton_ref_prev_clicked(bool checked);

    void on_radioButton_ref_new_clicked(bool checked);

    void on_pushButton_Intensity_clicked();

    void on_pushButton_Absorbance_clicked();

    void on_pushButton_Reflectance_clicked();

    void on_checkBox_overlay_clicked();
    
    void on_listWidget_savedscans_currentItemChanged(QListWidgetItem *current, QListWidgetItem *previous);

    void on_pushButton_regtest_clicked();

    void on_pushButton_UpdateRefCalData_clicked();

    void on_pushButton_erase_flash_clicked();

    void on_pushButton_errorStatus_clicked();

    void on_pushButton_dat_files_browse_clicked();

    void on_pushButton_dat_to_csv_clicked();

    void on_radioButton_Prefix_clicked(bool checked);

    void on_radioButton_FileName_clicked(bool checked);

    void on_spinBox_numRepeat_editingFinished();

    void on_spinBox_numRepeat_valueChanged(int arg1);

    void on_UART_Check_clicked();

    void on_checkBox_always_on_toggled(bool checked);

    void on_Filesettings_clicked();
public:
    Ui::MainWindow *ui;
    void UpdateErrorStatus(NNO_error_status_struct *);
    bool event(QEvent *ev);

    bool saveone_csv;
    bool saveseperate_csv;
    bool absorbance_csv;
    bool intensity_csv;
    bool reflectance_csv;

    bool save_JCAMP;
    bool absorbance_JCAMP;
    bool intensity_JCAMP;
    bool reflectance_JCAMP;

    bool savebinary;



    bool ifsavesep_csv();
    bool ifsaveone_csv();
    bool ifabsorbance_csv();
    bool ifintensity_csv();
    bool ifreflectance_csv();

    bool ifsave_JCAMP();
    bool ifabsorbance_JCAMP();
    bool ifintensity_JCAMP();
    bool ifreflectance_JCAMP();

    bool ifsavebinary();

    void putsavesep_csv(bool);
    void putsaveone_csv(bool);
    void putabsorbance_csv(bool);
    void putintensity_csv(bool);
    void putreflectance_csv(bool);

    void putsave_JCAMP(bool);
    void putabsorbance_JCAMP(bool);
    void putintensity_JCAMP(bool);
    void putreflectance_JCAMP(bool);

    void putsavebinary(bool);

private:
    void initUI(void);

    void showError(const char *str);

    int writeFile(unsigned char *pByteArray, long long sizeInBytes, QProgressBar *progressBar, bool updateBar);

    void SetDetAlignSliderMaxVal(int val);

    bool read_reference_spectrum(QString FineName = "", bool readblob = false);

    void plot_absorbance(QGraphicsScene *scene = NULL, bool saved_scan = false,QString Filename = "");

    void plot_intensities(QGraphicsScene *scene = NULL, bool saved_scan = false);

    void mark_labels(PLOT_TYPE type);

    void save_status();

    void restore_status();

    void apply_scan_config_to_ui(uScanConfig *pConfig);

    void on_USB_Connection();

    int PerformRefCalScan(void *);

    void compare_absorption_spectrum(double *testAbsorbance, double *refAbsorbance, int vectorLength, double simThreshold, bool *isSimilar, double *simNumber);

    double GetInterpolatedWavelength(double pixel);

    int DoReferenceCalibration(void);

    int DoWavelengthVerification();

    int DoSystemTest(bool *pSimilar, double *pSimVal);

    int DoSNRComputation( int );

    void clearTivaTestResults();

    void clearDLPCTestResults();

    void clearDetectorTestResults();

    void initDetectorAlignment(void);

    void initSlitAlignment(void);

    void get_num_files_in_SD_card();

    void PlotSpectrum(void);

    int PerformFullDMDScan(CALIB_SCAN_TYPES type, double *values, int *pSampleLen);

    void InitStatusBar();

    int FindNPeaks(double *pValues, int sampleLen, int expectedNumPeaks, int *pPeakIndices);

    void UpdateDeviceStatus(int deviceStatus);

    int PerformScanReadData(bool StoreInSD, int numRepeats, void *pData, int *bytesRead);

    int InterpretScanData(void *pData, int *pBytesRead);

    int GetSelectedScanConfig(scanConfig *pCfg, int combobox_index);

    void on_pushButton_interpret_clicked();

    int ScanRMNIRSample(double *pAbsorbance, int *pSampleLen);

    int Find_RMNIR_Peaks(double *rm_nir_peak_indices_interp, int *p_num_rm_nir_peaks);

    int Find_AR1_Peaks(double *ar1_peak_indices, int *p_num_ar1_peaks_per_scan);

    int SetPGAGain(int val);

    int ValidateCalResults(void);

    void GetPGASetting(bool *is_fixed, int *fixed_pga_val);

    QTimer *usbPollTimer;

    QString get_timestamp_string(scanResults *pData);

    ScanConfigDialog scan_cfg_dialog;
    QSettings uiSettings;

    //
    Filesettings_window *filewindow;
    Spectrum spectrum;

    ScanConfigList scancfglist;
    regressiontest regtest;

    bool detCalStop, slitCalStop;
    QGraphicsScene *scene;
    bool checkversion;

    bool b_serNumSet;
    uint8 prevRefDataBlob[SCAN_DATA_BLOB_SIZE];
    int DoWavelengthCalibration();
    char errorString[50];
    QVector<double> curIntensity;//to store the current scan intensities
    tLMDFUErr ResetTiva(tLMDFUDeviceState *pState,bool bCheckStatus,
          unsigned char *pcData, int iLength);

    QString refFile_timeStamp;
    QString scanFileName;

    QStringList datfileNames;


    bool b_VersionWindowSkipped; // check back on the context
    bool b_TivaSWCorrect;
    bool b_DLPCFWCorrect;

    int minAlignment_Short; //local variable
    int minAlignment_Medium;
    int minAlignment_Long;
    SCAN_REF_TYPE m_ref_selection; // scanUIselections
};

#endif // MAINWINDOW_H
