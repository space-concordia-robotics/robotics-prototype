/*****************************************************************************
 * mainwindow.cpp
 *
 * This module provides function handlers and supporting functions for different controls on Nirscan Nano UI MainWindow.
 *
 * Copyright (c) 2015 Texas Instruments Incorporated.
 * ALL RIGHTS RESERVED
 *
 ******************************************************************************/
#include <QImage>
#include <QTimer>
#include <QTime>
#include <QDirIterator>

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "scanconfigdialog.h"
#include "filesettings_window.h"
#include "ui_scanconfigdialog.h"
#include "version.h"
#include "bluetoothdialog.h"
#include "NNOCommandDefs.h"
#include "NNOStatusDefs.h"
#include <QTableWidget>
#include "Serial.h"
#include "versiondialog.h"
char *g_pszFile      = NULL;
static int num_tabs_in_gui = 5;
bool factory_gui = false;
//bool cont_scan_flag = false;
bool g_BluetoothTest;// = false;
extern bool g_StartupCompleted;
EVM evm;
FilePath filepath;
qint64 lastScanTimeMS = 0;

//custom pusButton on StatusWidget to show corresponding error
void ErrorPushButton::mousePressEvent(QMouseEvent *e)
/**
 * This function handles the mousePressEvent for Nano custom ErrorPushButton
 *
 */
{
    if(e == NULL)
        return;

    QString title("Error Message");
    if(errMesg != "")
    {
        QMessageBox msgBox(QMessageBox::Warning, title, errMesg, QMessageBox::NoButton, this);
        msgBox.exec();
    }
}


MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow),
    uiSettings("Texas Instruments", "NIRscan Nano GUI")
{
    ui->setupUi(this);
    checkversion = true;
    b_VersionWindowSkipped = false;
    b_TivaSWCorrect = true;
    b_DLPCFWCorrect = true;
    b_serNumSet = false;
    g_BluetoothTest = false;
    g_StartupCompleted = false;

    scene = new QGraphicsScene(0,0,515,450);
    initUI();
    USB_Init();

    emit check_USB_Connection();

    g_StartupCompleted = true;

#ifdef USB_POLL_ENABLED
	usbPollTimer = new QTimer(this);
	usbPollTimer->setInterval(2000);
	connect(usbPollTimer, SIGNAL(timeout()), this, SLOT(timerTimeout()));
	usbPollTimer->start();
#endif
}

MainWindow::~MainWindow()
{
	USB_Close();
	USB_Exit();

	save_status();
    delete ui;
}

void MainWindow::check_USB_Connection()
/**
 * This function checks for the USB Connection availability ans sets the Connected button status on the Mainwindow
 *
 */
{
    if(!USB_IsConnected())
    {
        if(USB_Open() == PASS)
            on_USB_Connection();
    }
    ui->ConnectToolButton->setEnabled(USB_IsConnected());
    if(USB_IsConnected())
        ui->ConnectToolButton->setText("Connected");
    else {
        ui->ConnectToolButton->setText("Not Connected");
        g_StartupCompleted = false;
    }
}

void MainWindow::get_num_files_in_SD_card()
/**
* This function will be called periodically on a timer and also on connection to a new EVM
*/
{
    int num;
    QString numStr;

    num = NNO_GetNumScanFilesInSD();
    numStr.setNum(num);

    if(num > 0)
    {
        ui->pushButton_import_scanData->setEnabled(true);
        ui->label_num_unread_scandata->setText(numStr);
    }
    else if (num < 0)
    {
        ui->pushButton_import_scanData->setEnabled(false);
        ui->label_num_unread_scandata->setText("SD card error");
    }
    else if (num == 0)
    {
        ui->label_num_unread_scandata->setText(numStr);
        ui->pushButton_import_scanData->setEnabled(false);
    }
}

void MainWindow::timerTimeout(void)
/*
 * This function does all the checking need to be done when the USB poll timer is timedout
 * If the USB connection is aviable it reads and updates the version numbers
 * If there is no USB connection, the corresponding GUI controls are disabled
 *
 */
{
    if(USB_IsConnected())
    {
        //do not do polling if we are doing any ther USB transactions
        if( QDateTime::currentDateTime().toTime_t() - USB_getLastTranstime() < 2)
            return;

        //if the connected bool is set, read from USB to make sure that the connection is still intact
        if(readVersionAndUpdate() == PASS)
        {
            get_num_files_in_SD_card();
            return;
        }
        else    //we must have lost the connection
        {
            USB_Close();
            check_USB_Connection();
        }
    }
    else
    {
        SetGUIControls(DISABLE);
        emit check_USB_Connection();
    }
}

static void find_meanstandard_deviation(float* data, int n , float* average , float* std)
{
    float mean=0.0, sum_deviation=0.0;
    int i;
    for(i=0; i<n;++i)
    {
        mean += data[i];
    }
    mean=mean/n;
    *average = mean;

    for(i=0; i<n;++i)
        sum_deviation+=(data[i]-mean)*(data[i]-mean);

    *std = sqrt(sum_deviation/n);
}

void MainWindow::initUI(void)
/*
 * This function initializes different tabs and other controls as per the respective env variables
 *
 */
{
	char versionStr[255];
	char * myenv = getenv("NANOENV");

    if(myenv == NULL)
    {
		num_tabs_in_gui = 3;
		factory_gui = false;
        //cont_scan_flag = false;
	}
	else if(strcmp(myenv,"FACTORY") != 0)
    {
		num_tabs_in_gui = 3;
		factory_gui = false;
        //cont_scan_flag = true;
	}
	else
    {
		num_tabs_in_gui = 5;
		factory_gui = true;
        //cont_scan_flag = true;
	}

	SetDetAlignSliderMaxVal(8388608/20);
    restore_status();

    sprintf(versionStr, "DLP NIRscan Nano GUI v%d.%d.%d", GUI_VERSION_MAJOR, GUI_VERSION_MINOR, GUI_VERSION_BUILD);
	setWindowTitle(versionStr);

    sprintf(versionStr, "%d.%d.%d", GUI_VERSION_MAJOR, GUI_VERSION_MINOR, GUI_VERSION_BUILD);
    ui->label_gui_ver_info->setText(versionStr);

    ui->graphicsView->setVisible(false);
	ui->label_yaxis->setVisible(false);
	ui->label_xaxis->setVisible(false);

	Populate_ScanList();

    //scan files are saved and saved scans are displayed from user documents Location
    if(filepath.GetsaveScanSettingsDirPath() == "")
    {
        filepath.SetsaveScanSettingsDirPath(filepath.GetuserPath());

    }
    if(filepath.GetdisplayScanSettingsDirPath() == "")
    {
       filepath.SetdisplayScanSettingsDirPath(filepath.GetuserPath());
    }



    ui->label_scan_directory->setText(filepath.GetdisplayScanSettingsDirPath());
    ui->label_scan_directory->setToolTip(filepath.GetdisplayScanSettingsDirPath());
    ui->pushButton_customSaveScanDir->setToolTip(filepath.GetsaveScanSettingsDirPath());
    ui->lineEdit_scan_save_dir->setText(filepath.GetsaveScanSettingsDirPath());

	ui->label_wait_plot->setVisible(false);

	if(factory_gui == true && b_TivaSWCorrect == true && b_DLPCFWCorrect == true)  // Set first page for factory
	{
		emit on_pushButton_FactoryTab_clicked();
	}

	if(factory_gui == true)  // Initial detector alignment and slit alignment for factory
	{
		initDetectorAlignment();
		initSlitAlignment();
	}
    m_ref_selection = SCAN_REF_FACTORY;
    ui->radioButton_ref_factory->setChecked(true);

    ui->pushButton_Intensity->setChecked(true);
    ui->pushButton_Absorbance->setChecked(false);
    ui->pushButton_Reflectance->setChecked(false);

    // Only alphanumeric characters
    QRegExp rx("[A-Za-z0-9_-]+");
    QValidator *validator = new QRegExpValidator(rx, this);

    ui->lineEdit_Prefix->setValidator(validator);
    ui->lineEdit_FileName->setValidator(validator);

    #ifndef PATTERN_WIDTH_CONTROL
        ui->spinBox_fwhmWidth->setVisible(false);
        ui->label_ptnwidth->setVisible(false);
    #endif


     ui->lineEdit_FileName->setEnabled(false);

    InitStatusBar();


}

void MainWindow::save_status(void)
/*
 * This function saves the current context of the GUI before closing
 *
 */
{
    QString configFile = filepath.GetconfigDir().absoluteFilePath("scan_cfgs.dat");

    uiSettings.setValue("DLPCFirmwarePath", filepath.GetdlpcFirmwarePath());
    uiSettings.setValue("BMP24Path", filepath.Getbmp24Path());
    uiSettings.setValue("activeTab", ui->main_tabs->currentIndex());
    uiSettings.setValue("TivaFirmwarePath", filepath.GettivaFirmwarePath());
    uiSettings.setValue("sidePanelActiveTab", ui->tabWidget_2->currentIndex());
    uiSettings.setValue("DisplaySettingDir",filepath.GetdisplayScanSettingsDirPath());
    uiSettings.setValue("SaveScanSettingsDir",filepath.GetsaveScanSettingsDirPath());

    uiSettings.setValue("savecombined_csv",spectrum.saveone_csv);
    uiSettings.setValue("savesep_csv",spectrum.saveseperate_csv);
    uiSettings.setValue("absorbance_csv",spectrum.absorbance_csv);
    uiSettings.setValue("intensity_csv",spectrum.intensity_csv);
    uiSettings.setValue("reflectance_csv",spectrum.reflectance_csv);
    uiSettings.setValue("save_JCAMP",spectrum.save_JCAMP);
    uiSettings.setValue("absorbance_JCAMP",spectrum.absorbance_JCAMP);
    uiSettings.setValue("intensity_JCAMP",spectrum.intensity_JCAMP);
    uiSettings.setValue("reflectance_JCAMP",spectrum.reflectance_JCAMP);
    uiSettings.setValue("savebinary",spectrum.savebinary);
    scancfglist.ExportLocalList(configFile);


}

void MainWindow::restore_status(void)
/*
 * This function restores    the last saved context of the GUI
 *
 */
{
    int active_tab_index;
    QString configFile = filepath.GetconfigDir().absoluteFilePath("scan_cfgs.dat");

    spectrum.saveone_csv=uiSettings.value("savecombined_csv",true).toBool();
    spectrum.saveseperate_csv=uiSettings.value("savesep_csv",false).toBool();
    spectrum.absorbance_csv=uiSettings.value("absorbance_csv",true).toBool();
    spectrum.intensity_csv=uiSettings.value("intensity_csv", true).toBool();
    spectrum.reflectance_csv=uiSettings.value("reflectance_csv", true).toBool();
    spectrum.save_JCAMP     =uiSettings.value("save_JCAMP", false).toBool();
    spectrum.absorbance_JCAMP=uiSettings.value("absorbance_JCAMP", true).toBool();
    spectrum.intensity_JCAMP   =uiSettings.value("intensity_JCAMP", true).toBool();
    spectrum.reflectance_JCAMP  =uiSettings.value("reflectance_JCAMP", true).toBool();
    spectrum.savebinary = uiSettings.value("savebinary", true).toBool();
    saveone_csv=uiSettings.value("savecombined_csv", true).toBool();
    saveseperate_csv=uiSettings.value("savesep_csv", false).toBool();
    absorbance_csv=uiSettings.value("absorbance_csv", true).toBool();
    intensity_csv=uiSettings.value("intensity_csv", true).toBool();
    reflectance_csv=uiSettings.value("reflectance_csv", true).toBool();
    save_JCAMP     =uiSettings.value("save_JCAMP", false).toBool();
    absorbance_JCAMP=uiSettings.value("absorbance_JCAMP", true).toBool();
    intensity_JCAMP   =uiSettings.value("intensity_JCAMP", true).toBool();
    reflectance_JCAMP  =uiSettings.value("reflectance_JCAMP", true).toBool();
    savebinary = uiSettings.value("savebinary", true).toBool();

    filepath.SetdlpcFirmwarePath(uiSettings.value("DLPCFirmwarePath", "").toString());
    filepath.Setbmp24Path(uiSettings.value("BMP24Path", "").toString());
    active_tab_index = uiSettings.value("activeTab").toInt();
    filepath.SettivaFirmwarePath(uiSettings.value("tivaFirmwarePath", "").toString());
    if(active_tab_index >= num_tabs_in_gui)
        active_tab_index = 0;
    filepath.SetdisplayScanSettingsDirPath(uiSettings.value("DisplaySettingDir", "").toString());
    filepath.SetsaveScanSettingsDirPath(uiSettings.value("SaveScanSettingsDir", "").toString());

    ui->main_tabs->setCurrentIndex(active_tab_index);
    ui->tabWidget_2->setCurrentIndex(uiSettings.value("sidePanelActiveTab").toInt());

	if(!factory_gui)
    {
		ui->main_tabsPage_Factory->hide();
		ui->pushButton_FactoryTab->hide();
		ui->main_tabsPage_Test->hide();
        ui->pushButton_TestTab->hide();
        NNO_SetFixedPGAGain(0, 64);
	}
    switch(active_tab_index)
	{
		case 0:
			ui->pushButton_InformationTab->setChecked(true);
			break;
		case 1:
			ui->pushButton_ScanTab->setChecked(true);
			break;
		case 2:
			ui->pushButton_UtilsTab->setChecked(true);
			break;
		case 3:
			ui->pushButton_FactoryTab->setChecked(true);
			break;
		case 4:
			ui->pushButton_TestTab->setChecked(true);
			break;
		default:
			ui->pushButton_InformationTab->setChecked(true);
	}

    scancfglist.ImportLocalList(configFile);
}

void MainWindow::on_USB_Connection()
/*
 * This function does all initializations to be done when we connect to the target
 * Enables the various GUI controls which work only when there is a target connected
 */
{
    int retval;

	if ( !g_StartupCompleted )
   		NNO_TestBT(false);

    checkversion = true;
    if(readVersionAndUpdate() != PASS)
        return;

    emit on_pushButton_SyncDateTime_clicked();

    if(scancfglist.ImportTargetList() != PASS)
    {
        showError("Error reading scan configs from device");
    }
    else
    {
        PopulateScanCfgListComboBox();
    }
    SetGUIControls(ENABLE);

    get_num_files_in_SD_card();
    evm.FetchRefCalData();
    evm.FetchRefCalMatrix();

    //Get hibernation status so that GUI is consistent with runtime Nano setting
    retval = NNO_GetHibernate();
    if(retval < 0)
    {
        ui->checkBox_hibernateEnabled->setEnabled(false);
        showError("Error reading hibernation state from device");
    }
    else
    {
        ui->checkBox_hibernateEnabled->setEnabled(true);
        ui->checkBox_hibernateEnabled->setChecked(retval);
    }
}

void MainWindow::showError(const char *str)
/*
 * This function if for showing the GUI error messages
 * @param str - I -  error string to be shown to the user
 *
 */
{
	QString title("Error Message");
    QString text(str);
    QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton, this);
    msgBox.exec();

}

int MainWindow::readVersionAndUpdate(void)
/*
 * This function reads and displays the GUI , TIVA FW and DLPC Flash versions on the Information Tab
 * @return  0 = PASS
 *         -1 = FAIL
 *
 */
{
    char versionStr[255];
    unsigned int tiva_sw_ver;
    unsigned int dlpc_sw_ver;
    unsigned int dlpc_fw_ver;
    unsigned int speclib_ver;
    unsigned int eeprom_cfg_ver;
    unsigned int eeprom_cal_ver;
    unsigned int eeprom_refcal_ver;
    unsigned int tiva_sw_ver_major;
    unsigned int tiva_sw_ver_minor;
    unsigned int tiva_sw_ver_build;
    unsigned int speclib_ver_major;
    unsigned int speclib_ver_minor;
    unsigned int speclib_ver_build;
    unsigned int dlpc_fw_ver_major;
    unsigned int dlpc_fw_ver_minor;
    unsigned int dlpc_fw_ver_build;
    int ret_val;
    static uint32 last_status;
    bool showVerWarning = false;
    bool showEEPROMVersion = false;


    if(USB_IsConnected())
    {
        uint32 device_status;

        if((ret_val = NNO_ReadDeviceStatus(&device_status)) == PASS)
        {
            last_status = device_status;
            UpdateDeviceStatus(device_status);

            NNO_error_status_struct error_status;

            if(NNO_ReadErrorStatus(&error_status) == PASS)
            {
                ui->pushButton_errorStatus->setText("Error Status\nPress to clear");
                ui->pushButton_errorStatus->setEnabled(true);
                UpdateErrorStatus(&error_status);
            }
            else
            {
                ui->pushButton_errorStatus->setText("Unable to read\n Error Status");
                ui->pushButton_errorStatus->setEnabled(false);
            }
        }
        else if (ret_val == NNO_CMD_BUSY) //If nano is servicing a higher priority command interface
        {
            /* Make sure NNO_GetVersion also returns NNO_CMD_BUSY because some older Tiva FW do not
             * support NNO_ReadDeviceStatus() command and their response to that command cannot be trusted
             */
            if(NNO_GetVersion(&tiva_sw_ver, &dlpc_sw_ver, &dlpc_fw_ver, &speclib_ver, &eeprom_cal_ver, &eeprom_refcal_ver, &eeprom_cfg_ver) == NNO_CMD_BUSY)
            {
                //Set Tiva Active Status as false and BT connected as true
                UpdateDeviceStatus(last_status | NNO_STATUS_ACTIVE_BLE_CONNECTION);

                if ( !g_BluetoothTest )
                {
                    BlueToothDialog* myDialog = new BlueToothDialog(this);
                    //To remove the close button on top right from this window
                    myDialog->setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint);
                    myDialog->exec();
                    delete myDialog;
                    return FAIL;
                }
            }
        }
        else
        {
            //Set Tiva Active Status as false
            UpdateDeviceStatus(0);
        }


        if(NNO_GetVersion(&tiva_sw_ver, &dlpc_sw_ver, &dlpc_fw_ver, &speclib_ver, &eeprom_cal_ver, &eeprom_refcal_ver, &eeprom_cfg_ver) == 0)
        {
            tiva_sw_ver &= 0xFFFFFF;
            tiva_sw_ver_major = tiva_sw_ver >> 16;
            tiva_sw_ver_minor = (tiva_sw_ver << 16) >> 24;
            tiva_sw_ver_build = (tiva_sw_ver << 24) >> 24;

            sprintf(versionStr, "%d.%d.%d", tiva_sw_ver_major, tiva_sw_ver_minor, tiva_sw_ver_build);

            ui->label_tiva_sw_ver_info->setText(versionStr);

            versiondialog* myDialog = new versiondialog(this);
            myDialog->setTivaVersion(QString(versionStr));

            if(tiva_sw_ver_major != GUI_VERSION_MAJOR || tiva_sw_ver_minor != GUI_VERSION_MINOR)
            {

                if(checkversion == true)
                {
                    showVerWarning = true;
                    myDialog->setTivaVersionRed();
                    emit on_pushButton_UtilsTab_clicked();
                }

                b_TivaSWCorrect = false;
            }
            else
                b_TivaSWCorrect = true;
            dlpc_fw_ver_major = dlpc_fw_ver >> 24;
            dlpc_fw_ver_minor = (dlpc_fw_ver << 8) >> 24;
            dlpc_fw_ver_build = (dlpc_fw_ver << 16) >> 16;
            sprintf(versionStr, "%d.%d.%d", dlpc_fw_ver_major, dlpc_fw_ver_minor, dlpc_fw_ver_build);

            myDialog->setDlpcVersion(QString(versionStr));
            ui->label_dlpc_fw_ver_info->setText(versionStr);


            if(dlpc_fw_ver_major < DLPC_VERSION_MAJOR || (int)dlpc_fw_ver_minor < DLPC_VERSION_MINOR)
            {
                if(checkversion == true)
                {
                    showVerWarning = true;
                    myDialog->setDLPCVersionRed();
                    emit on_pushButton_UtilsTab_clicked();
                }
                b_DLPCFWCorrect = false;
            }
            else
            {
                b_DLPCFWCorrect = true;
            }

            sprintf(versionStr, "%d", eeprom_cal_ver);
            ui->label_cal_coeff_ver->setText(versionStr);
            myDialog->setEEPROMcalVersion(QString(versionStr));

            sprintf(versionStr, "%d", eeprom_refcal_ver);
            ui->label_ref_cal_ver->setText(versionStr);
            myDialog->setRefcalVersion(QString(versionStr));

            sprintf(versionStr, "%d", eeprom_cfg_ver);
            ui->label_scan_config_ver->setText(versionStr);
            myDialog->setEEPROMcfgVersion(QString(versionStr));

            speclib_ver &= 0xFFFFFF;
            speclib_ver_major = speclib_ver >> 16;
            speclib_ver_minor = (speclib_ver << 16) >> 24;
            speclib_ver_build = (speclib_ver << 24) >> 24;

            sprintf(versionStr, "%d.%d.%d", speclib_ver_major, speclib_ver_minor, speclib_ver_build);
            myDialog->setSpeclibVersion(QString(versionStr));
            if(checkversion == true && b_VersionWindowSkipped == false)//skip version check during selftesting
            {

                if ((speclib_ver_major != DLPSPEC_VERSION_MAJOR) || (speclib_ver_minor != DLPSPEC_VERSION_MINOR))
                {
                    showVerWarning = true;
                    myDialog->setSpeclibVersionRed();
                }
                if(eeprom_cal_ver != DLPSPEC_CALIB_VER)
                {
                    showVerWarning = true;
                    showEEPROMVersion = true;
                    myDialog->setEEPROMcalVersionRed();
                }

                if(eeprom_refcal_ver != DLPSPEC_REFCAL_VER)
                {
                    showVerWarning = true;
                    showEEPROMVersion = true;
                    myDialog->setRefcalVersionRed();
                    emit on_pushButton_UtilsTab_clicked();
                }

                if(NNO_GetNumScanCfg() == 0)
                {
                    showError("No scan configs stored in the device");
                }
                else if(eeprom_cfg_ver != DLPSPEC_CFG_VER)
                {
                    showVerWarning = true;
                    showEEPROMVersion = true;
                    myDialog->setEEPROMcfgVersionRed();
                }
            }


            if(showVerWarning)
            {
                myDialog->showEEPROMVersions(showEEPROMVersion);
                myDialog->exec();
                delete myDialog;
            }
                checkversion = false;
                return PASS;
            }
            else
            {
                return FAIL;    //Read failed
            }
        }
        else //if USB is not connected
        {
            //Set Tiva Active Status as false
            UpdateDeviceStatus(0);
            ui->ConnectToolButton->setEnabled(false);
            ui->ConnectToolButton->setText("Not Connected");
            g_StartupCompleted = false;
            
        }
        QApplication::processEvents(); //update GUI
        return FAIL; //USB is not connected.
    }

int SNRHadStrucMain[HADSNR_NUM_DATA][HADSNR_LENGTH];
int MainWindow::DoSNRComputation( int iteration )
{
    int numpat;
    int scanStatus;
    QString dispStr;
    int gain_setting;
    QString fileName, fileNameRel;
    uScanConfig config;
    int fileSize;
    scanResults hadResults;
    scanData* pData = NULL;
    uint8_t dataBlob[SCAN_DATA_BLOB_SIZE];	//Scan data size is used since that gives us the biggest blob data size
    int count = 0;
    int hadSnrResult = 0;
    int i,j;
    float snrDiff[HADSNR_NUM_DATA];
    float snrtemp[HADSNR_NUM_DATA];
    float avg=0.0  , std = 0.0 , avgdummy= 0.0 , stddummy = 0.0;
    int ret = PASS;

    QString cur = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");

    if ( !iteration )
    {
        dispStr.sprintf("%d", iteration);
        fileNameRel = QString("SNR_%1_%2.csv").arg(dispStr,cur);
    }
    else
        fileNameRel = QString("SNR_%2.csv").arg(cur);
    fileName = filepath.GetsaveScanSettingsDir().absoluteFilePath(fileNameRel);

    QFile SNRResultsFile(fileName);
    SNRResultsFile.open(QIODevice::ReadWrite | QIODevice::Text);
    QTextStream out(&SNRResultsFile);

    gain_setting = NNO_GetPGAGain();
    SetGUIControls(DISABLE);
    do
    {
        //Set PGA gain to 64
        NNO_SetPGAGain(64);
        /* add the command to start SNR scan */
        ui->progressbar_cal->setValue(0);
        ui->label_scan_progress->setText("Start SNR Data Capture");
        QApplication::processEvents();
        NNO_StartSNRScan();

        config.scanCfg.scan_type = COLUMN_TYPE;
        config.scanCfg.num_patterns = 9;
        config.scanCfg.num_repeats = 720;
        config.scanCfg.wavelength_start_nm = MIN_WAVELENGTH;
        config.scanCfg.wavelength_end_nm = MAX_WAVELENGTH;
#ifdef PATTERN_WIDTH_CONTROL
        config.scanCfg.width_px = ui->spinBox_fwhmWidth->value();
#else
        config.scanCfg.width_px = 7;
#endif
        numpat = evm.ApplyScanCfgtoDevice(&config);
        if(numpat != 9)
        {
            showError("Apply scan config failed");
            ret = FAIL;
            break;
        }

        ui->progressbar_cal->setValue(10);
        ui->label_scan_progress->setText("Scan in progress");
        QApplication::processEvents();

        scanStatus = PerformScanReadData(NNO_DONT_STORE_SCAN_IN_SD, 720, dataBlob, &fileSize);

        if(scanStatus != PASS)
        {
            showError("scan failed");
            ret = FAIL;
            break;
        }

        //Set PGA gain to 64
        NNO_SetPGAGain(64);
        /* add the command to start SNR scan */
        ui->progressbar_cal->setValue(40);
        ui->label_scan_progress->setText("Start Hadamard SNR Data Capture");
        QApplication::processEvents();

        // Execute scan with one repeat to captures scanData struct
        config.scanCfg.scan_type = HADAMARD_TYPE;
        config.scanCfg.num_patterns = HADSNR_NUM_PATTERNS;
        config.scanCfg.num_repeats = 1;
        config.scanCfg.wavelength_start_nm = MIN_WAVELENGTH;
        config.scanCfg.wavelength_end_nm = MAX_WAVELENGTH;
#ifdef PATTERN_WIDTH_CONTROL
        config.scanCfg.width_px = ui->spinBox_fwhmWidth->value();
#else
        config.scanCfg.width_px = 7;
#endif
        numpat = evm.ApplyScanCfgtoDevice(&config);
        if(numpat > HADSNR_LENGTH)
            return FAIL;

        pData = (scanData *)malloc(SCAN_DATA_BLOB_SIZE);
        if(pData == NULL)
        {
            showError("Out of memory");
            ret = FAIL;
            break;
        }

        scanStatus = PerformScanReadData(NNO_DONT_STORE_SCAN_IN_SD, 1, pData, &fileSize);
        if(scanStatus != PASS)
        {
            showError("scan failed");
            ret = FAIL;
            break;
        }
        else
        {
            // deserialize scanData blob so we can iteratively replace with SNR data later
            if(dlpspec_deserialize(pData, SCAN_DATA_BLOB_SIZE, SCAN_DATA_TYPE) != DLPSPEC_PASS)
            {
                showError("dlpspec_deserialize failed");
                ret = FAIL;
                break;
            }
        }

        NNO_StartHadSNRScan();

        // Now execute the scan with greater repeats to capture SNR data
        config.scanCfg.num_repeats = HADSNR_NUM_REPEATS;
        numpat = evm.ApplyScanCfgtoDevice(&config);
        if(numpat > HADSNR_LENGTH)
        {
            showError("Apply scan config failed");
            ret = FAIL;
            break;
        }

        ui->progressbar_cal->setValue(70);
        ui->label_scan_progress->setText("Hadamard Scan in progress");
        QApplication::processEvents();

        scanStatus = PerformScanReadData(NNO_DONT_STORE_SCAN_IN_SD, HADSNR_NUM_REPEATS, dataBlob, &fileSize);

        if(scanStatus != PASS)
        {
            showError("scan failed");
            ret = FAIL;
            break;
        }

        fileSize = NNO_GetFileSizeToRead(NNO_FILE_HADSNR_DATA);
        if(NNO_GetFile((unsigned char *)SNRHadStrucMain, fileSize) != fileSize)
        {
            showError("Get Hadamard SNR data failed");
            ret = FAIL;
            break;
        }

        /* add the command to compute SNR data */
        // Compute spectrum
        for (i=0; i<HADSNR_NUM_DATA; i++)
        {
            for (j=0; j<HADSNR_LENGTH; j++)
                pData->adc_data[j] = SNRHadStrucMain[i][j];

            dlpspec_scan_write_data((uScanData *)pData, dataBlob, SCAN_DATA_BLOB_SIZE);
            if(dlpspec_scan_interpret(&dataBlob, SCAN_DATA_BLOB_SIZE, &hadResults) != DLPSPEC_PASS)
            {
                showError("Scan interpret failed");
                ret = FAIL;
                break;
            }

            for (j=0; j<SNR_HAD_PATTERNS; j++)
                SNRHadStrucMain[i][j] = hadResults.intensity[j];
        }

        //j = HADSNR_LENGTH/2;
        int meanHadSNR = 0;
        for (j=0; j<HADSNR_NUM_PATTERNS; j++)
        {
            for(i=0 ; i<HADSNR_NUM_DATA ; i++)
            {
                snrtemp[i] = SNRHadStrucMain[i][j];
                if(i>0 && i<HADSNR_NUM_DATA)
                {
                    count++;
                    snrDiff[i-1] = snrtemp[i]-snrtemp[i-1];
                }
            }
            find_meanstandard_deviation(snrtemp, HADSNR_NUM_DATA, &avg , &stddummy);
            find_meanstandard_deviation(snrDiff, count  , &avgdummy , &std);

            count = 0;

            if (hadSnrResult < (float)avg/std)
                hadSnrResult = (float)avg/std;
            meanHadSNR += (float)avg/std;
        }

        meanHadSNR /= HADSNR_NUM_PATTERNS;

        int snr17 = -1 , snr100 = -1, snr500 = -1;
        NNO_GetSNRData(&snr17 , &snr100 , &snr500);
        QString labelText;
        labelText.sprintf("SNR 17 is  %d , SNR 133 is %d , SNR 600 is %d\nHad SNR 1s: Peak=%d  Mean=%d",snr17,snr100,snr500,hadSnrResult,meanHadSNR);
        ui->label_cal->setText(labelText);

        out << "SNR Results\n";
        out << "SNR  17:," << snr17 << "\n";
        out << "SNR 133:," << snr100 << "\n";
        out << "SNR 600:," << snr500 << "\n";
        out << "Hadamard SNR 1s:," << hadSnrResult << "\n";
        SNRResultsFile.close();
    }while(0);

    if(pData != NULL)
        free(pData);
    NNO_SetPGAGain(gain_setting);
    ui->progressbar_cal->setValue(100);
    SetGUIControls(ENABLE);

    return ret;
}



void MainWindow::SetGUIControls(bool enable_disable)
/*
 *This function disables all the GUI controls when the USB is diconnected
 */
{
    //USB controls on Scan tab
    ui->groupBox_scan_controls->setEnabled(enable_disable);
    ui->groupBox_file_functions->setEnabled(enable_disable);
    ui->checkBox_always_on->setEnabled(enable_disable);

   //controls on Utils tab
    ui->groupBox_sensors->setEnabled(enable_disable);
    ui->groupBox_datetime->setEnabled(enable_disable);
    ui->groupBox_battery_charger->setEnabled(enable_disable);
    ui->groupBox_dlpc->setEnabled(enable_disable);
    ui->groupBox_resetevm->setEnabled(enable_disable);
    ui->groupBox_hibernate->setEnabled(enable_disable);
    ui->groupBox_RefCalUpdate->setEnabled(enable_disable);

    //controls on factory tab
    ui->groupBox_det_align->setEnabled(enable_disable);
    ui->groupBox_slit_align->setEnabled(enable_disable);
    ui->groupBox_wave_cal->setEnabled(enable_disable);
    ui->groupBox_Serial_Number->setEnabled(enable_disable);
    ui->groupBox_eeprom->setEnabled(enable_disable);
    ui->groupBox_dlpc_regset->setEnabled(enable_disable);

    //tests tab
    ui->groupBox_evm_selftests->setEnabled(enable_disable);

    if(enable_disable == true)
        ui->pushButton_scan->setText("Scan");
    else
        ui->pushButton_scan->setText("Wait");
    ui->pushButton_scan->setEnabled(enable_disable);
    ui->pushButton_scan->repaint();
}

//handler functions for the main Tabs and the controls on Information Tab

void MainWindow::on_pushButton_ScanTab_clicked()
{
    ui->main_tabs->setCurrentIndex(1);
    ui->pushButton_ScanTab->setChecked(true);
    ui->pushButton_UtilsTab->setChecked(false);
    ui->pushButton_FactoryTab->setChecked(false);
    ui->pushButton_TestTab->setChecked(false);
    ui->pushButton_InformationTab->setChecked(false);
    Populate_ScanList();    
    //re-apply in case another scan cfg was applied while in factory tab
     emit on_comboBox_scanID_currentIndexChanged(ui->comboBox_scanID->currentIndex());
}

void MainWindow::on_pushButton_UtilsTab_clicked()
{
    ui->main_tabs->setCurrentIndex(2);
    ui->pushButton_UtilsTab->setChecked(true);
    ui->pushButton_ScanTab->setChecked(false);
    ui->pushButton_FactoryTab->setChecked(false);
    ui->pushButton_TestTab->setChecked(false);
    ui->pushButton_InformationTab->setChecked(false);

    b_VersionWindowSkipped = false;
}

void MainWindow::on_pushButton_FactoryTab_clicked()
{
    if(factory_gui == false)
        return;

    ui->main_tabs->setCurrentIndex(3);
    ui->pushButton_FactoryTab->setChecked(true);
    ui->pushButton_ScanTab->setChecked(false);
    ui->pushButton_UtilsTab->setChecked(false);
    ui->pushButton_TestTab->setChecked(false);
    ui->pushButton_InformationTab->setChecked(false);
    on_comboBox_calID_currentIndexChanged(0);

    b_VersionWindowSkipped = false;
}

void MainWindow::on_pushButton_TestTab_clicked()
{
    if(factory_gui == false)
        return;

    ui->main_tabs->setCurrentIndex(4);
    ui->pushButton_TestTab->setChecked(true);
    ui->pushButton_FactoryTab->setChecked(false);
    ui->pushButton_ScanTab->setChecked(false);
    ui->pushButton_UtilsTab->setChecked(false);
    ui->pushButton_InformationTab->setChecked(false);

    b_VersionWindowSkipped = true;
}

void MainWindow::on_pushButton_InformationTab_clicked()
{
    ui->main_tabs->setCurrentIndex(0);
    ui->pushButton_InformationTab->setChecked(true);
    ui->pushButton_ScanTab->setChecked(false);
    ui->pushButton_UtilsTab->setChecked(false);
    ui->pushButton_FactoryTab->setChecked(false);
    ui->pushButton_TestTab->setChecked(false);
}

void MainWindow::on_ti_e2e_pushButton_clicked()
{
    QDesktopServices::openUrl(QUrl("http://e2e.ti.com/support/dlp__mems_micro-electro-mechanical_systems/default.aspx", QUrl::TolerantMode));
}

void MainWindow::InitStatusBar()
/*
 * This function initializes the Device Status and the Error status tables
 * These tables are out of the TabWidget and are visible from every tab
 * Sets the tables' columns with repsective device/error status text
 * Initializes the icons to grey to start with
 *
 */
{

    //intializes the device status table
    ui->statusWidget->setRowCount(1);
    ui->statusWidget->setColumnCount(6);
    ui->statusWidget->horizontalHeader()->hide();
    ui->statusWidget->verticalHeader()->hide();
    ui->statusWidget->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->statusWidget->horizontalHeader()->stretchLastSection();
    ui->statusWidget->verticalHeader()->stretchLastSection();
    ui->statusWidget->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);


    //initializes the error status table
    ui->errorWidget->setRowCount(1);
    ui->errorWidget->setColumnCount(11);
    ui->errorWidget->horizontalHeader()->hide();
    ui->errorWidget->verticalHeader()->hide();
    ui->errorWidget->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->errorWidget->horizontalHeader()->stretchLastSection();
    ui->errorWidget->verticalHeader()->stretchLastSection();


    ui->errorWidget->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);


    QString filename = ":/new/Icons/Led_Gray.png";
    QPixmap pixmap(filename);
    QIcon buton(pixmap);

    QPushButton *schk = new QPushButton();
    schk->setFlat(true);
    schk->setText("TIVA");
    schk->setIcon(buton);
    schk->setCheckable(false);

    ui->statusWidget->setCellWidget(0,0,schk);

    QPushButton *schk1 = new QPushButton();
    schk1->setFlat(true);
    schk1->setText("Scan");
    schk1->setIcon(buton);
    schk1->setCheckable(false);
    ui->statusWidget->setCellWidget(0,1,schk1);


    QPushButton *schk2 = new QPushButton();
    schk2->setFlat(true);
    schk2->setText("SD Card");
    schk2->setIcon(buton);
    schk2->setCheckable(false);
    ui->statusWidget->setCellWidget(0,2,schk2);

    QPushButton *schk3 = new QPushButton();
    schk3->setFlat(true);
    schk3->setText("Card I/O");
    schk3->setIcon(buton);
    schk3->setCheckable(false);
    ui->statusWidget->setCellWidget(0,3,schk3);

    QPushButton *schk4 = new QPushButton();
    schk4->setFlat(true);
    schk4->setText("BT Ready");
    schk4->setIcon(buton);
    schk4->setCheckable(false);
    ui->statusWidget->setCellWidget(0,4,schk4);

    QPushButton *schk5 = new QPushButton();
    schk5->setFlat(true);
    schk5->setText("BT Connected");
    schk5->setIcon(buton);
    schk5->setCheckable(false);
    ui->statusWidget->setCellWidget(0,5,schk5);


    ErrorPushButton *chk = new ErrorPushButton();
    chk->setFlat(true);
    chk->setIcon(buton);
    chk->setText("Scan");
    ui->errorWidget->setCellWidget(0,0,chk);

    ErrorPushButton *chk1 = new ErrorPushButton();
    chk1->setFlat(true);
    chk1->setIcon(buton);
    chk1->setText("ADC");
    ui->errorWidget->setCellWidget(0,1,chk1);

    ErrorPushButton *chk2 = new ErrorPushButton();
    chk2->setFlat(true);
    chk2->setIcon(buton);
    chk2->setText("SD Card");
    ui->errorWidget->setCellWidget(0,2,chk2);


    ErrorPushButton *chk3 = new ErrorPushButton();
    chk3->setFlat(true);
    chk3->setIcon(buton);
    chk3->setText("EEPROM");
    ui->errorWidget->setCellWidget(0,3,chk3);

    ErrorPushButton *chk4 = new ErrorPushButton();
    chk4->setFlat(true);
    chk4->setIcon(buton);
    chk4->setText("BLE");
    ui->errorWidget->setCellWidget(0,4,chk4);


    ErrorPushButton *chk5 = new ErrorPushButton();
    chk5->setFlat(true);
    chk5->setIcon(buton);
    chk5->setText("Spectrum");
    ui->errorWidget->setCellWidget(0,5,chk5);

    ErrorPushButton *chk6 = new ErrorPushButton();
    chk6->setFlat(true);
    chk6->setIcon(buton);
    chk6->setText("Hardware");
    ui->errorWidget->setCellWidget(0,6,chk6);

    ErrorPushButton *chk7= new ErrorPushButton();
    chk7->setFlat(true);
    chk7->setIcon(buton);
    chk7->setText("Temperature\n Sensor");
    ui->errorWidget->setCellWidget(0,7,chk7);

    ErrorPushButton *chk8 = new ErrorPushButton();
    chk8->setFlat(true);
    chk8->setIcon(buton);
    chk8->setText("Humidity\n Sensor");
    ui->errorWidget->setCellWidget(0,8,chk8);

    ErrorPushButton *chk9 = new ErrorPushButton();
    chk9->setFlat(true);
    chk9->setIcon(buton);
    chk9->setText("Battery");
    ui->errorWidget->setCellWidget(0,9,chk9);

    ErrorPushButton *chk10 = new ErrorPushButton();
    chk10->setFlat(true);
    chk10->setIcon(buton);
    chk10->setText("Memory");
    ui->errorWidget->setCellWidget(0,10,chk10);

    ui->statusWidget->setShowGrid(false);
    ui->errorWidget->setShowGrid(false);
}

void MainWindow::UpdateDeviceStatus(int deviceStatus)
/*
 * This function updates the Device Status table with the current Device status bits
 * If a particular status bit is on, in the correpsonding column in the table
 * the grey icon is now changed to the Yellow icon
 * For the Blue tooth status bits when on, the grey icon is changed to Blue icon
 * @param deviceStatus - I - the dveice status got from evm
 *
 */
{
    int i = 1;
    QString filename_Y = ":/new/Icons/Led_Y.png";
    QPixmap pixmapY(filename_Y);
    QIcon yellow_icon(pixmapY);

    QString filename_Gray = ":/new/Icons/Led_Gray.png";
    QPixmap pixmapGray(filename_Gray);
    QIcon gray_icon(pixmapGray);

    QString filename_B = ":/new/Icons/Led_B.png";
    QPixmap pixmapB(filename_B);
    QIcon blue_icon(pixmapB);

    for(int index = 0 ; index < 6 ; index++)
    {
        QPushButton* chk = (QPushButton *)ui->statusWidget->cellWidget(0,index);
        if(chk)
        {
            if((deviceStatus & i) != 0)
            {
                if(index < 4)
                    chk->setIcon(yellow_icon);
                else
                    chk->setIcon(blue_icon); //set blue for Bluetooth status
             }
            else
                chk->setIcon(gray_icon);
        }
        i = i << 1;
    }
}

void MainWindow::UpdateErrorStatus(NNO_error_status_struct *error)
/*
 * This function updates the Error Status table with the current error status bits
 * If a particular error bit is on, the corresonding column in the table
 * the grey icon is now changed to the Red icon
 * The message in the repective ErrorPush button is set to the corresponding error code
 * Rest of the buttons remain flat and  uncheckable
 * @param error - I - the error status structure got from EVM
 *
 */
{
    uint32 errorType = 0;
    short errorCode = 0;
    int index = 0;
    QString filename_R = ":/new/Icons/Led_R.png";
    QPixmap pixmapR(filename_R);
    QIcon red_icon(pixmapR);

    QString filename_Gray = ":/new/Icons/Led_Gray.png";
    QPixmap pixmapGray(filename_Gray);
    QIcon gray_icon(pixmapGray);

    ErrorPushButton* chk = NULL;
    for(uint16 i = 0 ; i <= NNO_ERROR_MAX; i++)
    {
        errorType = error->status & (1 << i);
        chk   = (ErrorPushButton *)ui->errorWidget->cellWidget(0,index);
        if(chk)
        {
            chk->setErrorMessage("");
            if (errorType == 0)  //error bit not set
            {
                chk->setIcon(gray_icon);
                index++;
                chk->setFlat(true);
                QApplication::processEvents();
                continue;

            }
            else
            {
                chk->setIcon(red_icon);
                chk->setFlat(false);
            }
        }
        index ++;
        if(!chk)
            return;

        // Get error code, if applicable
        if (i < NNO_error_code_max )
        {
            switch(i)
            {
            case NNO_error_code_scan:

                errorCode = error->errorCodes.scan;

                if(errorCode == NNO_ERROR_SCAN_DLPC150_BOOT_ERROR)
                   chk->setErrorMessage("Scan Error: DLPC150 Boot error detected.");
                else if(errorCode == NNO_ERROR_SCAN_DLPC150_INIT_ERROR)
                    chk->setErrorMessage("Scan Error: DLPC150 Init error detected.");
                else if(errorCode == NNO_ERROR_SCAN_DLPC150_LAMP_DRIVER_ERROR)
                    chk->setErrorMessage("Scan Error: DLPC150 Lamp Driver error detected.");
                else if (errorCode == NNO_ERROR_SCAN_DLPC150_CROP_IMG_FAILED)
                    chk->setErrorMessage("Scan Error: DLPC150 Crop Image Failed.");
                else if (errorCode == NNO_ERROR_SCAN_ADC_DATA_ERROR)
                    chk->setErrorMessage("Scan Error: ADC Data error.");
                else if(errorCode ==  NNO_ERROR_SCAN_CFG_INVALID	)
                    chk->setErrorMessage("Scan Error: Scan Config Invalid.");
                else if(errorCode == NNO_ERROR_SCAN_PATTERN_STREAMING)
                    chk->setErrorMessage("Scan Error: Scan Pattern Streaming Error");

                break;
            case NNO_error_code_adc:
                errorCode = error->errorCodes.adc;

                if(errorCode == NNO_ERROR_ADC_START)
                    chk->setErrorMessage("ADC Error: ADC Start Error.");
                else if(errorCode == ADC_ERROR_TIMEOUT)
                    chk->setErrorMessage("ADC Error: ADC Timeout Error.");
                else if(errorCode == ADC_ERROR_POWERDOWN)
                    chk->setErrorMessage("ADC Error: ADC PowerDown error.");
                else if (errorCode == ADC_ERROR_POWERUP)
                    chk->setErrorMessage("ADC Error: ADC PowerUp error.");
                else if (errorCode == ADC_ERROR_STANDBY)
                    chk->setErrorMessage("ADC Error: ADC StandBy error.");
                else if(errorCode ==  ADC_ERROR_WAKEUP)
                    chk->setErrorMessage("ADC Error: ADC WAKEUP error.");
                else if(errorCode == ADC_ERROR_READREGISTER)
                    chk->setErrorMessage("ADC Error: ADC Read Register error");
                else if(errorCode == ADC_ERROR_WRITEREGISTER)
                    chk->setErrorMessage("ADC Error: Scan Pattern Streaming Error");
                else if(errorCode == ADC_ERROR_CONFIGURE)
                    chk->setErrorMessage("ADC Error: Scan Pattern Streaming Error");
                else if(errorCode == ADC_ERROR_SETBUFFER)
                    chk->setErrorMessage("ADC Error: Scan Pattern Streaming Error");
                else if(errorCode == ADC_ERROR_COMMAND)
                    chk->setErrorMessage("ADC Error: Scan Pattern Streaming Error");
                break;
            case NNO_error_code_sd:
                errorCode = error->errorCodes.sd;
                if(errorCode == NNO_ERROR_SDC_START)
                    chk->setErrorMessage("SD Card Error: SD Card Start Error.");
                else if(errorCode == NNO_ERROR_SD_CARD_HARD_ERROR)
                    chk->setErrorMessage("SD Card Error: Hardware Error.");
                else if(errorCode == NNO_ERROR_SD_CARD_INTERNAL_ERRROR)
                    chk->setErrorMessage("SD Card Error: Internal error.");
                else if (errorCode == NNO_ERROR_SD_CARD_DOES_NOT_WORK)
                    chk->setErrorMessage("SD Card Error: SD Card Does Not Work.");
                else if (errorCode == NNO_ERROR_SD_CARD_FILE_NOT_FOUND)
                    chk->setErrorMessage("SD Card Error: File Not Found error.");
                else if(errorCode ==  NNO_ERROR_SD_CARD_PATH_NOT_FOUND)
                    chk->setErrorMessage("SD Card Error: Path Not Found error.");
                else if(errorCode == NNO_ERROR_SD_CARD_INVALID_PATH)
                    chk->setErrorMessage("SD Card Error: Invalid Path error.");
                else if(errorCode == NNO_ERROR_SD_CARD_ACCESS_DENIED)
                    chk->setErrorMessage("SD Card Error: SD Card Access denied.");
                else if(errorCode == NNO_ERROR_SD_CARD_ACCESS_PROHIBITED)
                    chk->setErrorMessage("SD Card Error: SD Card Acces prohibited.");
                else if(errorCode == NNO_ERROR_SD_CARD_INVALID_OBJECT)
                    chk->setErrorMessage("SD Card Error: Invalid Object error.");
                else if(errorCode == NNO_ERROR_SD_CARD_WRITE_PROTECTED)
                    chk->setErrorMessage("SD Card Error: SD Card Write Protected.");
                else if(errorCode == NNO_ERROR_SD_CARD_INVALID_DRIVE_NUM)
                    chk->setErrorMessage("SD Card Error: SD Card Invalid Drive Number.");
                else if(errorCode == NNO_ERROR_SD_CARD_NOT_ENABLED)
                    chk->setErrorMessage("SD Card Error: SD Card Not Enabled.");
                else if(errorCode == NNO_ERROR_SD_CARD_INVALID_FILE_SYSTEM)
                    chk->setErrorMessage("SD Card Error: Invalid File System.");
                else if(errorCode == NNO_ERROR_SD_CARD_MKFS_INVALID_PARAM)
                    chk->setErrorMessage("SD Card Error: Invalid Parameter.");
                else if(errorCode == NNO_ERROR_SD_CARD_TIMEOUT)
                    chk->setErrorMessage("SD Card Error: Timeout error.");
                else if(errorCode == NNO_ERROR_SD_CARD_LOCKED)
                    chk->setErrorMessage("SD Card Error: Card Locked error.");
                else if(errorCode == NNO_ERROR_SD_CARD_NOT_ENOUGH_CORE)
                    chk->setErrorMessage("SD Card Error: Not Enough core error.");
                else if(errorCode == NNO_ERROR_SD_CARD_TOO_MANY_OPEN_FILES)
                    chk->setErrorMessage("SD Card Error: Too Many Open Files.");
                break;
            case NNO_error_code_eeprom:
                errorCode = error->errorCodes.eeprom;
                break;
            case NNO_error_code_ble:
                errorCode = error->errorCodes.ble;
                break;
            case NNO_error_code_spec_lib:
                errorCode = error->errorCodes.spec_lib;
                break;
            case NNO_error_code_hw:
                errorCode = error->errorCodes.hw;
                if(errorCode == NNO_ERROR_HW_START)
                    chk->setErrorMessage("HW Error: HardWare Start Error.");
                else if(errorCode == NNO_ERROR_HW_DLPC150)
                    chk->setErrorMessage("HW Error: DLPC150 Error.");
                else if(errorCode == NNO_ERROR_HW_MAX)
                    chk->setErrorMessage("HW Error: HardWare Max error.");
                break;
            case NNO_error_code_tmp:
                errorCode = error->errorCodes.tmp;
                if (errorCode == NNO_ERROR_TMP006_START)
                    chk->setErrorMessage("TMP006 Error: Start error.");
                else if (errorCode == NNO_ERROR_TMP006_MANUID)
                    chk->setErrorMessage("TMP006 Error: Invalid Manufacturing ID.");
                else if(errorCode ==  NNO_ERROR_TMP006_DEVID)
                    chk->setErrorMessage("TMP006 Error: Invalid Device ID.");
                else if(errorCode == NNO_ERROR_TMP006_RESET)
                    chk->setErrorMessage("TMP006 Error: Reset error");
                else if(errorCode == NNO_ERROR_TMP006_READREGISTER)
                    chk->setErrorMessage("TMP006 Error: Read Register Error");
                else if(errorCode == NNO_ERROR_TMP006_WRITEREGISTER)
                    chk->setErrorMessage("TMP006 Error: Write Register Error");
                else if(errorCode == NNO_ERROR_TMP006_TIMEOUT)
                    chk->setErrorMessage("TMP006 Error: Timeout Error");
                else if(errorCode == NNO_ERROR_TMP006_I2C)
                    chk->setErrorMessage("TMP006 Error: I2C Error");
                else if(errorCode == NNO_ERROR_TMP006_MAX)
                    chk->setErrorMessage("TMP006 Error: Max Error");
                break;
            case NNO_error_code_hdc:
                errorCode = error->errorCodes.hdc;
                if (errorCode == NNO_ERROR_HDC1000_START)
                    chk->setErrorMessage("HDC1000 Error: Start error.");
                else if (errorCode == NNO_ERROR_HDC1000_MANUID)
                    chk->setErrorMessage("HDC1000 Error: Invalid Manufacturing ID.");
                else if(errorCode ==  NNO_ERROR_HDC1000_DEVID)
                    chk->setErrorMessage("HDC1000 Error: Invalid Device ID.");
                else if(errorCode == NNO_ERROR_HDC1000_RESET)
                    chk->setErrorMessage("HDC1000 Error: Reset error");
                else if(errorCode == NNO_ERROR_HDC1000_READREGISTER)
                    chk->setErrorMessage("HDC1000 Error: Read Register Error");
                else if(errorCode == NNO_ERROR_HDC1000_WRITEREGISTER)
                    chk->setErrorMessage("HDC1000 Error: Write Register Error");
                else if(errorCode == NNO_ERROR_HDC1000_TIMEOUT)
                    chk->setErrorMessage("HDC1000 Error: Timeout Error");
                else if(errorCode == NNO_ERROR_HDC1000_I2C)
                    chk->setErrorMessage("HDC1000 Error: I2C Error");
                else if(errorCode == NNO_ERROR_HDC1000_MAX)
                    chk->setErrorMessage("HDC1000 Error: Max Error");
                break;
            case NNO_error_code_battery:
                errorCode = error->errorCodes.battery;
                chk->setErrorMessage("Empty Battery");
                break;
            case NNO_error_code_memory:
                errorCode = error->errorCodes.memory;
                chk->setErrorMessage("Not Enough Memory");
                break;
            }
        }
    }
}
// to get and set status of filesettings from spectrum class object
bool MainWindow::ifsaveone_csv()
{
     return spectrum.saveone_csv;
}
bool MainWindow::ifsavesep_csv()
{
     return spectrum.saveseperate_csv;
}
bool MainWindow::ifabsorbance_csv()
{
    return spectrum.absorbance_csv;
}

bool MainWindow:: ifintensity_csv()
{
     return spectrum.intensity_csv;
}

bool MainWindow::ifreflectance_csv()
{
     return spectrum.reflectance_csv;
}

bool MainWindow::ifsave_JCAMP()
{
     return spectrum.save_JCAMP;
}

bool MainWindow::ifabsorbance_JCAMP()
{
     return spectrum.absorbance_JCAMP;
}

bool MainWindow::ifintensity_JCAMP()
{
     return spectrum.intensity_JCAMP;
}
bool MainWindow::ifreflectance_JCAMP()
{
     return spectrum.reflectance_JCAMP;
}

bool MainWindow::ifsavebinary()
{
    return spectrum.savebinary;
}

void MainWindow::putsaveone_csv(bool checked)
{
    spectrum.saveone_csv = checked;
} 
 void MainWindow::putsavesep_csv(bool checked)
{
    spectrum.saveseperate_csv = checked;
}

 void MainWindow::putabsorbance_csv(bool checked)
 {
      spectrum.absorbance_csv=checked;
 }

 void MainWindow:: putintensity_csv(bool checked)
 {
       spectrum.intensity_csv=checked;
 }

 void MainWindow::putreflectance_csv(bool checked)
 {
       spectrum.reflectance_csv=checked;
 }

 void MainWindow::putsave_JCAMP(bool checked)
 {
       spectrum.save_JCAMP=checked;
 }
 void MainWindow::putabsorbance_JCAMP(bool checked)
 {
       spectrum.absorbance_JCAMP=checked;
 }
 void MainWindow::putintensity_JCAMP(bool checked)
 {
       spectrum.intensity_JCAMP=checked;
 }
 void MainWindow::putreflectance_JCAMP(bool checked)
 {
       spectrum.reflectance_JCAMP=checked;
 }

void MainWindow:: putsavebinary(bool checked)
{
    spectrum.savebinary=checked;
}

int MainWindow::PerformScanReadData(bool storeInSD, int numRepeats, void *pData, int *pBytesRead)
/*
 * This function asks the Nano to perform the scan and gets back the ScanData
 * @param storeInSD - I - a boolean to indicate if the current scan is to be stored in SD card
 * @param numRepeats - I - an integer indicating the number of times the scan should repeat in Nano
 * @param pData - O - The scanData is readback from Nano in this variable
 * @param pBytesRead - O - the size of the scnData read
 *
 */
{
    int size;
    int scanTimeOut;
    qint64  timeScanEnd;
    qint64  timeScanStart;
    unsigned int  devStatus;
    QString scanTimeText;


    // NNO_SetFixedPGAGain(true,1);   /* Only used for testing Fixed PGA command  */
    NNO_SetScanNumRepeats(numRepeats);
    scanTimeText.sprintf("Scan in progress.Estimated Scan time is approximately %3.3f seconds", (float)NNO_GetEstimatedScanTime()/1000.0);
    ui->labelScanStatus->setText(scanTimeText);
    QApplication::processEvents();

    scanTimeOut = NNO_GetEstimatedScanTime() * 3;
    timeScanStart = QDateTime::currentDateTime().toMSecsSinceEpoch();

    NNO_PerformScan(storeInSD);
    //Wait for scan completion
    if(NNO_ReadDeviceStatus(&devStatus) == PASS)
    {
        do
        {
            UpdateDeviceStatus(devStatus);
            QApplication::processEvents();
            if((devStatus & NNO_STATUS_SCAN_IN_PROGRESS) != NNO_STATUS_SCAN_IN_PROGRESS)
                break;
            timeScanEnd = QDateTime::currentDateTime().toMSecsSinceEpoch();
            if((timeScanEnd - timeScanStart) >= scanTimeOut)
            {
                *pBytesRead = 0;
                ui->labelScanStatus->setText("Wait for scan completion timed out");
                return FAIL;
            }
            QThread::msleep(50);
        }while(NNO_ReadDeviceStatus(&devStatus) == PASS);
    }
    else
    {
        ui->labelScanStatus->setText("Reading device status for scan completion failed");
        return FAIL;
    }

    timeScanEnd = QDateTime::currentDateTime().toMSecsSinceEpoch();
    lastScanTimeMS = timeScanEnd - timeScanStart;
    scanTimeText.sprintf("Scan time was %3.3f seconds", (float)lastScanTimeMS/1000.0);
    ui->labelScanStatus->setText(scanTimeText);

    *pBytesRead = NNO_GetFileSizeToRead(NNO_FILE_SCAN_DATA);

    if((size = NNO_GetFile((unsigned char *)pData, *pBytesRead)) != *pBytesRead)
    {
        *pBytesRead = size;
        ui->labelScanStatus->setText("Scan data read from device failed");
        return FAIL;
    }

    return PASS;
}


int MainWindow::InterpretScanData(void *pData, int *pBytesRead)
/* *
 * This function asks the Nano to interprets the scan and gets back the ScanData
 * @param storeInSD - I - a boolean to indicate if the current scan is to be stored in SD card
 * @param numRepeats - I - an integer indicating the number of times the scan should repeat in Nano
 * @param pData - O - The scanData is readback from Nano in this variable
 * @param pBytesRead - O - the size of the scnData read
 *
 */
{
    int size;
    int scanTimeOut;
    qint64  timeScanEnd;
    qint64  timeScanStart;
    unsigned int  devStatus;
    QString scanTimeText;
    qint64 lastScanTimeMS;


    scanTimeOut = NNO_GetEstimatedScanTime() * 3;
    timeScanStart = QDateTime::currentDateTime().toMSecsSinceEpoch();
    timeScanEnd = QDateTime::currentDateTime().toMSecsSinceEpoch();
    NNO_InterpretScan();
    //Wait for scan completion
    if(NNO_ReadDeviceStatus(&devStatus) == PASS)
    {
        do
        {
            UpdateDeviceStatus(devStatus);
            QApplication::processEvents();
            if((devStatus & NNO_STATUS_SCAN_INTERPRET_IN_PROGRESS) != NNO_STATUS_SCAN_INTERPRET_IN_PROGRESS)
                break;
            timeScanEnd = QDateTime::currentDateTime().toMSecsSinceEpoch();
            if((timeScanEnd - timeScanStart) >= scanTimeOut)
            {
                *pBytesRead = 0;
                return FAIL;
            }
            QThread::msleep(500);
        }while(NNO_ReadDeviceStatus(&devStatus) == PASS);
    }
    else
    {
        return FAIL;
    }

    lastScanTimeMS = timeScanEnd - timeScanStart;
    scanTimeText.sprintf("Interpret time was %3.3f seconds", (float)lastScanTimeMS/1000.0);
    QString msg = QString("%1.Results saved in %2").arg(scanTimeText,filepath.GetuserPath());
    ui->labelScanStatus->setText(msg);

    *pBytesRead = NNO_GetFileSizeToRead(NNO_FILE_INTERPRET_DATA);

    if((size = NNO_GetFile((unsigned char *)pData, *pBytesRead)) != *pBytesRead)
    {
        *pBytesRead = size;
        return FAIL;
    }

    return PASS;
}


void MainWindow::on_pushButton_errorStatus_clicked()
/*
 * This is a hander function for pushButton_errorStatus cliecked() event
 * This Reset's the error status bits and changes the error status
 * display accordingly
 *
 */
{
    NNO_ResetErrorStatus();

    NNO_error_status_struct error_status;

    if(NNO_ReadErrorStatus(&error_status) == PASS)
    {
       UpdateErrorStatus(&error_status);
    }

}

bool MainWindow::event(QEvent *ev)
/*
 * Handles the resize event of the MainWindow
 * If there is Graphic Scene that exists currently (i.e any spectrum displayed at present)
 * That is either expanded or shrinked to fit in the View widget
 * Rest of the events are handled as it is by deault MainWindow functions
 * @param ev - I - the current eventtype
 */
{
    if(ev->type() == QEvent::Resize) {
        if(scene)
         ui->graphicsView->fitInView(scene->sceneRect());
    }
    return QMainWindow::event(ev);
}

void MainWindow::on_UART_Check_clicked()
/**
  * This is a handler funcion for UART check-box on the Test tab
  * if checked - Gets the COM port from the text box and opens the corresponding port
  * if unchecked closes the port connection
*/
{
    if(ui->UART_Check->isChecked())
    {
        QString port = ui->lineEdit_COMport->text();


        if(Serial_Open(port.toLocal8Bit().data()) != 0)
        {
            QString title("Error Message");
            QString text("Please enter the correct COM portname");
            QMessageBox msgBox(QMessageBox::Warning, title, text, QMessageBox::NoButton, this);
            msgBox.exec();
            NNO_SetUARTConnected(false);
        }
        else
            NNO_SetUARTConnected(true);

    }
    else
    {
        Serial_Close();
        NNO_SetUARTConnected(false);
    }
}

void MainWindow::on_checkBox_always_on_toggled(bool checked)
{
    if(checked)
    {
            NNO_SetScanControlsDLPCOnOff(false);
            NNO_DLPCEnable(true, true);
            //PGA Gain combobox - first item to be labeled Prev
            ui->comboBox_pga_gain->setItemText(0, "Prev");
            //Set PGA gain to 64
            ui->comboBox_pga_gain->setCurrentIndex(1);
    }
    else
    {
            NNO_SetScanControlsDLPCOnOff(true);
            NNO_DLPCEnable(false, false);
            //PGA Gain combobox - first item to be labeled Auto
            ui->comboBox_pga_gain->setItemText(0, "Auto");
    }
}

void MainWindow::on_Filesettings_clicked()
{
    filewindow = new Filesettings_window(this);
    filewindow->show();
}
