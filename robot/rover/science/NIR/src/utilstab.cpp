/*****************************************************************************
 *
 * This module provides function handlers and supporting functions for different controls on utils tab.
 *
 * Copyright (c) 2015 Texas Instruments Incorporated.
 * ALL RIGHTS RESERVED
 *
 ******************************************************************************/

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QTimer>
#include <QTime>
#include <QCheckBox>

//*****************************************************************************
//
// Globals whose values are set or overridden via command line parameters.
//
//*****************************************************************************
bool g_bVerbose      = false;
bool g_bQuiet        = false;
bool g_bOverwrite    = false;
bool g_bUpload       = false;
bool g_bClear        = false;
bool g_bBinary       = false;
bool g_bEnumOnly     = false;
bool g_bDisregardIDs = false;
bool g_bSkipVerify   = false;
bool g_bWaitOnExit   = false;
bool g_bReset        = false;
bool g_bSwitchMode   = false;
unsigned long g_ulAddress  = 0;
unsigned long g_ulAddressOverride  = 0xFFFFFFFF;
unsigned long g_ulLength = 0;
int g_iDeviceIndex = 0;
extern EVM evm;
extern FilePath filepath;

//helper functions

int MainWindow::writeFile(unsigned char *pByteArray, long long bytesToSend, QProgressBar *progressBar, bool updateBar)
/**
 * This function calls the API WriteFileData to write the DLPC Flash image file
 * @param pByteArray - I - pointer to the Flash Image ByteArray to be written to the DLPC Flash
 * @param bytesToSend - I - size of the Flash image in bytes
 * @param progressBar - I - pointer to the Progress bar which needs to be updated during the Flash download process
 * @param updateBar - I - a boolean to determine whether the progress bar needs to be updated or not
 * @return 0 = PASS
 *         <0 = FAIL
 */
{
    long long sizeInBytes;
    int bytesSent;
    int ret=0;
    long long percent_completion = 0;

    sizeInBytes = bytesToSend;
    if(updateBar)
        progressBar->setValue(0);

    //Disable usb connection check during this process
#ifdef USB_POLL_ENABLED
    usbPollTimer->stop();
#endif

    while(bytesToSend > 0)
    {
        bytesSent = NNO_WriteFileData(pByteArray+sizeInBytes-bytesToSend, bytesToSend);

        if(bytesSent < 0)
        {
            showError("Data send Failed");
            ret = -2;
            break;
        }

        bytesToSend -= bytesSent;
        if(percent_completion != (((sizeInBytes-bytesToSend)*100)/sizeInBytes))
        {
            percent_completion = (((sizeInBytes-bytesToSend)*100)/sizeInBytes);
            if(updateBar)
                progressBar->setValue(percent_completion);
        }
        QApplication::processEvents(); //Update the GUI
    }

//Restart usb connection check
#ifdef USB_POLL_ENABLED
    usbPollTimer->start();
#endif

    return ret;
}


#ifdef Q_OS_WIN
int MainWindow::DownloadImage(tLMDFUHandle hHandle)

/**
* Download an image to the the device identified by the passed handle.  The
* image to be downloaded and other parameters related to the operation are
* controlled by command line parameters via global variables.
*
* @return  0 = PASS success or a positive error return code on failure.
*          -1 = FAIL
*
*/
{
    QFile imgFile(ui->lineEdit_tiva_fw_filename->text());
    char displayStr[255];
    tLMDFUErr eRetcode;
    unsigned char *pcFileBuf = NULL;
    size_t iLen;
    bool bTivaFormat;
    long long dataLen;

    bool checkAppSignature = ui->checkBox_appsig->isChecked();
    //
    // Does the input file exist?
    //
    if(!imgFile.open(QIODevice::ReadOnly))
    {
        showError("Unable to open binary file. Copy binary file to a folder with Admin/read/write permission and try again\n");
        return FAIL;
    }

    dataLen = imgFile.size();
    pcFileBuf = (unsigned char *)malloc(dataLen);
    if(pcFileBuf == NULL)
    {
        imgFile.close();
        showError("Memory alloc for file read failed");
        return FAIL;
    }
    imgFile.read((char *)pcFileBuf, dataLen);

    iLen = dataLen;

    imgFile.close();


    //
    // Check to see whether this is a binary or a DFU-wrapped file.
    //
    eRetcode = LMDFUIsValidImage(hHandle, pcFileBuf, (unsigned long)iLen, &bTivaFormat);

    //
    // Is the image for the target we are currently talking to?
    //
    if((eRetcode == DFU_ERR_UNSUPPORTED) && !g_bDisregardIDs)
    {
        showError("This image does not appear to be valid for the target device.");
        free(pcFileBuf);
        return(14);
    }

    if(((eRetcode == DFU_OK) ||
                ((eRetcode == DFU_ERR_UNSUPPORTED) && g_bDisregardIDs)) &&
            bTivaFormat)
    {
        if(checkAppSignature == false)
                {
                    if(false==Frmw_IsTIVAFirmware(pcFileBuf ,dataLen))
                    {
                        showError("This image is not valid NIRNANO FW Image");
                        free(pcFileBuf);
                        return(FAIL);
                    }
                }
        //
        // This is a DFU formatted image and either it is intended for the
        // target device or the user wants us to ignore the IDs in the file.
        // Since it also contains a Tiva prefix, we can send it to the
        // main download function.
        //

        eRetcode = LMDFUDownload(hHandle, pcFileBuf, (unsigned long)iLen, g_bSkipVerify,
                g_bDisregardIDs, NULL);
    }
    else
    {
        if(checkAppSignature == false)
                {
                    if(false==Frmw_IsTIVAFirmware(pcFileBuf , dataLen))
                    {
                        showError("This image is not valid NIRNANO FW Image");
                        free(pcFileBuf);
                        return(FAIL);
                    }
                }
        //
        // This is not a DFU-formatted file so we download it as a binary.  In
        // this case, we need to pass the flash address passed by the user or,
        // if this was not provided, the application start address that the
        // device told us to use when we queried its capabilities earlier.
        //

        //
        // Did the file contain a DFU suffix but no Tiva prefix? If so,
        // we remove the DFU suffix since this doesn't get downloaded.  The
        // length of the suffix is in the 5th last byte.
        //
        if((eRetcode == DFU_OK) || (eRetcode == DFU_ERR_UNSUPPORTED))
        {
            iLen -= pcFileBuf[iLen - 5];
        }

        eRetcode = LMDFUDownloadBin(hHandle, pcFileBuf, (unsigned long)iLen,
                g_ulAddress, g_bSkipVerify, NULL);
    }
    //
    // Free the file buffer memory.
    //
    free(pcFileBuf);

    if(eRetcode != DFU_OK)
    {
        sprintf(displayStr, "Error %s (%d) reported during file download\n",
                LMDFUErrorStringGet(eRetcode), eRetcode);
        showError ( displayStr );
        return(FAIL);
    }
    else
    {
        return(PASS);
    }
}

tLMDFUErr MainWindow::ResetTiva(tLMDFUDeviceState *pState,bool bCheckStatus,
                                unsigned char *pcData, int iLength)
/**
 * This funtion is to reset the TIVA after the Fiwrware download that is when it is in boot loader mode
 * @param pState - I - the current state of the DFU device
 * @param bCheckStatus- I - boolean to keep polling to see if the transaction has succeeded
 * @param pcData - I - The DFUDownload header with Reset command enabled
 * @param iLength - I - size of the tDFUDownloadHeader structure
 * @return tLMDFUEr = 0 if PASS
 *                  < 0 if FAIL
 *
 *
 */
{
    unsigned short  usCount;
    BOOL bRetcode;
    tDFUGetStatusResponse sStatus;
    //
    // Make sure the size provided can be sent in a single DFU transfer.
    //
    if(iLength > (int)pState->usTransferSize)
    {
        return(DFU_ERR_INVALID_SIZE);
    }

    //
    // Send the download request with the supplied payload.
    //
    bRetcode = Endpoint0Transfer(pState->hUSB, (REQUEST_TRANSFER_OUT |
                                                REQUEST_TYPE_CLASS | REQUEST_RECIPIENT_INTERFACE),
                                 USBD_DFU_REQUEST_DNLOAD, pState->usBlockNum++,
                                 pState->usInterface, iLength, (unsigned char *)pcData,
                                 &usCount);

    //
    // Did the transfer complete successfully?
    //
    if(!bRetcode)
    {
        return(DFU_ERR_UNKNOWN);
    }

    //
    // Did we process the amount of data we expected?
    //
    if((int)usCount != iLength)
    {
        return(DFU_ERR_UNKNOWN);
    }
    int retryCount = 0;
    if(bCheckStatus)
    {
        //
        // Keep checking the device status until we see a problem reported
        // or the device tells us that the previous download request completed.
        //
        do
        {
            bRetcode = Endpoint0Transfer(pState->hUSB, (REQUEST_TRANSFER_IN |
                                                        REQUEST_TYPE_CLASS |
                                                        REQUEST_RECIPIENT_INTERFACE),
                                         USBD_DFU_REQUEST_GETSTATUS, 0,
                                         pState->usInterface,
                                         sizeof(tDFUGetStatusResponse),
                                         (unsigned char *)&sStatus, &usCount);

            //
            // Did the transaction succeed?
            //

            retryCount++;

            //
            // Did we get the data we expected?
            //
            if(usCount != sizeof(tDFUGetStatusResponse))
            {

                return(DFU_ERR_UNKNOWN);
            }

            //
            // If the device is still busy, we need to wait a while before
            // polling again.
            //
            if((sStatus.bState == STATE_DNLOAD_SYNC) ||
                    (sStatus.bState == STATE_DNBUSY))
            {
                __LONG32 dwTimeout;

                //
                // Read the timeout in milliseconds from the 3 byte field
                // in the status structure.
                //
                dwTimeout = sStatus.bwPollTimeout[0] +
                        (sStatus.bwPollTimeout[1] << 8) +
                        (sStatus.bwPollTimeout[2] << 16);

                //
                // Twiddle our thumbs until this number of milliseconds has
                // elapsed.
                //
                QThread::msleep(dwTimeout);
            }

        }
        while(((sStatus.bState == STATE_DNLOAD_SYNC) ||
               (sStatus.bState == STATE_DNBUSY)) && (retryCount < 50));

        //
        // Return the appropriate return code depending upon the device status.
        //
        return(DFU_OK);
    }
    else
    {
        //
        // We didn't have to check the device status so tell the caller all is
        // apparently well.
        //
        return(DFU_OK);
    }
}
#endif
//handler functions for different widget slots on the Utils tab

void MainWindow::on_pushButton_dlpc_fw_browse_clicked()
/**
 * This is a handler function for pushButton_dlpc_fw_browse on Utils tab clicked() event
 * Opens a file browse dialog poiting to the last saved path directory
 *
 */
{
    QString fileName;

    fileName = QFileDialog::getOpenFileName(this,
            QString("Select Image to load"),
            filepath.GetdlpcFirmwarePath(),
            tr("img files (*.img)"));

    ui->pushButton_dlpc_fw_update->setEnabled(false);
    ui->label_dlpc_fw_update_progress->setText("");

    if(!fileName.isEmpty())
    {
        QFile imgFileIn(fileName);

        ui->lineEdit_dlpc_fw_filename->setText(fileName);
        QFileInfo firmwareFileInfo;
        firmwareFileInfo.setFile(fileName);
        filepath.SetdlpcFirmwarePath(firmwareFileInfo.absolutePath());
        ui->pushButton_dlpc_fw_update->setEnabled(true);
    }
}

void MainWindow::on_checkBox_hibernateEnabled_clicked()
/**
 * This is a handler for checkBox_hibernateEnabled on Utils tab, clicked() event
 * Sends command to NIRscan Nano to set hibernation enabled status whenever changed
 */
{
    NNO_SetHibernate(ui->checkBox_hibernateEnabled->isChecked());
}

void MainWindow::on_pushButton_GetDateTime_clicked()
/**
 * This is a handler for pushButton_GetDateTime button on Utils tab, clicked() event
 * Gets the Date and time from the EVM and displays in the
 * repsective text boxes
 *
 */
{
    uint8 year, month, day, wday, hour, min, sec;
    QTime time;
    QDate date;

    NNO_GetDateTime( &year, &month, &day, &wday, &hour, &min, &sec );
    date.setDate( (int) (year + 2000), (int) month, (int) day );
    time = QTime( (int)hour, (int)min, (int)sec, (int)0 );

    ui->dateEdit->setDate( date );
    ui->timeEdit->setTime( time );
}

void MainWindow::on_pushButton_SyncDateTime_clicked()
/**
 * This is a handler for pushButton_SyncDateTime button on Utils tab, clicked() event
 * Gets the current Date and Time and sets it in the EVM
 * also shows in the repsective text boxes
 *
 */
{
    QDate currentDate = QDate::currentDate();
    QTime currentTime = QTime::currentTime();
    uint8 year, month, day, wday;
    uint8 hour, minute, second;

    year = currentDate.year() - 2000;
    month = currentDate.month();
    day = currentDate.day();
    wday = currentDate.dayOfWeek();

    hour = currentTime.hour();
    minute = currentTime.minute();
    second = currentTime.second();

    NNO_SetDateTime( year, month, day, wday, hour, minute, second );

    ui->dateEdit->setDate( currentDate );
    ui->timeEdit->setTime( currentTime );
}


void MainWindow::on_pushButton_dat_files_browse_clicked()
/**
 * This is a handler function for pushButton_dat_files_browse on Utils Tab clicked() event
 * Opens a file browse dialog poiting to the last saved path directory
 * one or Multiple .dat files can be selected for conversion to .csv format
 */

{

    datfileNames = QFileDialog::getOpenFileNames(this,
                                              QString("Select files for conversion"),
                                              filepath.GetuserPath(),
                                              "*.dat");
    ui->lineEdit_dattocsv->setText("");
    if(datfileNames.isEmpty())
        return;


    QString filelist;
    QDir filesdir = QFileInfo(QFile(datfileNames.at(0))).absoluteDir();
    for(int i = 0; i < datfileNames.size(); i++)
    {
        QString scanFileName = datfileNames.at(i);
        QString name = filesdir.relativeFilePath(scanFileName);
        filelist.append(name);
        filelist.append(",");
    }
    ui->lineEdit_dattocsv->setText(filelist);
    ui->pushButton_dat_to_csv->setEnabled(true);
    ui->progressbar_dat_to_csv->setValue(0);
    ui->label_dat_to_csv_progress->setText("");


}

void MainWindow::on_pushButton_dat_to_csv_clicked()
/**
 * This is a handler function for pushButton_dat_to_csv on Utils Tab clicked() event
 * Iterates through the list of files selected from the browse button above
 * reads the scan data from each file and saves it to a corresponding .csv file with the same name
 */

{
    if(datfileNames.isEmpty())
        return;

    char displayStr[255];

    ui->progressbar_dat_to_csv->setValue(0);
    ui->label_dat_to_csv_progress->setText("Conversion in progress");
    for(int i = 0; i < datfileNames.size(); i++)
    {
        QString scanFileName = datfileNames.at(i);
       if(spectrum.ReadSavedScanFromFile(scanFileName) == FAIL)
       {
           sprintf(displayStr, "Unable to read file %d",i+1);
           showError(displayStr);
           continue;
       }
        scanFileName.replace(".dat",".csv",Qt::CaseInsensitive);
        spectrum.SaveCombinedCSV(scanFileName);
        ui->progressbar_dat_to_csv->setValue(((i+1)/datfileNames.size()) * 100);
        QApplication::processEvents();
    }
    ui->progressbar_dat_to_csv->setValue(100);
    ui->label_dat_to_csv_progress->setText("Conversion complete");
}

void MainWindow::on_pushButton_tiva_fw_browse_clicked()
/**
 * This is a handler function for pushButton_tiva_fw_browse on Utils Tab clicked() event
 * Opens a file browse dialog poiting to the last saved path directory
 *
 */

{
    QString fileName;

    fileName = QFileDialog::getOpenFileName(this,
            QString("Select Image to load"),
            filepath.GettivaFirmwarePath(),
            tr("bin files (*.bin)"));

    ui->pushButton_tiva_fw_usb_update->setEnabled(false);
    ui->label_tiva_fw_update_progress->setText("");

    if(!fileName.isEmpty())
    {
        QFile imgFileIn(fileName);
        ui->lineEdit_tiva_fw_filename->setText(fileName);
        QFileInfo firmwareFileInfo;
        firmwareFileInfo.setFile(fileName);
        filepath.SettivaFirmwarePath(firmwareFileInfo.absolutePath());
        ui->pushButton_tiva_fw_usb_update->setEnabled(true);
    }
}

void MainWindow::on_pushButton_dlpc_fw_update_clicked()
/**
 * This is a handler function for pushButton_dlpc_fw_update on Utils Tab clicked() event
 * Initiates the DPC FW download process.
 * Checks if it is a valid FW image by verifying the signature.
 * calculates the checksum, calls the writeFile and verifies the returned checksum
 *
 */
{

    QFile imgFile(ui->lineEdit_dlpc_fw_filename->text());
    char displayStr[255];
    unsigned char *pByteArray=NULL;
    unsigned int dataLen;
    unsigned int expectedChecksum=0, checksum=0;
    int ret;
    unsigned int i;

    if(!imgFile.open(QIODevice::ReadOnly))
    {
        showError("Unable to open image file. Copy image file to a folder with Admin/read/write permission and try again\n");
        return;
    }
    dataLen = imgFile.size();
    pByteArray = (unsigned char *)malloc(dataLen);
    if(pByteArray == NULL)
    {
        imgFile.close();
        showError("Memory alloc for file read failed");
        return;
    }
    imgFile.read((char *)pByteArray, dataLen);

    //CR18546 - KV
    if(!Frmw_IsFirwareSignatureMatch(pByteArray))
    {
        showError("Invalid DPC150 image file\n");
        free(pByteArray);
        return;
    }

    ret = NNO_SetFileSizeAndAction(dataLen, NNO_FILE_DLPC_UPDATE);

    if (ret < 0)
    {

        free(pByteArray);
        return;
    }

    for(i=0; i<dataLen; i++)
    {
        expectedChecksum += pByteArray[i];
    }

    //Inserted this dealy to workaround some checksum mismatch errors
    QThread::msleep(1000);

    ui->label_dlpc_fw_update_progress->setText("Updating Firmware Image");

    if( writeFile(pByteArray, dataLen, ui->progressbar_dlpc_fw_update, true) < 0 )
    {
        imgFile.close();
        free(pByteArray);
        ui->label_dlpc_fw_update_progress->setText("Update failed");
        return;
    }

    ui->label_dlpc_fw_update_progress->setText("Waiting for checksum verification");
    QApplication::processEvents(); //Update the GUI

    if(NNO_GetFlashChecksum(&checksum) < 0)
    {
        sprintf(displayStr, "Error reading checksum: Expected %x", expectedChecksum);
        showError(displayStr);
        ui->label_dlpc_fw_update_progress->setText("Update failed");
    }
    else  if(checksum != expectedChecksum)
    {
        sprintf(displayStr, "Checksum mismatch: Expected %x; Received %x", expectedChecksum, checksum);
        showError(displayStr);
        ui->label_dlpc_fw_update_progress->setText("Update failed");
    }
    else
    {
        showError("Update completed successfully");
        ui->label_dlpc_fw_update_progress->setText("Update completed successfully");
    }

    ui->progressbar_dlpc_fw_update->setValue(0);
    imgFile.close();
    free(pByteArray);

}

void MainWindow::on_pushButton_tiva_fw_usb_update_clicked()
/**
 *This function is a handler for pushButton_tiva_fw_usb_update on Utils Tab and clicked() event
 * Initialites the transfer for TIVA FW Download visa USB
 * Enters into BootLoader mode ,opens the DFU device calls the DownloadImage()
 * rests the device once the download is complete
 */
{
    tLMDFUErr eRetcode;
    char displayStr[255];
    int iDevIndex;
    bool bCompleted;
#ifdef Q_OS_WIN
    tLMDFUDeviceInfo sDevInfo;
    tLMDFUHandle hHandle;
    tLMDFUParams sDFU;
#endif
    int retryCount = 0;

    //Disable usb connection check during this process
#ifdef USB_POLL_ENABLED
    usbPollTimer->stop();
#endif

#ifdef Q_OS_WIN
    //this box is to be checked if Tiva flash is empty and Tiva is already in bootloader mode.
    if(!ui->checkBox_emptyTiva->isChecked())
        NNO_GotoTivaBootLoader();

    eRetcode = LMDFUInit();
    //
    // Could we load the DLL and query all its entry points successfully?
    //
    ui->progressbar_tiva_fw_usb_update->setValue(10);
    if(eRetcode != DFU_OK)
    {
        //
        // Oops - something is wrong with the DFU DLL.
        //
        if(eRetcode == DFU_ERR_NOT_FOUND)
        {
            showError("The driver, lmdfu.dll, for the USB Device Firmware Upgrade device cannot be found.");
            ui->label_tiva_fw_update_progress->setText("Update failed");
        }
        else if(eRetcode == DFU_ERR_INVALID_ADDR)
        {
            showError("The driver for the USB Device Firmware Upgrade device was found but appears to be a version which this program does not support. ");
            ui->label_tiva_fw_update_progress->setText("Update failed");
        }
        else
        {
            showError("An error was reported while attempting to load the device driver for the USB Device Firmware Upgrade device.");
            ui->label_tiva_fw_update_progress->setText("Update failed");
        }

        //
        // Exit now with the appropriate error code.
        //
        return;
    }
    iDevIndex = 0;
    bCompleted = false;

    do
    {
        //
        // Try to open a device.
        //
        eRetcode = LMDFUDeviceOpen(iDevIndex, &sDevInfo, &hHandle);

        //
        // Were we successful?
        //
        if(eRetcode == DFU_OK)
        {
            if(iDevIndex == g_iDeviceIndex)
            {
                //
                // We have found the required device
                // Is the device currently in runtime mode?
                //
                sDevInfo.bDFUMode = true;

                if(sDevInfo.bSupportsTivaExtensions)
                {
                    //
                    // Read flash size parameters from the device.
                    //
                    eRetcode = LMDFUParamsGet(hHandle, &sDFU);
                    if(eRetcode != DFU_OK)
                    {
                        sprintf(displayStr, "Error %s (%d) reading flash parameters.", LMDFUErrorStringGet(eRetcode), eRetcode);
                        //						ui->label_tiva_fw_update_progress->setText(displayStr);
                    }
                    ui->progressbar_tiva_fw_usb_update->setValue(20);
                    QApplication::processEvents(); //Update the GUI
                }

                //
                // No errors so far?
                //
                if(eRetcode == DFU_OK)
                {
                    ui->progressbar_tiva_fw_usb_update->setValue(30);
                    ui->label_tiva_fw_update_progress->setText("Firmware download in progress");
                    QApplication::processEvents(); //Update the GUI

                    //
                    // Download an image to the device if a filename has been provided.
                    //
                    if(DownloadImage(hHandle) == PASS)
                    {
                        ui->progressbar_tiva_fw_usb_update->setValue(100);
                        ui->label_tiva_fw_update_progress->setText("Update completed successfully");
                        showError("Update completed successfully");
                    }
                    else
                    {
                        ui->progressbar_tiva_fw_usb_update->setValue(100);
                        ui->label_tiva_fw_update_progress->setText("Update failed");
                        showError("Update failed");
                    }
                    QApplication::processEvents(); //Update the GUI
                    //
                    // At this point, we've done whatever we have been asked
                    // to do so make sure we exit the loop next time we get
                    // to the end.
                    //
                    bCompleted = true;

                }
            }

            //retry if there is an error when trying to open the device
            retryCount++;
        }
    }
    while((retryCount < 50) &&  !bCompleted);


    // eRetcode = _LMDFUDeviceClose(hHandle, g_bReset);
    //CR18529 - automatically reet TIVA after download

    // Send the target the command telling it to reset.  If this succeeds, the
    // device will not be accessible after this function returns.
    //
    tDFUDownloadHeader sHeader;
    sHeader.ucCommand = TIVA_DFU_CMD_RESET;

    //
    // Get our state information.
    //
    tLMDFUDeviceState* pState = (tLMDFUDeviceState *)hHandle;
#endif

#ifdef USB_POLL_ENABLED
    usbPollTimer->start();
#endif

    USB_Close(); //So that we reopen connection when Tiva is back after reset
#ifdef Q_OS_WIN
    eRetcode = ResetTiva(pState, true, (unsigned char *)&sHeader,
                         sizeof(tDFUDownloadHeader));
#endif

}

void MainWindow::on_pushButton_hibernate_clicked()
/**
 *This function is a handler for pushButton_hibernate on Utils tab, clicked() event
 *Gets the TIVA in hibernate mode
 */
{
    NNO_HibernateMode();
    USB_Close(); //So that we reopen connection when Tiva is back after reset
    readVersionAndUpdate(); //to update device status in the GUI soon after hibernate
    showError("Hibernation activated\nUse WAKE button on the EVM to exit hiberation");
}

int MainWindow::on_pushButton_reset_tiva_clicked()
/**
 *This function is a handler for pushButton_reset_tiva on Utils tab, clicked() event
 *Resets the TIVA when it is in Normal mode
 */
{

    NNO_ResetTiva();
    USB_Close(); //So that we reopen connection when Tiva is back after reset
    readVersionAndUpdate(); //to update device status in the GUI soon after reset
    return PASS;
}

void MainWindow::on_pushButton_UpdateRefCalData_clicked()
/**
 *This function is a handler for pushButton_UpdateRefCalData on Utils tab, clicked() event
 * This functionality is added to correct the wrong reference data stored in first set of
 * EVMs shipped
 * Aagain perform the Reference Calibration and saves the updated/current reference Calibration data
 * in EVM
 */
{


    const char* text = "This scan will overwrite the existing stored Factory Reference Calibration.\n\n To replace the Factory Reference Calibration, cover the sampling window of the reflective illumination module with the desired reference.\n\nIf a calibrated reference is unavailable, a White-out sample can be used. If a White-out sample is used, check the box below to apply a correction factor to the reference.\n\nIf using a different input module, prepare the NIRscan Nano in the desired condition for the reference.";
    QMessageBox prompt(QMessageBox::Information, "Reference Calibration",text,
            QMessageBox::NoButton);
    QCheckBox applyCorrection(QObject::tr("Check the box when using a White-out sample to apply a correction to approximate Permaflect spectrum.\nLeave box unchecked to store the unaltered scanned spectrum as the factory reference."), &prompt);

    // HACK: BLOCKING SIGNALS SO QMessageBox WON'T CLOSE
    applyCorrection.blockSignals(true);

    prompt.addButton(&applyCorrection, QMessageBox::ActionRole);


    QAbstractButton* pYES = (QAbstractButton*)prompt.addButton(tr("Scan and Replace"), QMessageBox::ActionRole);
    QAbstractButton* pNO  = (QAbstractButton*)prompt.addButton(QMessageBox::Cancel);
    prompt.setDefaultButton(QMessageBox::Cancel);

    prompt.exec();

    if (prompt.clickedButton() == pNO)
    {
        return;
    }

    else if(prompt.clickedButton() == pYES)
    {

    void *pData = malloc(SCAN_DATA_BLOB_SIZE);

    if(pData == NULL)
    {
        showError("Out of Memory");
        return;
    }

    PerformRefCalScan(pData);

   if(applyCorrection.checkState() == Qt::Checked)
    {
        NNO_UpdateRefCalDataWithWORefl();
    }

    NNO_SaveRefCalPerformed();

    ui->progressbar_cal->setValue(100);

    //the refCal data might be modified in the TIVA if it is with whiteout
    //so fetch the data from EVM and set in Spectrum.
    evm.FetchRefCalData();

    /* Set the reference */
    spectrum.SetData(pData, evm.GetRefCalDataBlob());
    free(pData);
    spectrum.SaveReferenceToFile();
    }

    return;
}

