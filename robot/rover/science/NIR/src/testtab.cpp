/*****************************************************************************
 *
 * This module provides function handlers and supporting functions for different controls on test tab.
 *
 * Copyright (c) 2015 Texas Instruments Incorporated.
 * ALL RIGHTS RESERVED
 *
 ******************************************************************************/
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "NNOStatusDefs.h"
extern bool g_BluetoothTest;
extern FilePath filepath;
//helper functions for the controls on Test Tab
void MainWindow::clearTivaTestResults()
/**
 * This function Clears all the GUI controls indicating the last runt TIVA tests' status
 */
{
    ui->label_result_eeprom->clear();
    ui->label_result_sdram->clear();
    ui->label_result_hdc->clear();
    ui->label_result_bt->clear();
    ui->label_result_sdc->clear();
    ui->label_result_led->clear();

    ui->label_remark_eeprom->clear();
    ui->label_remark_sdram->clear();
    ui->label_remark_hdc->clear();
    ui->label_remark_bt->clear();
    ui->label_remark_sdc->clear();
    ui->label_remark_led->clear();
}

void MainWindow::clearDLPCTestResults()
/**
 * This function Clears all the GUI controls indicating the last runt DLPC tests' status
 */

{
    ui->label_result_dlpc150->clear();
    ui->label_remark_dlpc150->clear();
}

void MainWindow::clearDetectorTestResults()
/**
 * This function Clears all the GUI controls indicating the last runt Detector tests' status
 */
{
    ui->label_result_adc->clear();
    ui->label_result_tmp->clear();

    ui->label_remark_adc->clear();
    ui->label_remark_tmp->clear();
}

//handler functions for different widget slots on the Test Tab
int MainWindow::on_pushButton_test_eeprom_clicked()
/**
 * This is a handler function for pushButton_test_eeprom on Test Tab clicked() event
 * Calls the API Test EEPROM function and updates the remark and result labels to show pass/fail
 * @return 0 = PASS
 *         -1 = FAIL
 */
{
    ui->label_result_eeprom->clear();
    ui->label_remark_eeprom->clear();

    int result = NNO_TestEEPROM();
    if(result == PASS)
    {
        ui->label_remark_eeprom->setText("PASS");
        ui->label_result_eeprom->setPixmap(QPixmap(":/new/Icons/pass_icon.png"));
    }
    else if(result == FAIL)
    {
        ui->label_remark_eeprom->setText("FAIL");
        ui->label_result_eeprom->setPixmap(QPixmap(":/new/Icons/fail_icon.png"));
    }
    else
    {
        ui->label_remark_eeprom->setText("READ FAIL");
        ui->label_result_eeprom->setPixmap(QPixmap(":/new/Icons/fail_icon.png"));
    }
    return result;
}


void MainWindow::on_pushButton_test_adc_clicked()
/**
 * This is a handler function for pushButton_test_adc on Test Tab clicked() event
 * Calls the API Test ADC function and updates the remark and result labels to show pass/fail
 * @return none ; error string will be stored in errorString
 *
 */
{
    uint16_t err;
    ui->label_result_adc->clear();
    ui->label_remark_adc->clear();

    NNO_ClearSpecificError(NNO_error_code_adc);
    int result = NNO_TestADC();
    if(result == PASS)
    {
        ui->label_remark_adc->setText("PASS");
        ui->label_result_adc->setPixmap(QPixmap(":/new/Icons/pass_icon.png"));
    }
    else if(result == FAIL)
    {
        err = NNO_GetSpecificErrorcode(NNO_error_code_adc);
        NNO_ADC_GetErrorMessageFromErrorCode(err, errorString);
        NNO_ClearSpecificError(NNO_error_code_adc);
        ui->label_remark_adc->setText(errorString);
        ui->label_result_adc->setPixmap(QPixmap(":/new/Icons/fail_icon.png"));
    }
    else
    {
        ui->label_remark_adc->setText("READ FAIL");
        ui->label_result_adc->setPixmap(QPixmap(":/new/Icons/fail_icon.png"));
    }
}

void MainWindow::on_pushButton_test_tmp_clicked()
/**
 * This is a handler function for pushButton_test_tmp on Test Tab clicked() event
 * Calls the API Test Tmp function and updates the remark and result labels to show pass/fail
 * @return none ; error string will be stored in errorString
 *
 */
{
    uint16_t err;
    int8_t result;

    ui->label_result_tmp->clear();
    ui->label_remark_tmp->clear();

    NNO_ClearSpecificError(NNO_error_code_tmp);
    result = NNO_TestTMP();
    if(result == PASS)
    {
        ui->label_remark_tmp->setText("PASS");
        ui->label_result_tmp->setPixmap(QPixmap(":/new/Icons/pass_icon.png"));
    }
    else if(result == FAIL)
    {
        err = NNO_GetSpecificErrorcode(NNO_error_code_tmp);
        NNO_TMP_GetErrorMessageFromErrorCode(err, errorString);
        NNO_ClearSpecificError(NNO_error_code_tmp);
        ui->label_remark_tmp->setText(errorString);
        ui->label_result_tmp->setPixmap(QPixmap(":/new/Icons/fail_icon.png"));
    }
    else
    {
        ui->label_remark_tmp->setText("READ FAIL");
        ui->label_result_tmp->setPixmap(QPixmap(":/new/Icons/fail_icon.png"));
    }
}

void MainWindow::on_pushButton_readSensors_clicked()
/**
 * This is a handler function for pushButton_readSensors  on Test Tab clicked() event
 * Calls the API functions to read the Temperatures(ambient, TIVA, Detector) Humidity and Voltage
 * Displays the values in the corresponding labels
 *
 */
{
    int	result;
    uint32 pBatt_Volt, pHumidity;
    int pAmbient_Temp, pDetector_Temp, pHDC_Temp, pTiva_Temp;
    uint32 red , green , blue;
    QString labelTextGreen;

    float  pfBatt_Volt, pfAmbient_Temp, pfDetector_Temp, pfHumidity, pfHDC_Temp, pfTiva_Temp;

    if ( ( result = NNO_ReadBattVolt( &pBatt_Volt ) ) == PASS )
    {
        pfBatt_Volt = pBatt_Volt / 100.0;
        ui->label_batt_volt->setText( QString("%1 V").arg(QString::number(pfBatt_Volt, 'f', 1)));
    }
    else
    {
        ui->label_hum->setText( "USB Error");
    }

    if ( ( result = NNO_ReadTemp( &pAmbient_Temp, &pDetector_Temp  ) ) == PASS )
    {
        pfAmbient_Temp = pAmbient_Temp / 100.0;
        pfDetector_Temp = pDetector_Temp / 100.0;
        ui->label_ambient_temp->setText( QString("%1 C").arg(QString::number(pfAmbient_Temp, 'f', 2)));
        ui->label_detect_temp->setText( QString("%1 C").arg(QString::number(pfDetector_Temp, 'f', 2)));
    }
    else
    {
        ui->label_hum->setText( "USB Error");
    }

    if ( ( result = NNO_ReadHum( &pHumidity, &pHDC_Temp  ) ) == PASS )
    {
        pfHumidity = pHumidity / 100.0;
        pfHDC_Temp = pHDC_Temp / 100.0;
        ui->label_hum->setText( QString("%1 %").arg(QString::number(pfHumidity, 'f', 2)));
        ui->label_hdc_temp->setText( QString("%1 C").arg(QString::number(pfHDC_Temp, 'f', 2)));
    }
    else
    {
        ui->label_hum->setText( "USB Error");
    }

    if ( ( result = NNO_ReadTivaTemp( &pTiva_Temp  ) ) == PASS )
    {
        pfTiva_Temp = pTiva_Temp / 100.0;
        ui->label_tiva_temp->setText( QString("%1 C").arg(QString::number(pfTiva_Temp, 'f', 2)));
    }
    else
    {
        ui->label_hum->setText( "USB Error");
    }

    if ( ( result = NNO_GetPhotoDetector(&red, &green , &blue ) ) == PASS )
    {
        labelTextGreen.sprintf("%d",green);
        ui->label_grren_val->setText(labelTextGreen);
    }
    else
    {
        ui->label_hum->setText( "USB Error");
    }
}

void MainWindow::on_pushButton_test_hdc_clicked()
/**
 * This is a handler function for pushButton_test_hdc on Test Tab clicked() event
 * Calls the API Test HDC function and updates the remark and result labels to show pass/fail
 * @return pErrorString returned from API function call
 *
 */
{
    uint16_t err;

    ui->label_result_hdc->clear();
    ui->label_remark_hdc->clear();

    NNO_ClearSpecificError(NNO_error_code_hdc);
    int result = NNO_TestHDC();
    if(result == PASS)
    {
        ui->label_remark_hdc->setText("PASS");
        ui->label_result_hdc->setPixmap(QPixmap(":/new/Icons/pass_icon.png"));
    }
    else if(result == FAIL)
    {
        err = NNO_GetSpecificErrorcode(NNO_error_code_hdc);
        NNO_HDC_GetErrorMessageFromErrorCode(err, errorString);
        NNO_ClearSpecificError(NNO_error_code_hdc);
        ui->label_remark_hdc->setText(errorString);
        ui->label_result_hdc->setPixmap(QPixmap(":/new/Icons/fail_icon.png"));
    }
    else
    {
        ui->label_remark_hdc->setText("READ FAIL");
        ui->label_result_hdc->setPixmap(QPixmap(":/new/Icons/fail_icon.png"));
    }
}

int MainWindow::on_pushButton_test_bt_clicked()
/**
 * This is a handler function for pushButton_test_hdc on Test Tab clicked() event
 * Calls the API Test HDC function and updates the remark and result labels to show pass/fail
 * @return 0 = PASS
 *         -1 = FAIL
 *
 */
{
    int result;

    g_BluetoothTest = true;
    ui->label_result_bt->clear();
    ui->label_remark_bt->clear();

    result = NNO_TestBT(true);
    if(result == FAIL)
    {
        ui->label_remark_bt->setText("USB COMM FAIL");
        ui->label_result_bt->setPixmap(QPixmap(":/new/Icons/fail_icon.png"));
        return result;
    }

    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Bluetooth Test", "Is the blue LED on and LightBlue App on iOS device shows NIRscan Nano and advertises data?",
            QMessageBox::Yes|QMessageBox::No);

    result = NNO_TestBT(false);
    if(result == FAIL)
    {
        ui->label_remark_bt->setText("USB COMM FAIL");
        ui->label_result_bt->setPixmap(QPixmap(":/new/Icons/fail_icon.png"));
        return result;
    }

    if (reply == QMessageBox::Yes)
    {
        ui->label_remark_bt->setText("PASS");
        ui->label_result_bt->setPixmap(QPixmap(":/new/Icons/pass_icon.png"));
        result = PASS;
    } else
    {
        ui->label_remark_bt->setText("FAIL");
        ui->label_result_bt->setPixmap(QPixmap(":/new/Icons/fail_icon.png"));
        result = FAIL;
    }

    g_BluetoothTest = false;
    return result;
}


int MainWindow::on_pushButton_test_sdc_clicked()
/**
 * This is a handler function for pushButton_test_sdc on Test Tab clicked() event
 * Calls the API Test SDC function to test the SD Card and updates the remark and result labels to show pass/fail
 * @return 0 = PASS
 *         -1 = FAIL
 *
 */
{
    int result;
    int16_t err;

    ui->label_result_sdc->clear();
    ui->label_remark_sdc->clear();

    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Test SD Card", "Insert SD Card into NIRscan Nano",
            QMessageBox::Yes|QMessageBox::No);

    if (reply == QMessageBox::No)
    {
        ui->label_remark_sdc->setText("NO CARD");
        ui->label_result_sdc->setPixmap(QPixmap(":/new/Icons/fail_icon.png"));
        result = FAIL;
    }
    else
    {
        NNO_ClearSpecificError(NNO_error_code_sd);
        result = NNO_TestSDC();

        if(result == PASS)
        {
            ui->label_remark_sdc->setText("PASS");
            ui->label_result_sdc->setPixmap(QPixmap(":/new/Icons/pass_icon.png"));
        }
        else if(result == FAIL)
        {
            err = NNO_GetSpecificErrorcode(NNO_error_code_sd);
            NNO_SDC_GetErrorMessageFromErrorCode(err, errorString);
            NNO_ClearSpecificError(NNO_error_code_sd);
            ui->label_remark_sdc->setText(errorString);
            ui->label_result_sdc->setPixmap(QPixmap(":/new/Icons/fail_icon.png"));
        }
        else
        {
            ui->label_remark_sdc->setText("READ FAIL");
            ui->label_result_sdc->setPixmap(QPixmap(":/new/Icons/fail_icon.png"));
        }
    }
    return result;
}


int MainWindow::on_pushButton_test_battery_clicked()
/**
 * This is a handler function for pushButton_test_battery on Test Tab clicked() event
 * Calls the API Test battery function to test the batteryand updates the remark and result labels to show pass/fail
 * @return 0 = PASS
 *         -1 = FAIL
 *
 */
{
    int status, batt_volt, usb_det, charge_curr, ts_fault, result;

    if((result = NNO_TestBQ(&status, &batt_volt, &usb_det, &charge_curr, &ts_fault)) != PASS)
    {
        return result;
    }
    switch ( status & 0x30 ) {
        case 0x0:
            ui->label_bq_status_2->setText("Ready");
            break;
        case 0x10:
            ui->label_bq_status_2->setText("Charging Battery");
            break;
        case 0x20:
            ui->label_bq_status_2->setText("Battery Charged");
            break;
        case 0x30:
            switch( status & 0x0F  ) {
                case 0:
                    ui->label_bq_status_2->setText("Normal");
                    break;
                case 1:
                    ui->label_bq_status_2->setText("Input Overvoltage");
                    break;
                case 2:
                    ui->label_bq_status_2->setText("Input Undervoltage");
                    break;
                case 3:
                    ui->label_bq_status_2->setText("Sleep");
                    break;
                case 4:
                    ui->label_bq_status_2->setText("No Thermistor Detected");
                    break;
                case 5:
                    ui->label_bq_status_2->setText("Battery Overvoltage");
                    break;
                case 6:
                    ui->label_bq_status_2->setText("Thermal Shutdown");
                    break;
                case 7:
                    ui->label_bq_status_2->setText("Timer Fault");
                    break;
                case 8:
                    ui->label_bq_status_2->setText("No Battery Detected");
                    break;
                case 9:
                    ui->label_bq_status_2->setText("ISET Short");
                    break;
                case 10:
                    ui->label_bq_status_2->setText("Input Fault and LDO low");
                    break;
            }
            break;
    }

    switch( charge_curr  ) {
        case 31:
            ui->label_bq_charge_current_2->setText("1.0 A");
            break;
        case 30:
            ui->label_bq_charge_current_2->setText("2.0 A");
            break;
        case 29:
            ui->label_bq_charge_current_2->setText("1.95 A");
            break;
        case 28:
            ui->label_bq_charge_current_2->setText("1.90 A");
            break;
        case 27:
            ui->label_bq_charge_current_2->setText("1.85 A");
            break;
        case 26:
            ui->label_bq_charge_current_2->setText("1.80 A");
            break;
        case 25:
            ui->label_bq_charge_current_2->setText("1.75 A");
            break;
        case 24:
            ui->label_bq_charge_current_2->setText("1.70 A");
            break;
        case 23:
            ui->label_bq_charge_current_2->setText("1.65 A");
            break;
        case 22:
            ui->label_bq_charge_current_2->setText("1.60 A");
            break;
        case 21:
            ui->label_bq_charge_current_2->setText("1.55 A");
            break;
        case 20:
            ui->label_bq_charge_current_2->setText("1.50 A");
            break;
        case 19:
            ui->label_bq_charge_current_2->setText("1.45 A");
            break;
        case 18:
            ui->label_bq_charge_current_2->setText("1.40 A");
            break;
        case 17:
            ui->label_bq_charge_current_2->setText("1.35 A");
            break;
        case 16:
            ui->label_bq_charge_current_2->setText("1.30 A");
            break;
        case 15:
            ui->label_bq_charge_current_2->setText("1.25 A");
            break;
        case 14:
            ui->label_bq_charge_current_2->setText("1.20 A");
            break;
        case 13:
            ui->label_bq_charge_current_2->setText("1.15 A");
            break;
        case 12:
            ui->label_bq_charge_current_2->setText("1.10 A");
            break;
        case 11:
            ui->label_bq_charge_current_2->setText("1.05 A");
            break;
        case 10:
            ui->label_bq_charge_current_2->setText("1.00 A");
            break;
        case 9:
            ui->label_bq_charge_current_2->setText("0.95 A");
            break;
        case 8:
            ui->label_bq_charge_current_2->setText("0.90 A");
            break;
        case 7:
            ui->label_bq_charge_current_2->setText("0.85 A");
            break;
        case 6:
            ui->label_bq_charge_current_2->setText("0.80 A");
            break;
        case 5:
            ui->label_bq_charge_current_2->setText("0.75 A");
            break;
        case 4:
            ui->label_bq_charge_current_2->setText("0.70 A");
            break;
        case 3:
            ui->label_bq_charge_current_2->setText("0.65 A");
            break;
        case 2:
            ui->label_bq_charge_current_2->setText("0.60 A");
            break;
        case 1:
            ui->label_bq_charge_current_2->setText("0.55 A");
            break;
    }

    return result;
}

int MainWindow::on_pushButton_test_bq_clicked()
{
    int status, batt_volt, usb_det, charge_curr, ts_fault, result;

    if((result = NNO_TestBQ(&status, &batt_volt, &usb_det, &charge_curr, &ts_fault)) != PASS)
    {
        ui->label_bq_status->setText("READ FAIL");
        return result;
    }
    switch ( status & 0x30 ) {
        case 0x0:
            ui->label_bq_status->setText("Ready");
            break;
        case 0x10:
            ui->label_bq_status->setText("Charging");
            break;
        case 0x20:
            ui->label_bq_status->setText("Charge Done");
            break;
        case 0x30:
            ui->label_bq_status->setText("Fault");
            break;
    }

    switch( status & 0x0F  ) {
        case 0:
            ui->label_bq_fault->setText("PASS");
            break;
        case 1:
            ui->label_bq_fault->setText("Input Overvoltage");
            break;
        case 2:
            ui->label_bq_fault->setText("Input Undervoltage");
            break;
        case 3:
            ui->label_bq_fault->setText("Sleep");
            break;
        case 4:
            ui->label_bq_fault->setText("Battery Temp Fault");
            break;
        case 5:
            ui->label_bq_fault->setText("Battery Overvoltage");
            break;
        case 6:
            ui->label_bq_fault->setText("Thermal Shutdown");
            break;
        case 7:
            ui->label_bq_fault->setText("Timer");
            break;
        case 8:
            ui->label_bq_fault->setText("No Battery");
            break;
        case 9:
            ui->label_bq_fault->setText("ISET Short");
            break;
        case 10:
            ui->label_bq_fault->setText("Input Fault and LDO low");
            break;
    }

    switch( usb_det ) {
        case 0:
            ui->label_bq_curr_limit->setText("500 mA");
            ui->label_bq_in_thershold->setText("4.36 V");
            break;
        case 1:
            ui->label_bq_curr_limit->setText("1.0 A");
            ui->label_bq_in_thershold->setText("4.27 V");
            break;
        case 2:
            ui->label_bq_curr_limit->setText("100 mA");
            ui->label_bq_in_thershold->setText("4.36 V");
            break;
        case 3:
            ui->label_bq_curr_limit->setText("Hi-Z");
            ui->label_bq_in_thershold->setText("None");
            break;
    }
    switch( batt_volt ) {
        case 63:
            ui->label_batt_voltage->setText("4.76 V");
            break;
        case 62:
            ui->label_batt_voltage->setText("4.74 V");
            break;
        case 61:
            ui->label_batt_voltage->setText("4.72 V");
            break;
        case 60:
            ui->label_batt_voltage->setText("4.70 V");
            break;
        case 59:
            ui->label_batt_voltage->setText("4.68 V");
            break;
        case 58:
            ui->label_batt_voltage->setText("4.66 V");
            break;
        case 57:
            ui->label_batt_voltage->setText("4.64 V");
            break;
        case 56:
            ui->label_batt_voltage->setText("4.62 V");
            break;
        case 55:
            ui->label_batt_voltage->setText("4.60 V");
            break;
        case 54:
            ui->label_batt_voltage->setText("4.58 V");
            break;
        case 53:
            ui->label_batt_voltage->setText("4.56 V");
            break;
        case 52:
            ui->label_batt_voltage->setText("4.54 V");
            break;
        case 51:
            ui->label_batt_voltage->setText("4.52 V");
            break;
        case 50:
            ui->label_batt_voltage->setText("4.50 V");
            break;
        case 49:
            ui->label_batt_voltage->setText("4.48 V");
            break;
        case 48:
            ui->label_batt_voltage->setText("4.46 V");
            break;
        case 47:
            ui->label_batt_voltage->setText("4.44 V");
            break;
        case 46:
            ui->label_batt_voltage->setText("4.42 V");
            break;
        case 45:
            ui->label_batt_voltage->setText("4.40 V");
            break;
        case 44:
            ui->label_batt_voltage->setText("4.38 V");
            break;
        case 43:
            ui->label_batt_voltage->setText("4.36 V");
            break;
        case 42:
            ui->label_batt_voltage->setText("4.34 V");
            break;
        case 41:
            ui->label_batt_voltage->setText("4.32 V");
            break;
        case 40:
            ui->label_batt_voltage->setText("4.30 V");
            break;
        case 39:
            ui->label_batt_voltage->setText("4.28 V");
            break;
        case 38:
            ui->label_batt_voltage->setText("4.26 V");
            break;
        case 37:
            ui->label_batt_voltage->setText("4.24 V");
            break;
        case 36:
            ui->label_batt_voltage->setText("4.22 V");
            break;
        case 35:
            ui->label_batt_voltage->setText("4.20 V");
            break;
        case 34:
            ui->label_batt_voltage->setText("4.18 V");
            break;
        case 33:
            ui->label_batt_voltage->setText("4.16 V");
            break;
        case 32:
            ui->label_batt_voltage->setText("4.14 V");
            break;
        case 31:
            ui->label_batt_voltage->setText("4.12 V");
            break;
        case 30:
            ui->label_batt_voltage->setText("4.10 V");
            break;
        case 29:
            ui->label_batt_voltage->setText("4.08 V");
            break;
        case 28:
            ui->label_batt_voltage->setText("4.06 V");
            break;
        case 27:
            ui->label_batt_voltage->setText("4.04 V");
            break;
        case 26:
            ui->label_batt_voltage->setText("4.02 V");
            break;
        case 25:
            ui->label_batt_voltage->setText("4.00 V");
            break;
        case 24:
            ui->label_batt_voltage->setText("3.98 V");
            break;
        case 23:
            ui->label_batt_voltage->setText("3.96 V");
            break;
        case 22:
            ui->label_batt_voltage->setText("3.94 V");
            break;
        case 21:
            ui->label_batt_voltage->setText("3.92 V");
            break;
        case 20:
            ui->label_batt_voltage->setText("3.90 V");
            break;
        case 19:
            ui->label_batt_voltage->setText("3.88 V");
            break;
        case 18:
            ui->label_batt_voltage->setText("3.86 V");
            break;
        case 17:
            ui->label_batt_voltage->setText("3.84 V");
            break;
        case 16:
            ui->label_batt_voltage->setText("3.82 V");
            break;
        case 15:
            ui->label_batt_voltage->setText("3.80 V");
            break;
        case 14:
            ui->label_batt_voltage->setText("3.78 V");
            break;
        case 13:
            ui->label_batt_voltage->setText("3.76 V");
            break;
        case 12:
            ui->label_batt_voltage->setText("3.74 V");
            break;
        case 11:
            ui->label_batt_voltage->setText("3.72 V");
            break;
        case 10:
            ui->label_batt_voltage->setText("3.70 V");
            break;
        case 9:
            ui->label_batt_voltage->setText("3.68 V");
            break;
        case 8:
            ui->label_batt_voltage->setText("3.66 V");
            break;
        case 7:
            ui->label_batt_voltage->setText("3.64 V");
            break;
        case 6:
            ui->label_batt_voltage->setText("3.62 V");
            break;
        case 5:
            ui->label_batt_voltage->setText("3.60 V");
            break;
        case 4:
            ui->label_batt_voltage->setText("3.58 V");
            break;
        case 3:
            ui->label_batt_voltage->setText("3.56 V");
            break;
        case 2:
            ui->label_batt_voltage->setText("3.54 V");
            break;
        case 1:
            ui->label_batt_voltage->setText("3.52 V");
            break;
    }

    switch( charge_curr  ) {
        case 31:
            ui->label_bq_charge_current->setText("1.0 A");
            break;
        case 30:
            ui->label_bq_charge_current->setText("2.0 A");
            break;
        case 29:
            ui->label_bq_charge_current->setText("1.95 A");
            break;
        case 28:
            ui->label_bq_charge_current->setText("1.90 A");
            break;
        case 27:
            ui->label_bq_charge_current->setText("1.85 A");
            break;
        case 26:
            ui->label_bq_charge_current->setText("1.80 A");
            break;
        case 25:
            ui->label_bq_charge_current->setText("1.75 A");
            break;
        case 24:
            ui->label_bq_charge_current->setText("1.70 A");
            break;
        case 23:
            ui->label_bq_charge_current->setText("1.65 A");
            break;
        case 22:
            ui->label_bq_charge_current->setText("1.60 A");
            break;
        case 21:
            ui->label_bq_charge_current->setText("1.55 A");
            break;
        case 20:
            ui->label_bq_charge_current->setText("1.50 A");
            break;
        case 19:
            ui->label_bq_charge_current->setText("1.45 A");
            break;
        case 18:
            ui->label_bq_charge_current->setText("1.40 A");
            break;
        case 17:
            ui->label_bq_charge_current->setText("1.35 A");
            break;
        case 16:
            ui->label_bq_charge_current->setText("1.30 A");
            break;
        case 15:
            ui->label_bq_charge_current->setText("1.25 A");
            break;
        case 14:
            ui->label_bq_charge_current->setText("1.20 A");
            break;
        case 13:
            ui->label_bq_charge_current->setText("1.15 A");
            break;
        case 12:
            ui->label_bq_charge_current->setText("1.10 A");
            break;
        case 11:
            ui->label_bq_charge_current->setText("1.05 A");
            break;
        case 10:
            ui->label_bq_charge_current->setText("1.00 A");
            break;
        case 9:
            ui->label_bq_charge_current->setText("0.95 A");
            break;
        case 8:
            ui->label_bq_charge_current->setText("0.90 A");
            break;
        case 7:
            ui->label_bq_charge_current->setText("0.85 A");
            break;
        case 6:
            ui->label_bq_charge_current->setText("0.80 A");
            break;
        case 5:
            ui->label_bq_charge_current->setText("0.75 A");
            break;
        case 4:
            ui->label_bq_charge_current->setText("0.70 A");
            break;
        case 3:
            ui->label_bq_charge_current->setText("0.65 A");
            break;
        case 2:
            ui->label_bq_charge_current->setText("0.60 A");
            break;
        case 1:
            ui->label_bq_charge_current->setText("0.55 A");
            break;
    }
    switch( ts_fault  ) {
        case 7:
            ui->label_bq_ts_fault->setText("No Thermistor");
            break;
        case 6:
            ui->label_bq_ts_fault->setText("Deep Freeze Temp");
            break;
        case 5:
            ui->label_bq_ts_fault->setText("Freeze Temp");
            break;
        case 4:
            ui->label_bq_ts_fault->setText("Cold Temp");
            break;
        case 3:
            ui->label_bq_ts_fault->setText("Cool Temp");
            break;
        case 2:
            ui->label_bq_ts_fault->setText("Warm Temp");
            break;
        case 1:
            ui->label_bq_ts_fault->setText("High Temp");
            break;
        case 0:
            ui->label_bq_ts_fault->setText("No Fault");
            break;
    }

    return result;
}

int MainWindow::on_pushButton_test_sdram_clicked()

/**
 * This is a handler function for pushButton_test_sdram on Test Tab clicked() event
 * Calls the API Test SDRam function to test the SD RAM and updates the remark and result labels to show pass/fail
 * @return 0 = PASS
 *         -1 = FAIL
 *
 */
{

    ui->label_result_sdram->clear();
    ui->label_remark_sdram->clear();
    QApplication::processEvents();

    int result = NNO_TestSDRAM();

    if(result == PASS)
    {
        ui->label_remark_sdram->setText("PASS");
        ui->label_result_sdram->setPixmap(QPixmap(":/new/Icons/pass_icon.png"));
    }
    else  if(result == FAIL)
    {
        ui->label_remark_sdram->setText("FAIL");
        ui->label_result_sdram->setPixmap(QPixmap(":/new/Icons/fail_icon.png"));
    }
    else
    {
        ui->label_remark_sdram->setText("READ FAIL");
        ui->label_result_sdram->setPixmap(QPixmap(":/new/Icons/fail_icon.png"));
    }

    return result;
}


int MainWindow::on_pushButton_test_dlpc150_clicked()
/**
 * This is a handler function for pushButton_test_dlpc on Test Tab clicked() event
 * Calls the API Test DLPC function to test the SD RAM and updates the remark and result labels to show pass/fail
 * @return 0 = PASS
 *         -1 = FAIL
 *
 */
{
    int result = NNO_DLPCEnable(true, true);

    if(result == FAIL)
    {
        ui->label_remark_dlpc150->setText("USB COMM FAIL");
        ui->label_result_dlpc150->setPixmap(QPixmap(":/new/Icons/fail_icon.png"));
        return result;
    }

    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "DLPC150 Test", "Is the lamp glowing?",
            QMessageBox::Yes|QMessageBox::No);

    NNO_DLPCEnable(false, false);

    if (reply == QMessageBox::Yes)
    {
        ui->label_remark_dlpc150->setText("PASS");
        ui->label_result_dlpc150->setPixmap(QPixmap(":/new/Icons/pass_icon.png"));
        result = PASS;
    } else
    {
        ui->label_remark_dlpc150->setText("FAIL");
        ui->label_result_dlpc150->setPixmap(QPixmap(":/new/Icons/fail_icon.png"));
        result = FAIL;
    }

    return result;
}

int MainWindow::on_pushButton_test_led_clicked()
/**
 *This function is a handler for pushButton_test_led on Test tab, clicked() event
 *Calls the corresponding API function and displays the result in respective GUI labels
 *@return 0 = PASS
 *         -1 = FAIL
 */
{
    int result = NNO_TestLED(true);
    ui->label_result_led->clear();
    ui->label_remark_led->clear();

    if(result == FAIL)
    {
        ui->label_remark_led->setText("USB COMM FAIL");
        ui->label_result_led->setPixmap(QPixmap(":/new/Icons/fail_icon.png"));
        return result;
    }

    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "LED Test", "Are the Green, Yellow, and Blue LEDs on?",
                                  QMessageBox::Yes|QMessageBox::No);

    NNO_TestLED(false);

    if (reply == QMessageBox::Yes)
    {
        ui->label_remark_led->setText("PASS");
        ui->label_result_led->setPixmap(QPixmap(":/new/Icons/pass_icon.png"));
        result = PASS;
    } else
    {
        ui->label_remark_led->setText("FAIL");
        ui->label_result_led->setPixmap(QPixmap(":/new/Icons/fail_icon.png"));
        result = FAIL;
    }

    return result;
}

void MainWindow::on_pushButton_runAll_tests_Tiva_clicked()
/**
 *This function is a handler for pushButton_runAll_tests_TIVA on Test tab, clicked() event
 *Sequentially runs all the TIVA tests on EVM and saves the results in SelfTest_TIVA* file
 */
{
    if(ui->lineEdit_testname->text().isEmpty())
    {
        showError("Empty PCBA S/N. Please enter S/N!");
        return;
    }

    QFile testResultsFile(filepath.GettestTivaFileName());
    testResultsFile.open(QIODevice::ReadWrite | QIODevice::Text);
    QTextStream out(&testResultsFile);
    long long percent_completion;
    int result;
    QString savedData = out.readAll();

    if(savedData == NULL)
    {
        out << "PCBA S/N,EEPROM Test,SDRAM Test,HDC Test,";
        out << "BQ Status, BQ Fault, BQ Thermistor Fault,BQ Battery Regulation Voltage,";
        out << "BQ Input Current Limit, BQ Input Threshold,BQ Charge Current,";
        out << "Bluetooth test,SD Card,LED Test\n";
    }
    else
    {
        out << "\n";
    }

    //Log PCBA Serial number to the test results file
    out << ui->lineEdit_testname->text() << ",";

    clearTivaTestResults();
    ui->progressbar_runTests_Tiva->setValue(0);

    ui->label_runtests_progress_Tiva->setText("Running EEPROM Test");
    QApplication::processEvents();

    result = on_pushButton_test_eeprom_clicked();
    if(result == PASS)
    {
        out << "PASS,";
    }
    else if(result == FAIL)
    {
        out << "FAIL,";
    }
    else
    {
        out << "READ FAIL,";
    }
    percent_completion = 15;
    ui->progressbar_runTests_Tiva->setValue(percent_completion);

    ui->label_runtests_progress_Tiva->setText("Running SDRAM Test");
    QApplication::processEvents();

    result = on_pushButton_test_sdram_clicked();
    if(result == PASS)
    {
        out << "PASS,";
    }
    else if(result == FAIL)
    {
        out << "FAIL,";
    }
    else
    {
        out << "READ FAIL,";
    }
    percent_completion = 30;
    ui->progressbar_runTests_Tiva->setValue(percent_completion);

    ui->label_runtests_progress_Tiva->setText("Running HDC Test");
    QApplication::processEvents();

    on_pushButton_test_hdc_clicked();
    out << errorString;
    out << ",";
    percent_completion = 45;
    ui->progressbar_runTests_Tiva->setValue(percent_completion);

    ui->label_runtests_progress_Tiva->setText("Running Battery Test");
    QApplication::processEvents();

    int status, batt_volt, usb_det, charge_curr, ts_fault;

    if(NNO_TestBQ(&status, &batt_volt, &usb_det, &charge_curr, &ts_fault) != PASS)
    {
        ui->label_runtests_progress_Tiva->setText("READ FAIL");
        return;
    }
    else
        ui->label_runtests_progress_Tiva->setText("PASS");

    switch ( status & 0x30 ) {
    case 0x00:
        ui->label_bq_status->setText("Ready");
        out << "Ready,";
        break;
    case 0x10:
        ui->label_bq_status->setText("Charging");
        out << "Charging,";
        break;
    case 0x20:
        ui->label_bq_status->setText("Charge Done");
        out << "Charge Done,";
        break;
    case 0x30:
        ui->label_bq_status->setText("Fault");
        out << "Fault,";
        break;
    }

    switch( status & 0x0F  ) {
    case 0:
        ui->label_bq_fault->setText("PASS");
        out << "No Fault,";
        break;
    case 1:
        ui->label_bq_fault->setText("Input Overvoltage");
        out << "Input Overvoltage,";
        break;
    case 2:
        ui->label_bq_fault->setText("Input Undervoltage");
        out << "Input Undervoltage,";
        break;
    case 3:
        ui->label_bq_fault->setText("Sleep");
        out << "Sleep,";
        break;
    case 4:
        ui->label_bq_fault->setText("Battery Temp Fault");
        out << "Battery Temp Fault,";
        break;
    case 5:
        ui->label_bq_fault->setText("Battery Overvoltage");
        out << "Battery Overvoltage,";
        break;
    case 6:
        ui->label_bq_fault->setText("Thermal Shutdown");
        out << "Thermal Shutdown,";
        break;
    case 7:
        ui->label_bq_fault->setText("Timer");
        out << "Timer,";
        break;
    case 8:
        ui->label_bq_fault->setText("No Battery");
        out << "No Battery,";
        break;
    case 9:
        ui->label_bq_fault->setText("ISET Short");
        out << "ISET Short,";
        break;
    case 10:
        ui->label_bq_fault->setText("Input Fault and LDO low");
        out << "Input Fault and LDO low,";
        break;
    }

    switch( ts_fault  ) {
    case 7:
        ui->label_bq_ts_fault->setText("No Thermistor");
        out << "No Thermistor,";
        break;
    case 6:
        ui->label_bq_ts_fault->setText("Deep Freeze Temp");
        out << "Deep Freeze Temp,";
        break;
    case 5:
        ui->label_bq_ts_fault->setText("Freeze Temp");
        out << "Freeze Temp,";
        break;
    case 4:
        ui->label_bq_ts_fault->setText("Cold Temp");
        out << "Cold Temp,";
        break;
    case 3:
        ui->label_bq_ts_fault->setText("Cool Temp");
        out << "Cool Temp,";
        break;
    case 2:
        ui->label_bq_ts_fault->setText("Warm Temp");
        out << "Warm Temp,";
        break;
    case 1:
        ui->label_bq_ts_fault->setText("High Temp");
        out << "High Temp,";
        break;
    case 0:
        ui->label_bq_ts_fault->setText("No Fault");
        out << "No Fault,";
        break;
    }

    switch( batt_volt ) {
    case 63:
        ui->label_batt_voltage->setText("4.76 V");
        out << "4.76 V,";
        break;
    case 62:
        ui->label_batt_voltage->setText("4.74 V");
        out << "4.74 V,";
        break;
    case 61:
        ui->label_batt_voltage->setText("4.72 V");
        out << "4.72 V,";
        break;
    case 60:
        ui->label_batt_voltage->setText("4.70 V");
        out << "4.70 V,";
        break;
    case 59:
        ui->label_batt_voltage->setText("4.68 V");
        out << "4.68 V,";
        break;
    case 58:
        ui->label_batt_voltage->setText("4.66 V");
        out << "4.66 V,";
        break;
    case 57:
        ui->label_batt_voltage->setText("4.64 V");
        out << "4.64 V,";
        break;
    case 56:
        ui->label_batt_voltage->setText("4.62 V");
        out << "4.62 V,";
        break;
    case 55:
        ui->label_batt_voltage->setText("4.60 V");
        out << "4.60 V,";
        break;
    case 54:
        ui->label_batt_voltage->setText("4.58 V");
        out << "4.58 V,";
        break;
    case 53:
        ui->label_batt_voltage->setText("4.56 V");
        out << "4.56 V,";
        break;
    case 52:
        ui->label_batt_voltage->setText("4.54 V");
        out << "4.54 V,";
        break;
    case 51:
        ui->label_batt_voltage->setText("4.52 V");
        out << "4.52 V,";
        break;
    case 50:
        ui->label_batt_voltage->setText("4.50 V");
        out << "4.50 V,";
        break;
    case 49:
        ui->label_batt_voltage->setText("4.48 V");
        out << "4.48 V,";
        break;
    case 48:
        ui->label_batt_voltage->setText("4.46 V");
        out << "4.46 V,";
        break;
    case 47:
        ui->label_batt_voltage->setText("4.44 V");
        out << "4.44 V,";
        break;
    case 46:
        ui->label_batt_voltage->setText("4.42 V");
        out << "4.42 V,";
        break;
    case 45:
        ui->label_batt_voltage->setText("4.40 V");
        out << "4.40 V,";
        break;
    case 44:
        ui->label_batt_voltage->setText("4.38 V");
        out << "4.38 V,";
        break;
    case 43:
        ui->label_batt_voltage->setText("4.36 V");
        out << "4.36 V,";
        break;
    case 42:
        ui->label_batt_voltage->setText("4.34 V");
        out << "4.34 V,";
        break;
    case 41:
        ui->label_batt_voltage->setText("4.32 V");
        out << "4.32 V,";
        break;
    case 40:
        ui->label_batt_voltage->setText("4.30 V");
        out << "4.30 V,";
        break;
    case 39:
        ui->label_batt_voltage->setText("4.28 V");
        out << "4.28 V,";
        break;
    case 38:
        ui->label_batt_voltage->setText("4.26 V");
        out << "4.26 V,";
        break;
    case 37:
        ui->label_batt_voltage->setText("4.24 V");
        out << "4.24 V,";
        break;
    case 36:
        ui->label_batt_voltage->setText("4.22 V");
        out << "4.22 V,";
        break;
    case 35:
        ui->label_batt_voltage->setText("4.20 V");
        out << "4.20 V,";
        break;
    case 34:
        ui->label_batt_voltage->setText("4.18 V");
        out << "4.18 V,";
        break;
    case 33:
        ui->label_batt_voltage->setText("4.16 V");
        out << "4.16 V,";
        break;
    case 32:
        ui->label_batt_voltage->setText("4.14 V");
        out << "4.14 V,";
        break;
    case 31:
        ui->label_batt_voltage->setText("4.12 V");
        out << "4.12 V,";
        break;
    case 30:
        ui->label_batt_voltage->setText("4.10 V");
        out << "4.10 V,";
        break;
    case 29:
        ui->label_batt_voltage->setText("4.08 V");
        out << "4.08 V,";
        break;
    case 28:
        ui->label_batt_voltage->setText("4.06 V");
        out << "4.06 V,";
        break;
    case 27:
        ui->label_batt_voltage->setText("4.04 V");
        out << "4.04 V,";
        break;
    case 26:
        ui->label_batt_voltage->setText("4.02 V");
        out << "4.02 V,";
        break;
    case 25:
        ui->label_batt_voltage->setText("4.00 V");
        out << "4.00 V,";
        break;
    case 24:
        ui->label_batt_voltage->setText("3.98 V");
        out << "3.98 V,";
        break;
    case 23:
        ui->label_batt_voltage->setText("3.96 V");
        out << "3.96 V,";
        break;
    case 22:
        ui->label_batt_voltage->setText("3.94 V");
        out << "3.94 V,";
        break;
    case 21:
        ui->label_batt_voltage->setText("3.92 V");
        out << "3.92 V,";
        break;
    case 20:
        ui->label_batt_voltage->setText("3.90 V");
        out << "3.90 V,";
        break;
    case 19:
        ui->label_batt_voltage->setText("3.88 V");
        out << "3.88 V,";
        break;
    case 18:
        ui->label_batt_voltage->setText("3.86 V");
        out << "3.86 V,";
        break;
    case 17:
        ui->label_batt_voltage->setText("3.84 V");
        out << "3.84 V,";
        break;
    case 16:
        ui->label_batt_voltage->setText("3.82 V");
        out << "3.82 V,";
        break;
    case 15:
        ui->label_batt_voltage->setText("3.80 V");
        out << "3.80 V,";
        break;
    case 14:
        ui->label_batt_voltage->setText("3.78 V");
        out << "3.78 V,";
        break;
    case 13:
        ui->label_batt_voltage->setText("3.76 V");
        out << "3.76 V,";
        break;
    case 12:
        ui->label_batt_voltage->setText("3.74 V");
        out << "3.74 V,";
        break;
    case 11:
        ui->label_batt_voltage->setText("3.72 V");
        out << "3.72 V,";
        break;
    case 10:
        ui->label_batt_voltage->setText("3.70 V");
        out << "3.70 V,";
        break;
    case 9:
        ui->label_batt_voltage->setText("3.68 V");
        out << "3.68 V,";
        break;
    case 8:
        ui->label_batt_voltage->setText("3.66 V");
        out << "3.66 V,";
        break;
    case 7:
        ui->label_batt_voltage->setText("3.64 V");
        out << "3.64 V,";
        break;
    case 6:
        ui->label_batt_voltage->setText("3.62 V");
        out << "3.62 V,";
        break;
    case 5:
        ui->label_batt_voltage->setText("3.60 V");
        out << "3.60 V,";
        break;
    case 4:
        ui->label_batt_voltage->setText("3.58 V");
        out << "3.58 V,";
        break;
    case 3:
        ui->label_batt_voltage->setText("3.56 V");
        out << "3.56 V,";
        break;
    case 2:
        ui->label_batt_voltage->setText("3.54 V");
        out << "3.54 V,";
        break;
    case 1:
        ui->label_batt_voltage->setText("3.52 V");
        out << "3.52 V,";
        break;
    }

    switch( usb_det ) {
    case 0:
        ui->label_bq_curr_limit->setText("500 mA");
        ui->label_bq_in_thershold->setText("4.36 V");
        out << "500 mA,";
        out << "4.36 V,";
        break;
    case 1:
        ui->label_bq_curr_limit->setText("1.0 A");
        ui->label_bq_in_thershold->setText("4.27 V");
        out << "1.0 A,";
        out << "4.27 V,";
        break;
    case 2:
        ui->label_bq_curr_limit->setText("100 mA");
        ui->label_bq_in_thershold->setText("4.36 V");
        out << "100 mA,";
        out << "4.36 V,";
        break;
    case 3:
        ui->label_bq_curr_limit->setText("Hi-Z");
        ui->label_bq_in_thershold->setText("None");
        out << "Hi-Z,";
        out << "None,";
        break;
    }

    switch( charge_curr  ) {
    case 31:
        ui->label_bq_charge_current->setText("1.0 A");
        out << "1.0 A,";
        break;
    case 30:
        ui->label_bq_charge_current->setText("2.0 A");
        out << "2.0 A,";
        break;
    case 29:
        ui->label_bq_charge_current->setText("1.95 A");
        out << "1.95 A,";
        break;
    case 28:
        ui->label_bq_charge_current->setText("1.90 A");
        out << "1.90 A,";
        break;
    case 27:
        ui->label_bq_charge_current->setText("1.85 A");
        out << "1.85 A,";
        break;
    case 26:
        ui->label_bq_charge_current->setText("1.80 A");
        out << "1.80 A,";
        break;
    case 25:
        ui->label_bq_charge_current->setText("1.75 A");
        out << "1.75 A,";
        break;
    case 24:
        ui->label_bq_charge_current->setText("1.70 A");
        out << "1.70 A,";
        break;
    case 23:
        ui->label_bq_charge_current->setText("1.65 A");
        out << "1.65 A,";
        break;
    case 22:
        ui->label_bq_charge_current->setText("1.60 A");
        out << "1.60 A,";
        break;
    case 21:
        ui->label_bq_charge_current->setText("1.55 A");
        out << "1.55 A,";
        break;
    case 20:
        ui->label_bq_charge_current->setText("1.50 A");
        out << "1.50 A,";
        break;
    case 19:
        ui->label_bq_charge_current->setText("1.45 A");
        out << "1.45 A,";
        break;
    case 18:
        ui->label_bq_charge_current->setText("1.40 A");
        out << "1.40 A,";
        break;
    case 17:
        ui->label_bq_charge_current->setText("1.35 A");
        out << "1.35 A,";
        break;
    case 16:
        ui->label_bq_charge_current->setText("1.30 A");
        out << "1.30 A,";
        break;
    case 15:
        ui->label_bq_charge_current->setText("1.25 A");
        out << "1.25 A,";
        break;
    case 14:
        ui->label_bq_charge_current->setText("1.20 A");
        out << "1.20 A,";
        break;
    case 13:
        ui->label_bq_charge_current->setText("1.15 A");
        out << "1.15 A,";
        break;
    case 12:
        ui->label_bq_charge_current->setText("1.10 A");
        out << "1.10 A,";
        break;
    case 11:
        ui->label_bq_charge_current->setText("1.05 A");
        out << "1.05 A,";
        break;
    case 10:
        ui->label_bq_charge_current->setText("1.00 A");
        out << "1.00 A,";
        break;
    case 9:
        ui->label_bq_charge_current->setText("0.95 A");
        out << "0.95 A,";
        break;
    case 8:
        ui->label_bq_charge_current->setText("0.90 A");
        out << "0.90 A,";
        break;
    case 7:
        ui->label_bq_charge_current->setText("0.85 A");
        out << "0.85 A,";
        break;
    case 6:
        ui->label_bq_charge_current->setText("0.80 A");
        out << "0.80 A,";
        break;
    case 5:
        ui->label_bq_charge_current->setText("0.75 A");
        out << "0.75 A,";
        break;
    case 4:
        ui->label_bq_charge_current->setText("0.70 A");
        out << "0.70 A,";
        break;
    case 3:
        ui->label_bq_charge_current->setText("0.65 A");
        out << "0.65 A,";
        break;
    case 2:
        ui->label_bq_charge_current->setText("0.60 A");
        out << "0.60 A,";
        break;
    case 1:
        ui->label_bq_charge_current->setText("0.55 A");
        out << "0.55 A,";
        break;
    }
    percent_completion = 60;
    ui->progressbar_runTests_Tiva->setValue(percent_completion);

    ui->label_runtests_progress_Tiva->setText("Running Bluetooth Test");
    QApplication::processEvents();

    result = on_pushButton_test_bt_clicked();
    if(result == PASS)
    {
        out << "PASS,";
    }
    else if(result == FAIL)
    {
        out << "FAIL,";
    }
    else
    {
        out << "READ FAIL,";
    }
    percent_completion = 70;
    ui->progressbar_runTests_Tiva->setValue(percent_completion);

    ui->label_runtests_progress_Tiva->setText("Running SD Card Test");
    QApplication::processEvents();

    result = on_pushButton_test_sdc_clicked();
    if(result == PASS)
    {
        out << "PASS,";
    }
    else if(result == FAIL)
    {
        out << "FAIL,";
    }
    else
    {
        out << "READ FAIL,";
    }
    percent_completion = 85;
    ui->progressbar_runTests_Tiva->setValue(percent_completion);

    ui->label_runtests_progress_Tiva->setText("Running LED Test");
    QApplication::processEvents();

    result = on_pushButton_test_led_clicked();
    if(result == PASS)
    {
        out << "PASS,";
    }
    else if(result == FAIL)
    {
        out << "FAIL,";
    }
    else
    {
        out << "READ FAIL,";
    }
    percent_completion = 100;
    ui->progressbar_runTests_Tiva->setValue(percent_completion);

    ui->label_runtests_progress_Tiva->setText("Results logged in file below");
    ui->lineEdit_test_results->setText(QString("%1").arg(filepath.GettestTivaFileName()));

    testResultsFile.close();

    NNO_ResetTiva();
}


void MainWindow::on_pushButton_runAll_tests_DLPC_clicked()
/**
 *This function is a handler for pushButton_runAll_tests_DLPC on Test tab, clicked() event
 *Sequentially runs all the DLPC tests on EVM and saves the results inSelfTest_DLPC* file
 */
{
    if(ui->lineEdit_testname->text().isEmpty())
    {
        showError("Empty PCBA S/N. Please enter S/N!");
        return;
    }

    QFile testResultsFile(filepath.GettestDLPCFileName());
    testResultsFile.open(QIODevice::ReadWrite | QIODevice::Text);
    QTextStream out(&testResultsFile);
    long long percent_completion;
    int result;
    QString savedData = out.readAll();

    if(savedData == NULL)
    {
        out << "PCBA S/N,DLPC150 Test\n";
    }
    else
    {
        out << "\n";
    }

    //Log PCBA Serial number to the test results file
    out << ui->lineEdit_testname->text() << ",";

    clearDLPCTestResults();
    ui->progressbar_runTests_DLPC->setValue(0);

    ui->label_runtests_progress_DLPC->setText("Running DLPC150 Test");
    QApplication::processEvents();

    result = on_pushButton_test_dlpc150_clicked();
    if(result == PASS)
    {
        out << "PASS,";
    }
    else if(result == FAIL)
    {
        out << "FAIL,";
    }
    else
    {
        out << "READ FAIL,";
    }

    percent_completion = 100;
    ui->progressbar_runTests_DLPC->setValue(percent_completion);

    ui->label_runtests_progress_DLPC->setText("Results logged in file below");
    ui->lineEdit_test_results->setText(QString("%1").arg(filepath.GettestDLPCFileName()));

    testResultsFile.close();

    NNO_ResetTiva();
}


void MainWindow::on_pushButton_runAll_tests_Detector_clicked()
/**
 *This function is a handler for pushButton_runAll_tests_Detector on Test tab, clicked() event
 *Sequentially runs all the detector tests on EVM and saves the results in SelfTest_Detector* file
 */
{
    if(ui->lineEdit_testname->text().isEmpty())
    {
        showError("Empty PCBA S/N. Please enter S/N!");
        return;
    }

    QFile testResultsFile(filepath.GettestDetectorFileName());
    testResultsFile.open(QIODevice::ReadWrite | QIODevice::Text);
    QTextStream out(&testResultsFile);
    long long percent_completion;
    QString savedData = out.readAll();

    if(savedData == NULL)
    {
        out << "PCBA S/N,ADC Test,TMP Test\n";
    }
    else
    {
        out << "\n";
    }

    //Log PCBA Serial number to the test results file
    out << ui->lineEdit_testname->text() << ",";

    clearDetectorTestResults();
    ui->progressbar_runTests_Detector->setValue(0);

    ui->label_runtests_progress_Detector->setText("Running ADC Test");
    QApplication::processEvents();

    on_pushButton_test_adc_clicked();
    out << errorString;
    out << ",";

    percent_completion = 50;
    ui->progressbar_runTests_Detector->setValue(percent_completion);

    ui->label_runtests_progress_Detector->setText("Running TMP Test");
    QApplication::processEvents();

    on_pushButton_test_tmp_clicked();
    out << errorString;
    out << ",";

    percent_completion = 100;
    ui->progressbar_runTests_Detector->setValue(percent_completion);

    ui->label_runtests_progress_Detector->setText("Results logged in file below");
    ui->lineEdit_test_results->setText(QString("%1").arg(filepath.GettestDetectorFileName()));

    testResultsFile.close();

    NNO_ResetTiva();
}

void MainWindow::on_pushButton_erase_flash_clicked()
{
    ui->pushButton_erase_flash->setEnabled(false);
    QApplication::processEvents();
    if(PASS != NNO_EraseDLPC150Flash())
        showError("command failed");
    else
        showError("Erase complete");
    ui->pushButton_erase_flash->setEnabled(true);
}

