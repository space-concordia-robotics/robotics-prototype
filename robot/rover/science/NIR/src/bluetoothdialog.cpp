/*****************************************************************************
**
** This class implements the Bluetooth Active dialog in Nano GUI
**  Copyright (c) 2015 Texas Instruments Incorporated.
**  ALL RIGHTS RESERVED
**
******************************************************************************/

#include "bluetoothdialog.h"
#include "ui_bluetoothdialog.h"
#include "API.h"
#include "usb.h"
#include <QThread>

BlueToothDialog::BlueToothDialog(MainWindow *myWindow, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::BlueToothDialog)
{
    ui->setupUi(this);
    myMainWindow = myWindow;
}


BlueToothDialog::~BlueToothDialog()
{
    delete ui;
}

void BlueToothDialog::on_pushButton_Ok_clicked()
/**
 * This is a handler function for pushButton_Ok and clliecked() event
 * This is called when a BlueTooth active dialog pops up and user wants to
 * wait till the connection is closed.
 * This periodically gets the status from EVM and closes the dialog once
 * the BlueTooth active status is off i.e the connection is closed
 */
{
    ui->label->setText("Waiting for the BlueTooth Connection to be closed");
    QApplication::processEvents();

    unsigned int tiva_sw_ver;
    unsigned int dlpc_sw_ver;
    unsigned int dlpc_fw_ver;
    unsigned int speclib_ver;
    unsigned int eeprom_cfg_ver;
    unsigned int eeprom_cal_ver;
    unsigned int eeprom_refcal_ver;

    ui->pushButton_Ok->hide();
    QApplication::processEvents();

    while(1)
    {
        if(USB_IsConnected())
        {
            if(NNO_GetVersion(&tiva_sw_ver, &dlpc_sw_ver, &dlpc_fw_ver, &speclib_ver, &eeprom_cal_ver, &eeprom_refcal_ver, &eeprom_cfg_ver) != NNO_CMD_BUSY)
            {
                this->close();
                return;
            }
        }
        else
        {
            this->close();
            return;
        }

        QApplication::processEvents();
        QThread::msleep(500);
    }
}

void BlueToothDialog::on_pushButton_Cancel_clicked()
/**
 * This is a handler for pushButton_Cancel(Quit) and clicked() event
 * When a BlueTooth connection is active and User would like to quit the GUI
 *
 */
{
    USB_Close();
    this->close();
    QApplication::quit();
}
