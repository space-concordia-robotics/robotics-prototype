/*****************************************************************************
**
**  Copyright (c) 2015 Texas Instruments Incorporated.
**  ALL RIGHTS RESERVED
**
******************************************************************************/

#ifndef BLUETOOTHDIALOG_H
#define BLUETOOTHDIALOG_H

#include <QDialog>
#include "mainwindow.h"


namespace Ui {
class BlueToothDialog;
}

class BlueToothDialog : public QDialog
{
    Q_OBJECT

public:
    explicit BlueToothDialog(MainWindow *myWindow, QWidget *parent = 0);
    ~BlueToothDialog();

private slots:
    void on_pushButton_Ok_clicked();

    void on_pushButton_Cancel_clicked();

private:
    Ui::BlueToothDialog *ui;
    MainWindow* myMainWindow;
};

#endif // BLUETOOTHDIALOG_H
