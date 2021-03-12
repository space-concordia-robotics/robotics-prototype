#include "versiondialog.h"
#include "ui_versiondialog.h"
#include "QTableWidgetItem"
#include "QTableWidget"
#include "version.h"
#include "dlpspec_version.h"

versiondialog::versiondialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::versiondialog)
{
    ui->setupUi(this);

   // ui->versiontable->setHorizontalHeaderItem(0, new QTableWidgetItem("Component"));
    ui->versiontable->setHorizontalHeaderItem(0, new QTableWidgetItem("Expected Version"));
    ui->versiontable->setHorizontalHeaderItem(1, new QTableWidgetItem("Detected Version"));

    ui->versiontable->setVerticalHeaderItem(0, new QTableWidgetItem("TIVA SW"));
    ui->versiontable->setVerticalHeaderItem(1, new QTableWidgetItem("DLPC FW"));
    ui->versiontable->setVerticalHeaderItem(2, new QTableWidgetItem("SPECTRUM LIBRARY"));
    ui->versiontable->setVerticalHeaderItem(3, new QTableWidgetItem("EEPROM CAL"));
    ui->versiontable->setVerticalHeaderItem(4, new QTableWidgetItem("REFERENCE CAL"));
    ui->versiontable->setVerticalHeaderItem(5, new QTableWidgetItem("EEPROM CFG"));


    QString versionStr;

    ui->versiontable->horizontalHeader()->stretchLastSection();
    ui->versiontable->resizeColumnsToContents();
    //set expected TIVA version
    versionStr.sprintf("%d.%d.x", GUI_VERSION_MAJOR,GUI_VERSION_MINOR);

    ui->versiontable->setItem(0,0,new QTableWidgetItem(versionStr));


    //set expected DLPC version
    versionStr.sprintf("%d.%d.x", DLPC_VERSION_MAJOR,DLPC_VERSION_MINOR);

    ui->versiontable->setItem(1,0,new QTableWidgetItem(versionStr));


    versionStr.sprintf("%d.%d.x", DLPSPEC_VERSION_MAJOR,DLPSPEC_VERSION_MINOR);

    ui->versiontable->setItem(2,0,new QTableWidgetItem(versionStr));


    versionStr.sprintf("%d", DLPSPEC_CALIB_VER);

    ui->versiontable->setItem(3,0,new QTableWidgetItem(versionStr));

    versionStr.sprintf("%d", DLPSPEC_REFCAL_VER);

    ui->versiontable->setItem(4,0,new QTableWidgetItem(versionStr));

    versionStr.sprintf("%d", DLPSPEC_CFG_VER);

    ui->versiontable->setItem(5,0,new QTableWidgetItem(versionStr));

}

versiondialog::~versiondialog()
{
    delete ui;
}
void versiondialog::setTivaVersion(QString str)
{
    QTableWidgetItem *itab = new QTableWidgetItem();
    itab->setTextColor(Qt::black);
    itab->setText(str);
    ui->versiontable->setItem(0, 1, itab);

}

void versiondialog::setDlpcVersion(QString str)
{
    QTableWidgetItem *itab = new QTableWidgetItem();
    itab->setTextColor(Qt::black);
    itab->setText(str);
    ui->versiontable->setItem(1, 1, itab);

}
void versiondialog::setSpeclibVersion(QString str)
{
    QTableWidgetItem *itab = new QTableWidgetItem();
    itab->setTextColor(Qt::black);
    itab->setText(str);
    ui->versiontable->setItem(2, 1, itab);

}

void versiondialog::setEEPROMcalVersion(QString str)
{
    QTableWidgetItem *itab = new QTableWidgetItem();
    itab->setTextColor(Qt::black);
    itab->setText(str);
    ui->versiontable->setItem(3, 1, itab);
}

void versiondialog::setRefcalVersion(QString str)
{
    QTableWidgetItem *itab = new QTableWidgetItem();
    itab->setTextColor(Qt::black);
    itab->setText(str);
    ui->versiontable->setItem(4, 1, itab);
}

void versiondialog::setEEPROMcfgVersion(QString str)
{
    QTableWidgetItem *itab = new QTableWidgetItem();
    itab->setTextColor(Qt::black);
    itab->setText(str);
    ui->versiontable->setItem(5, 1, itab);
}

void versiondialog::setTivaVersionRed()
{
    QTableWidgetItem* itab = ui->versiontable->item(0,1);
    if(itab)
     itab->setTextColor(Qt::red);

}
void versiondialog::setDLPCVersionRed()
{
    QTableWidgetItem* itab = ui->versiontable->item(1,1);
    if(itab)
     itab->setTextColor(Qt::red);

}
void versiondialog::setSpeclibVersionRed()
{
    QTableWidgetItem* itab = ui->versiontable->item(2,1);
    if(itab)
     itab->setTextColor(Qt::red);

}

void versiondialog::setEEPROMcalVersionRed()
{
    QTableWidgetItem* itab = ui->versiontable->item(3,1);
    if(itab)
     itab->setTextColor(Qt::red);

}

void versiondialog::setRefcalVersionRed()
{
    QTableWidgetItem* itab = ui->versiontable->item(4,1);
    if(itab)
     itab->setTextColor(Qt::red);

}

void versiondialog::setEEPROMcfgVersionRed()
{
    QTableWidgetItem* itab = ui->versiontable->item(5,1);
    if(itab)
     itab->setTextColor(Qt::red);

}

void versiondialog::showEEPROMVersions(bool show)
{
    if(show)
    {
        ui->versiontable->setRowCount(6);
        ui->versiontable->verticalHeader()->setDefaultSectionSize(30);
    }
    else
    {
         ui->versiontable->setRowCount(3);
         ui->versiontable->verticalHeader()->setDefaultSectionSize(60);
    }
}

void versiondialog::on_Close_clicked()
{
   this->close();
}
