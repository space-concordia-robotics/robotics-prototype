/**
 *
 * This class provides path specific functions.
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
*/


#include "filepath.h"
#include "QStandardPaths"
#include <QTime>

FilePath::FilePath()
{

    configDir= ".//";
    userDir = ".//";


    userPath = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    userPath = QString("%1/Texas Instruments/NIRscanNano").arg(userPath);
    userDir.setPath(userPath);
    if (!userDir.exists())
    {
        userDir.mkpath(".");
    }

    //scan files are saved and saved scans are displayed from user documents Location
    if(saveScanSettingsDirPath != "")
        saveScanSettingsDir.setPath(saveScanSettingsDirPath);
    if(displayScanSettingsDirPath != "")
        displayScanSettingsDir.setPath(displayScanSettingsDirPath);
    if(!saveScanSettingsDir.exists())
        saveScanSettingsDir.setPath(userPath);
    if(!displayScanSettingsDir.exists())
        displayScanSettingsDir.setPath(userPath);

    //the scan configuration file is saved in "C:\Users\<UserName>\AppData\Local\Texas Instruments\NIRscanNano"

    QString Path;
    QStringList PathList = QStandardPaths::standardLocations(QStandardPaths::GenericDataLocation);


    //in the list usually the first one is user specific pat i.e C:/Users/<UserName>/AppData/Local
    //and the second one is C:/ProgramData
    //access the last path in the list
    if(PathList.size() > 0)
        Path = PathList[PathList.size() - 1];

    QString configPath = QString("%1/Texas Instruments/NIRscanNano").arg(Path);

    configDir.setPath(configPath);
    if (!configDir.exists())
    {
        configDir.mkpath(".");
    }

    QString current = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    QString testTivaFileNameRel = QString("SelfTest_Tiva_%1.csv").arg(current);
    QString testDLPCFileNameRel = QString("SelfTest_DLPC_%1.csv").arg(current);
    QString testDetectorFileNameRel = QString("SelfTest_Detector_%1.csv").arg(current);

    testTivaFileName = saveScanSettingsDir.absoluteFilePath(testTivaFileNameRel);
    testDLPCFileName = saveScanSettingsDir.absoluteFilePath(testDLPCFileNameRel);
    testDetectorFileName = saveScanSettingsDir.absoluteFilePath(testDetectorFileNameRel);

    m_selectedScanFile = "";

    QString specDetFileNameRel = "Spec_DetectorAlignment.csv";
    specDetFileName =  configDir.absoluteFilePath(specDetFileNameRel);
    QString specSlitFileNameRel = "Spec_SlitAlignment.csv";
    specSlitFileName = configDir.absoluteFilePath(specSlitFileNameRel);

    QString specAr1FileNameRel = QString("Spec_Ar1Locations_%1.csv").arg(current);
    specAr1FileName = configDir.absoluteFilePath(specAr1FileNameRel);
    QString specSrm2036FileNameRel = QString("Spec_NistSrm2036Locations_%1.csv").arg(current);
    specSrm2036FileName = configDir.absoluteFilePath(specSrm2036FileNameRel);
}
