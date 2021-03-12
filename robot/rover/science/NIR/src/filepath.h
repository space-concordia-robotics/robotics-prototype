/**
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
*/
#ifndef DATAPATH_H
#define DATAPATH_H

#include <QFile>
#include <QDir>
class FilePath
{
public:
    FilePath();

    //configuration directories
    QDir userDir;
    QDir displayScanSettingsDir;
    QDir configDir;
    QDir saveScanSettingsDir;

    //file paths
    QString testResultsFileName;
    QString userPath;
    QString dlpcFirmwarePath;
    QString tivaFirmwarePath ;
    QString bmp24Path;
    QString m_selectedScanFile;
    QString testTivaFileName;
    QString testDLPCFileName;
    QString testDetectorFileName;
    QString RecordFileName;
    QString specDetFileName;
    QString specSlitFileName;
    QString specAr1FileName;
    QString specSrm2036FileName;
    QString displayScanSettingsDirPath;
    QString saveScanSettingsDirPath;


    QDir GetdisplayScanSettingsDir(){return displayScanSettingsDir;}
    void SetdisplayScanSettingsDirPath(QString path){ displayScanSettingsDirPath = path;displayScanSettingsDir = QDir(displayScanSettingsDirPath);}
    QString GetdisplayScanSettingsDirPath(){return displayScanSettingsDirPath;}
    QDir GetsaveScanSettingsDir(){ return saveScanSettingsDir;}
    void SetsaveScanSettingsDirPath(QString path){ saveScanSettingsDirPath = path; saveScanSettingsDir = QDir(saveScanSettingsDirPath);}
    QString GetsaveScanSettingsDirPath(){return saveScanSettingsDirPath;}
    QDir GetconfigDir() { return configDir;}

    QString GettestResultsFileName(){return testResultsFileName;}
    QString GetuserPath(){return userPath;}
    QString GettestTivaFileName(){return testTivaFileName;}
    QString GettestDLPCFileName(){return testDLPCFileName;}
    QString GettestDetectorFileName(){return testDetectorFileName;}
    QString GetRecordFileName(){return RecordFileName;}
    QString GetspecDetFileName() { return specDetFileName;}
    QString GetspecSlitFileName() { return specSlitFileName;}
    QString GetspecAr1FileName() { return specAr1FileName;}
    QString GetspecSrm2036FileName() { return specSrm2036FileName;}

    QString GetdlpcFirmwarePath(){return dlpcFirmwarePath;}
    void SetdlpcFirmwarePath(QString path){dlpcFirmwarePath = path;}

    QString GettivaFirmwarePath(){return tivaFirmwarePath;}
    void SettivaFirmwarePath(QString path)
    {
        tivaFirmwarePath = path;
    }

    QString Getbmp24Path(){ return bmp24Path;}
    void Setbmp24Path(QString path)
    {
        bmp24Path = path;
    }

    QString Getm_selectedScanFile(){return m_selectedScanFile;}
    void Setm_selectedScanFile(QString file)
    {
        m_selectedScanFile = file;
    }


};

#endif // DATAPATH_H
