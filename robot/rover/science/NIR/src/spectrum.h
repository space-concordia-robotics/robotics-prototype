/*****************************************************************************
**
**  Copyright (c) 2015 Texas Instruments Incorporated.
**  ALL RIGHTS RESERVED
**
******************************************************************************/

/**
 * \class Spectrum
 *
 * \brief Encapsulates spectrum data storage and manipulation.
 * 
 * This class encapsulates the data retrieved from NIRscan Nano EVM after scanning
 * a sample. It uses APIs from the spectrum library to deserialize the binary data
 * and interpret it. It also provides functions to save, retrieve and plot this data.
 *
 */

#ifndef SPECTRUM_H
#define SPECTRUM_H

#pragma once

#include <QList>
#include <QVector>
#include "dlpspec_scan.h"
#include <iostream>
#include <QTextStream>
#include "Common.h"
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsTextItem>
#include "plot.h"
#include "filepath.h"

#define COMPUTE_ABSORPTION_VAL(S , R)	((double)-1.0 * (double)log10( (double) ((double) (S) / (double)(R) ) ) )
#define COMPUTE_REFLECTANCE_VAL(S , R)	((double) (S) / (double)(R))

class Spectrum
{
public:

    int SetData(void* pScanDataBlob, void *pRefDataBlob);
    int SetInterpretData(void* pScanDataBlob);

    //bool setheader_csv =true;
    //bool setheader_JCAMP =true;
    //bool saveseperate_JCAMP=false;
    //bool saveone_JCAMP=true;
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

    QVector<double> GetWavelengths();
    QVector<double> GetIntensities();
    QVector<double> GetAbsorbance();
    QVector<double> GetReflectance();
    slewScanConfig GetScanConfig();
    QString GetReferenceTimeStamp();

    bool Plot(PLOT_TYPE type,QGraphicsScene *scene);
    void ResetPlot();

    int SaveToFile();
    void SaveReferenceToFile();    
    void SetSaveFileNamePrefix(QString prefix){m_prefix = prefix; m_fileName = "";}
    void SetSaveFileName(QString fname){m_fileName = fname; m_prefix = "";}

    int ReadSavedScanFromFile(QString FileName);
    int ReadReferenceFromFile(void *pRefDataBlob, const uScanConfig *pCfg);
    void SavetoJCamp(QString scanName);
    void SaveToCSV(QString scanName);
    void Saveheader(QTextStream *out,bool ifJCAMP);
    void SaveCombinedCSV(QString fileNameRe);

    QString GetScanTimeStamp();

private:

    scanResults m_scanResults;
    scanResults m_referenceResults;    

    double m_maxAbsorbance;

    uint8  m_curScanDataBlob[SCAN_DATA_BLOB_SIZE];
    uint8  m_curReferenceDataBlob[SCAN_DATA_BLOB_SIZE];

    QString m_prefix;
    CustomPlot m_plot;
    QString m_fileName;


    QVector<double> GetRefIntensities();
    QString GetReferenceFileName(const uScanConfig *pCfg);
};

#endif // SPECTRUM_H
