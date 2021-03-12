/**
 *
 * This class provides EVM specific functions.
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
*/
#ifndef EVM_H
#define EVM_H

#include "dlpspec_scan.h"
#include "dlpspec_calib.h"
#include <QList>

class EVM
{
public:
    EVM();
    ~EVM();

private:

    int m_refPGA, m_curPGA;

    void *m_RefCalMatrixBlob;

    void *m_RefCalDataBlob;

public:

    int ApplyScanCfgtoDevice(uScanConfig *pCfg);

    void *GetRefCalMatrixBlob(char *p_ser_num_str);

    void * GetRefCalDataBlob() { return m_RefCalDataBlob;}

    int GenCalibPatterns(CALIB_SCAN_TYPES scan_type);

    int FetchRefCalData();

    int FetchRefCalMatrix(void);

};

#endif // EVM_H
