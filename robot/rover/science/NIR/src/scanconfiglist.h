/*****************************************************************************
**
**  Copyright (c) 2015 Texas Instruments Incorporated.
**  ALL RIGHTS RESERVED
**
******************************************************************************/
#ifndef SCANCONFIGLIST_H
#define SCANCONFIGLIST_H

#include <QList>

class ScanConfigList
{
public:
    explicit ScanConfigList();

    QList<uScanConfig> GetLocalList() { return scan_cfg_list ;}
    void SetLocalList(QList<uScanConfig> local_scans) { scan_cfg_list = local_scans;}

    QList<uScanConfig> GetTargetList() { return target_scan_cfg_list;}
    void SetTargetList(QList<uScanConfig> target_scans) { target_scan_cfg_list = target_scans;}

    int count(void) { return (scan_cfg_list.count() + target_scan_cfg_list.count()); }
    int GetItemAt(int index, uScanConfig *pItem);

    int Get_ActiveConfigIndex(){ return m_activeindex;}
    void Set_ActiveConfigIndex(int index){ m_activeindex = index;}

    int ImportTargetList(void);
    int ExportTargetList(void);
    int ImportLocalList(QString configFileName);
    int ExportLocalList(QString configFileName);

private:
    //Data
     QList<uScanConfig> scan_cfg_list  ;
     QList<uScanConfig> target_scan_cfg_list;
     int m_activeindex;
};

#endif // SCANCONFIGLIST_H
