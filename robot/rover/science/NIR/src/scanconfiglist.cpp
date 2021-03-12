/**
 *
 * scan configuration list management functions.
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
*/

#include <QString>
#include <QFile>
#include <QDataStream>
#include "API.h"
#include "dlpspec_scan.h"
#include "scanconfiglist.h"

ScanConfigList::ScanConfigList()
{
    //in case the EVM is not coonected
    m_activeindex = -1;
}

int ScanConfigList::ImportTargetList(void)
/**
 * This function imports Scan configurations stored in the EVM
 * @return  < 0 = FAIL;
 *
 */
{
    int num_records = NNO_GetNumScanCfg();
    uint8 i;
    void *buf;
    uScanConfig *pCfg;
    uint32 bufSize;
    int ret_val;

    if(num_records <= 0)
    {
        return FAIL;
    }

    buf = (void *)malloc(NNO_DATA_MAX_SIZE);
    if(buf == NULL)
    {
        return FAIL;
    }
    pCfg = (uScanConfig *)buf;
    target_scan_cfg_list.clear();

    for(i=0; i<num_records; i++)
    {
        ret_val = NNO_GetScanCfg(i, buf, &bufSize);
        if((ret_val == PASS) && (bufSize != 0))
        {
            dlpspec_scan_read_configuration(buf, bufSize);
            target_scan_cfg_list.push_back(*pCfg);
        }
        else
        {
            break;
        }
    }

    free(buf);
    m_activeindex = NNO_GetActiveScanIndex();

    return ret_val;
}

int ScanConfigList::ExportTargetList()
/**
 * This function exports TargetList of Scan configurations to the EVM
 * @return  < 0 = FAIL; The negative number returned specifies which item failed to save.
 *
 */
{
    size_t bufferSize;
    void *pBuffer;
    int i;
    int ret = PASS;
    uScanConfig cfg;

    if(target_scan_cfg_list.count() >= 0)
        NNO_EraseAllScanCfg();

    for(i=0; i < target_scan_cfg_list.count(); i++)
    {
        memcpy(&cfg, &target_scan_cfg_list.at(i), sizeof(cfg));
        if((ret = dlpspec_get_scan_config_dump_size(&cfg, &bufferSize)) != DLPSPEC_PASS)
            break;
        pBuffer =  malloc(bufferSize);
        if(pBuffer == NULL)
        {
            return FAIL;
        }
        if(dlpspec_scan_write_configuration(&cfg, pBuffer, bufferSize) != DLPSPEC_PASS)
        {
            free(pBuffer);
            ret = FAIL;
            break;
        }
        if(NNO_SaveScanCfgInEVM(i, pBuffer, bufferSize) != PASS)
        {
            free(pBuffer);
            ret = 0-i;
            break;
        }

        free(pBuffer);
    }
    NNO_SetActiveScanIndex(m_activeindex);
    return ret;
}

QDataStream &operator<<(QDataStream &out, uScanConfig& config)
/*
 * This function writes the scanConfig Data to Datastream
 *
 * @param  out - O - reference to QDatastream
 * @param  config - I - reference to the scanConfig that should be written
 * @return out - the output QDataStream containing the scanConfig
 *          <0 = error
 *
 */
{
    QString str(config.scanCfg.config_name);

    out << config.scanCfg.scan_type;
    if(config.scanCfg.scan_type != SLEW_TYPE)
    {
        out << config.scanCfg.wavelength_start_nm;
        out << config.scanCfg.wavelength_end_nm;
        out << config.scanCfg.width_px;
        out << config.scanCfg.num_patterns;
        out << config.scanCfg.num_repeats;
        out << str;
    }
    else
    {
        out << config.slewScanCfg.head.num_sections;
        out << config.slewScanCfg.head.num_repeats;
        out << str;
        for(int i=0; i < config.slewScanCfg.head.num_sections; i++)
        {
            out << config.slewScanCfg.section[i].section_scan_type;
            out << config.slewScanCfg.section[i].wavelength_start_nm;
            out << config.slewScanCfg.section[i].wavelength_end_nm;
            out << config.slewScanCfg.section[i].width_px;
            out << config.slewScanCfg.section[i].num_patterns;
            out << config.slewScanCfg.section[i].exposure_time;
        }
    }
    return out;
}

QDataStream &operator>>(QDataStream &in, uScanConfig* config)
/*
 * This function reads the scanConfig Data from the input Datastream
 *
 * @param  in - O - reference to QDatastream
 * @param  config - O - reference to the scanConfig that should be written
 * @return in - the QDataStream containing the scanConfig
 *          <0 = error
 *
*/
{
    QString str;

    in >> config->scanCfg.scan_type;
    if(config->scanCfg.scan_type != SLEW_TYPE)
    {
        in >> config->scanCfg.wavelength_start_nm;
        in >> config->scanCfg.wavelength_end_nm;
        in >> config->scanCfg.width_px;
        in >> config->scanCfg.num_patterns;
        in >> config->scanCfg.num_repeats;
        in >> str;
    }
    else
    {
        in >> config->slewScanCfg.head.num_sections;
        in >> config->slewScanCfg.head.num_repeats;
        in >> str;
        for(int i=0; i < config->slewScanCfg.head.num_sections; i++)
        {
            in >> config->slewScanCfg.section[i].section_scan_type;
            in >> config->slewScanCfg.section[i].wavelength_start_nm;
            in >> config->slewScanCfg.section[i].wavelength_end_nm;
            in >> config->slewScanCfg.section[i].width_px;
            in >> config->slewScanCfg.section[i].num_patterns;
            in >> config->slewScanCfg.section[i].exposure_time;
        }
    }

    strcpy(config->scanCfg.config_name, (char *)str.toStdString().c_str());

    return in;
}

int ScanConfigList::ImportLocalList(QString configFileName)
/**
 * This function imports Scan configurations stored in a file
 * @return  < 0 = FAIL;
 *
 */
{
    QFile file(configFileName);
    uScanConfig config;
    int num=0;

    if(file.open(QIODevice::ReadOnly))
    {
        QDataStream in(&file);   // we will serialize the data into the file
        in >> num;
        scan_cfg_list.clear();
        for(int i = 0; i < num; i++)
        {
            in >> &config;
            scan_cfg_list.append(config);
        }
        file.close();
        return PASS;
    }
    return FAIL;
}

int ScanConfigList::ExportLocalList(QString configFileName)
/**
 * This function exports local Scan configurations to a file
 * @return  < 0 = FAIL;
 *
 */
{
    QFile file(configFileName);

    if(file.open(QIODevice::ReadWrite))
    {
        QDataStream out(&file);   // we will serialize the data into the file
        out << scan_cfg_list.size();
        for(int i = 0; i < scan_cfg_list.size(); i++)
        {
            uScanConfig config = scan_cfg_list.at(i);
            out << config;
        }
        file.close();
        return PASS;
    }
    return FAIL;
}

int ScanConfigList::GetItemAt(int index, uScanConfig *pItem)
{
    if(index < scan_cfg_list.count())
    {
        memcpy(pItem, &scan_cfg_list.at(index), sizeof(uScanConfig));
    }
    else if(index < (scan_cfg_list.count() + target_scan_cfg_list.count()))
    {
        memcpy(pItem, &target_scan_cfg_list.at(index - scan_cfg_list.count()), sizeof(uScanConfig));
    }
    else
    {
        return FAIL;
    }

    return PASS;
}
