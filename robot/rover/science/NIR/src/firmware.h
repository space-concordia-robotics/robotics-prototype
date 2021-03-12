/*****************************************************************************
**
**  Copyright (c) 2015 Texas Instruments Incorporated.
**  ALL RIGHTS RESERVED
**
******************************************************************************/
#ifndef FIRMWARE_H
#define FIRMWARE_H

#include "Common.h"
#include <QString>

#define FLASHTABLE_APP_SIGNATURE	0x01234567


/** Flash Data Address Types */
typedef struct
{
    uint32          Signature;    /* = 0x1234567 */
    uint32          Boot_Address;     /* Address of Application entry to bootloader */
    uint16          Version;          /**< Version == 0x13 */
    uint16          FlashBlockCount;  /**< Number of Flash block entries present in the flash image */
    uint32          Free_Area_Start;  /**< Address of first free location in flash */
} FLASH_TABLE; /*  */

bool Frmw_IsFirwareSignatureMatch(const unsigned char *pByteArray);
bool Frmw_IsTIVAFirmware(const unsigned char *pFrmwImageArray , long long dataLen);


#endif // FIRMWARE_H


