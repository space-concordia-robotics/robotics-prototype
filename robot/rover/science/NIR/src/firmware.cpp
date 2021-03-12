/*****************************************************************************
**
**  Copyright (c) 2015 Texas Instruments Incorporated.
**  ALL RIGHTS RESERVED
**
******************************************************************************/
#include "firmware.h"

#define FLASH_CONST_START_ADDRESS 0x00044000
#define FLASH_CONST_END_ADDRESS   0x0004E200

uint32 FLASH_SIG_ADDRESS = 0x00000000;
uint32 FLASH_TAB_ADDRESS = 0x00000004;



bool Frmw_IsFirwareSignatureMatch(const unsigned char *pFrmwImageArray)
{
    uchar flash_sig[4];
    uchar flash_tab[4];

    //the second 4 bytes in the composer fimrware is always 0x 00 01 00 00
    //using this for now, since Composer is not putting the App signature correctly

    //the signature of the flash is "BATF" so checking the corresponding hex values for the signature
    for ( int i = 0; i < 4; i++)
        flash_sig[i] = (uchar) *(pFrmwImageArray + FLASH_SIG_ADDRESS + i);

    if((flash_sig[0] == 0x42) && (flash_sig[1] == 0x41) && (flash_sig[2] == 0x54) && (flash_sig[3] == 0x46))
    {
        for ( int i = 0; i < 4; i++)
            flash_tab[i] = (uchar) *(pFrmwImageArray + FLASH_TAB_ADDRESS + i);

        if((flash_tab[0] == 0x00) && (flash_tab[1] == 0x10) && (flash_tab[2] == 0x00) && (flash_tab[3] == 0x00))
            return true;
    }
    return false;
}


bool Frmw_IsTIVAFirmware(const unsigned char *pFrmwImageArray , long long datalen)
{
    if(datalen < FLASH_CONST_END_ADDRESS)
        return false;
    
    //APP Signature hardcoded in application code is "DLPNIRNANO"
    for(int i=FLASH_CONST_START_ADDRESS; i<=FLASH_CONST_END_ADDRESS;i++)
    {
        if((*(pFrmwImageArray + i + 0) == 'D') && (*(pFrmwImageArray + i + 1) == 'L')\
        && (*(pFrmwImageArray + i + 2) == 'P') && (*(pFrmwImageArray + i + 3) == 'N')\
        && (*(pFrmwImageArray + i + 4) == 'I') && (*(pFrmwImageArray + i + 5) == 'R')\
        && (*(pFrmwImageArray + i + 6) == 'N') && (*(pFrmwImageArray + i + 7) == 'A')\
        && (*(pFrmwImageArray + i + 8) == 'N') && (*(pFrmwImageArray + i + 9) == 'O'))

            return true;


    }

    return false;
}
