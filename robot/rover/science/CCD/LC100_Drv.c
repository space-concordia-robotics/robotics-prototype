/****************************************************************************

   Thorlabs LC100 series VXIpnp instrument driver

   FOR DETAILED DESCRIPTION OF THE DRIVER FUNCTIONS SEE THE ONLINE HELP FILE
   AND THE PROGRAMMERS REFERENCE MANUAL.

   Copyright:  Copyright(c) 2011, Thorlabs (www.thorlabs.com)
   Author(s):  Michael Biebl (mbiebl@thorlabs.com)
               Lutz Hoerl (lhoerl@thorlabs.com)

   Disclaimer:

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


   Source file

   Date:          Oct-26-2011
   Built with:    NI LabWindows/CVI 2010
   Software-Nr:   091.93.xxx
   Version:       1.2.3

   Changelog:     see 'readme.rtf'

****************************************************************************/


//#include <utility.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdint.h>

#define NIVISA_USB   // this is necessary to access VISA-USB raw resources
#include <ni-visa/visa.h>
#include "LC100_Drv.h"
#include "vpptype.h"
#include "visatype.h"


/*===========================================================================
 Macros
===========================================================================*/
// Resource locking
#ifdef _CVI_DEBUG_
   // We are in a debugging session - do not lock resource
   #define LC100_LOCK_STATE            VI_NULL
#else
   #define LC100_LOCK_STATE            VI_EXCLUSIVE_LOCK
#endif

// Version
#define DRIVER_REVISION_TXT            "1.2.3"    // Instrument driver revision

// Macros for LC100 commands
// reflect commands found in LC100_cmds.h of 8051 firmware project 08010
//    WRITE commands (OUT)
#define LC100_WCMD_LOAD_FPGA           0x10
#define LC100_WCMD_SET_VID_PID         0x11
#define LC100_WCMD_SET_CONFIG          0x12
#define LC100_WCMD_WRITE_I2C           0x20
#define LC100_WCMD_WRITE_EEPROM        0x21
#define LC100_WCMD_RESET               0x2E
#define LC100_WCMD_ALWAYS_STALL        0x2F
//#define LC100_WCMD_WRITE_FPGA_I2C      0x40
#define LC100_WCMD_WRITE_EVALBOX       0x41
#define LC100_WCMD_GPIO_FEED           0x42
#define LC100_WCMD_ITIME               0x43
//#define LC100_WCMD_LUT                 0x44
#define LC100_WCMD_GPIO_MODE           0x45
#define LC100_WCMD_CCD                 0x46
#define LC100_WCMD_SET_OPMODE          0x47
#define LC100_WCMD____                 0x48
#define LC100_WCMD_EXEC_SCAN           0x49
#define LC100_WCMD_RESET_FIFOS         0x4B  // do not use, use LC100_WCMD_RESET instead
#define LC100_WCMD_USER_TEXT           0x4D
#define LC100_WCMD_NV_MEMORY           0x4E
#define LC100_WCMD_SERVICE_MODE        0x50
#define LC100_WCMD_AOUT                0x51

//    READ commands (IN)
#define LC100_RCMD_READ_I2C            0x20
#define LC100_RCMD_READ_EEPROM         0x21
#define LC100_RCMD_ALWAYS_STALL        0x2F
//#define LC100_RCMD_READ_FPGA_I2C       0x40
#define LC100_RCMD_READ_EVALBOX        0x41
#define LC100_RCMD_GPIO_FEED           0x42
#define LC100_RCMD_ITIME               0x43
//#define LC100_RCMD_LUT                 0x44
#define LC100_RCMD_GPIO_MODE           0x45
#define LC100_RCMD_CCD                 0x46
#define LC100_RCMD_GET_OPMODE          0x47
#define LC100_RCMD_GET_VERSION         0x48
#define LC100_RCMD_GET_STATUS          0x4A
#define LC100_RCMD_GPIO_STATE          0x4C
#define LC100_RCMD_USER_TEXT           0x4D
#define LC100_RCMD_NV_MEMORY           0x4E
#define LC100_RCMD_SERVICE_MODE        0x50
#define LC100_RCMD_AOUT                0x51
#define LC100_RCMD_GET_ERROR           0xFF

// macros for parameter index of commands above
// macros for index for command LC100_WCMD_NV_MEMORY
   #define LC100_NVMEM_EEPROM                0
   #define LC100_NVMEM_STORE                 1
   #define LC100_NVMEM_LUT_FACTORY           2
   #define LC100_NVMEM_LUT_USER              3
   #define LC100_NVMEM_VERSION               4
   #define LC100_NVMEM_LUT_SCRATCH           5
   #define LC100_NVMEM_USER_WL_POLY          6
   #define LC100_NVMEM_USER_WL_POINTS        7
   #define LC100_NVMEM_USER_WL_FLAG          8
   #define LC100_NVMEM_USER_WL_NPTS          9
   #define LC100_NVMEM_FACT_WL_POLY          10
   #define LC100_NVMEM_FACT_WL_POINTS        11
   #define LC100_NVMEM_FACT_WL_FLAG          12
   #define LC100_NVMEM_FACT_WL_NPTS          13

// macros for parameter val for command LC100_WCMD_NV_MEMORY for index LC100_NVMEM_STORE
/*#define LC100_SETUP_LUT_NV_FACT_TO_SCRATCH   3     // copy Factory Amplitude Correction Data from NVMEM to SCRATCH RAM for later readout via
#define LC100_SETUP_LUT_NV_USER_TO_SCRATCH   4     // copy LUT from USER NVMEM to SCRATCH RAM
#define LC100_SETUP_LUT_APPLIED_TO_SCRATCH   5     // copy LUT from APPLIED RAM to SCRATCH RAM (with 16/32 correction)
#define LC100_SETUP_LUT_STORE_LUT_FACT       6     // copy LUT from RAM to NVMEM FACTORY
#define LC100_SETUP_LUT_STORE_LUT_USER       7     // copy LUT from RAM to NVMEM USER
#define LC100_SETUP_LUT_READ_LUT_FACT        8     // copy LUT from NVMEM FACTORY to RAM
#define LC100_SETUP_LUT_READ_LUT_USER        9     // copy LUT from NVMEM USER to RAM
#define LC100_SETUP_LUT_APPLY_LUT            10    // apply LUT from RAM to RAM (with multiplication)
  */
   #define NVMEM_STORE_RESET_RAM       0     // reset to factory defaults, RAM only, no change in NVMEM
   #define NVMEM_STORE_RAM             1     // store current settings to NVMEM
   #define NVMEM_STORE_RESET_EEPROM    2     // reset to factory defaults, RAM and NVMEM + apply
   #define NVMEM_STORE_RELOAD          3     // reload NVMEM data and apply them
   #define NVMEM_NV_FACT_TO_SCRATCH    4     // copy LUT from FACTORY NVMEM to SCRATCH RAM
   #define NVMEM_NV_USER_TO_SCRATCH    5     // copy LUT from USER    NVMEM to SCRATCH RAM
   #define NVMEM_SCRATCH_TO_NV_FACT    6     // copy LUT from SCRATCH RAM   to FACTORY NVMEM
   #define NVMEM_SCRATCH_TO_NV_USER    7     // copy LUT from SCRATCH RAM   to USER    NVMEM
   #define NVMEM_STORE_LUT_FACT        8     // copy LUT from FACTORY RAM   to FACTORY NVMEM
   #define NVMEM_STORE_LUT_USER        9     // copy LUT from USER    RAM   to USER    NVMEM
   #define NVMEM_READ_LUT_FACT         10    // copy LUT from FACTORY NVMEM to FACTORY RAM
   #define NVMEM_READ_LUT_USER         11    // copy LUT from USER    NVMEM to USER    RAM
   #define NVMEM_FACT_TO_SCRATCH       12    // copy LUT from FACTORY RAM   to SCRATCH RAM
   #define NVMEM_USER_TO_SCRATCH       13    // copy LUT from USER    RAM   to SCRATCH RAM
   #define NVMEM_SCRATCH_TO_FACT       14    // copy LUT from SCRATCH RAM   to FACTORY RAM
   #define NVMEM_SCRATCH_TO_USER       15    // copy LUT from SCRATCH RAM   to USER    RAM
   #define NVMEM_APPLIED_TO_SCRATCH    16    // copy LUT from APPLIED RAM   to SCRATCH RAM (with 16/32 correction)
   #define NVMEM_APPLY_LUT             17    // apply LUT from RAM to RAM (with multiplication)



// strings
#define DEFAULT_USER_TEXT              "My LC100 Line Camera"

// Buffers
#define LC100_NUM_POLY_POINTS          4     // number of polynom coefficients

// Analysis 
#define MATRIX_ROWS                    4
#define MATRIX_COLS                    4

// User versus Factory settinfs/calibration
#define TARGET_FACT                    0
#define TARGET_USER                    1

/*===========================================================================
   EEPROM mapping
===========================================================================*/
#define EE_LENGTH_ACOR              (sizeof(ViUInt16) * LC100_NUM_PIXELS)

// data from nvmem.h from MUN1040 project
#define NUM_PIXEL_MAX               LC100_NUM_PIXELS
#define NVMA_LUT_FACTORY            (4096)
#define NVML_LUT_FACTORY            (NUM_PIXEL_MAX * sizeof(ViUInt16))

#define NVMA_LUT_USER               (NVMA_LUT_FACTORY + NVML_LUT_FACTORY + NVML_CRC)
#define NVML_LUT_USER               (NUM_PIXEL_MAX * sizeof(ViUInt16))



/*===========================================================================
 Structures
===========================================================================*/
// Converter types
typedef struct
{
   ViUInt8 hb;
   ViUInt8 lb;
} conv16_t;

typedef union
{
   conv16_t two_bytes;
   ViUInt16 one_word;
} union16_t;

typedef struct
{
   ViUInt8 hhb;
   ViUInt8 hlb;
   ViUInt8 lhb;
   ViUInt8 llb;
} conv32_t;

typedef union
{
   conv32_t four_bytes;
   ViUInt32 one_dword;
} union32_t;

// static error list
typedef struct
{
   ViStatus err;
   ViString descr;
} errDescrStat_t;

// Sensor dependent data
typedef struct
{
   ViUInt16 numPix;        // number of pixels
   ViUInt32 minIntTim;     // minimum integration time in ?s
   ViUInt32 maxIntTim;     // maximum integration time in ?s
   ViChar   *descr;        // description
} LC100_sensData_t;

// wavelength calibratiobn
typedef struct
{
   ViUInt16       cal_node_cnt;                           // number of user-defined supporting points
   ViUInt32       cal_node_pixel[LC100_MAX_NUM_USR_ADJ];  // pixel array of supporting points
   ViReal64       cal_node_wl[LC100_MAX_NUM_USR_ADJ];     // wavelength array of supporting points
   
} LC100_wl_cal_pts_t; 

typedef struct
{
   ViReal64       poly[LC100_NUM_POLY_POINTS];  // polynomial coefficients for pixel - wavelength computation
   ViReal64       min;                          // lower wavelength limit
   ViReal64       max;
   ViReal64       wl[LC100_NUM_PIXELS];         // array of wavelengths according to pixel number
   ViUInt16       valid;                        // valid flag
} LC100_wl_cal_t;

// driver private data
typedef struct
{
   ViSession               instr;         // instrument handle
   ViBoolean               reset;         // reset parameter passed to LC100_init()
   ViBoolean               idQuery;       // idQuery parameter passed to LC100_init()
   ViUInt16                pid;           // USB PID value
   ViUInt16                nvVersion;     // version of NV-memory, specifies record organisation
   const LC100_sensData_t  *sensData;     // sensor dependent data
   ViAttrState             userData;      // user data - available via LC100_setAttribute()/LC100_getAttribute()

   // device calibration
   LC100_wl_cal_t          factory_cal;
   LC100_wl_cal_t          user_cal;
   LC100_wl_cal_pts_t      factory_points;
   LC100_wl_cal_pts_t      user_points;
   
} LC100_data_t;


/*===========================================================================
 Constants
===========================================================================*/
#define RAW_DATA_BUF_SIZE        4096     // must be equal or greater than following
                                          // "ViUInt16 numPix;"-entries for number of sensor pixels


// ILX 554B sensor data
static const LC100_sensData_t LC100_sensData_ILX554B =
{
   2048,       // number of pixels
   1054,       // minimum integration time in ?s, default value, has to be read out of device
   1000000,    // maximum integration time in ?s, default value, has to be read out of device
   "ILX 554B CCD sensor"         // description
};


// Static error descriptions
static const errDescrStat_t LC100_errDescrStat[] =
{
   {VI_ERROR_PARAMETER1,               "Parameter 1 out of range"                               },
   {VI_ERROR_PARAMETER2,               "Parameter 2 out of range"                               },
   {VI_ERROR_PARAMETER3,               "Parameter 3 out of range"                               },
   {VI_ERROR_PARAMETER4,               "Parameter 4 out of range"                               },
   {VI_ERROR_PARAMETER5,               "Parameter 5 out of range"                               },
   {VI_ERROR_PARAMETER6,               "Parameter 6 out of range"                               },
   {VI_ERROR_PARAMETER7,               "Parameter 7 out of range"                               },
   {VI_ERROR_PARAMETER8,               "Parameter 8 out of range"                               },
   {VI_ERROR_INV_RESPONSE,             "Errors occured interpreting instrument's response"      },

   {VI_ERROR_NSUP_COMMAND,             "Command not supported by instrument"                    },
   {VI_ERROR_LC100_UNKNOWN,            "Unknown LC100 error, please report this"                },
   {VI_ERROR_NSUP_DEVICE,              "The device is not supported by this instrument driver"  },


   {VI_ERROR_XSVF_SIZE,                "XSVF stream size must be greater 0"               },
   {VI_ERROR_XSVF_MEMORY,              "Memory allocation for XSVF stream failed"         },
   {VI_ERROR_XSVF_FILE,                "Access to XSVF file failed"                       },

   {VI_ERROR_FIRMWARE_SIZE,            "Firmware size must be greater than 0"             },
   {VI_ERROR_FIRMWARE_MEMORY,          "Memory allocation for firmware data failed"       },
   {VI_ERROR_FIRMWARE_FILE,            "Access to firmware file failed"                   },
   {VI_ERROR_FIRMWARE_CHKSUM,          "Checksum error in firmware HEX-File"              },
   {VI_ERROR_FIRMWARE_BUFOFL,          "Given buffer is to small for firmware HEX-File"   },
   {VI_ERROR_FIRMWARE_MISMATCH,        "Readback data unexpected"                         },

   {VI_ERROR_CYEEPROM_SIZE,            "EEPROM size mismatch"                             },
   {VI_ERROR_CYEEPROM_MEMORY,          "Memory allocation for EEPROM data failed"         },
   {VI_ERROR_CYEEPROM_FILE,            "Access to EEPROM file failed"                     },
   {VI_ERROR_CYEEPROM_CHKSUM,          "Checksum error in EEPROM HEX-File"                },
   {VI_ERROR_CYEEPROM_BUFOVL,          "Given buffer is to small for EEPROM HEX-File"     },

   {VI_ERROR_IIC_SIZE,                 "EEPROM size mismatch"                             },
   {VI_ERROR_IIC_MEMORY,               "Memory allocation for IIC data failed"            },
   {VI_ERROR_IIC_FILE,                 "Access to EEPROM file failed"                     },
   {VI_ERROR_IIC_CHKSUM,               "Checksum error in EEPROM HEX-File"                },
   {VI_ERROR_IIC_BUFOVL,               "Given buffer is to small for EEPROM HEX-File"     },

   {VI_ERROR_NVMEM_CHKSUM,             "Nonvolatile memory checksum error"                },

   {VI_ERROR_LC100_ENDP0_SIZE,         "Attempt to send or receive too many bytes EP0"    },
   {VI_ERROR_LC100_EEPROM_ADR_TO_BIG,  "Given EEPROM address is to big"                   },
   {VI_ERROR_LC100_INVAL_RCMD,         "Unknown USB read command"                         },
   {VI_ERROR_LC100_INVAL_WCMD,         "Unknown USB write command"                        },
   {VI_ERROR_LC100_INVAL_INDEX,        "Invalid USB control transfer parameter index"     },
   {VI_ERROR_LC100_INVAL_LEN,          "Invalid USB control transfer parameter length"    },
   {VI_ERROR_LC100_INVAL_VALUE,        "Invalid USB control transfer parameter value"     },
   {VI_ERROR_LC100_FPGA_UNKNOWN,       "Unkown errror from FPGA"                          },
   {VI_ERROR_LC100_FPGA_COMM_ERR,      "FPGA communications timeout "                     },

   // XSVF errors copied from 'xsvf.c'
   {VI_ERROR_LC100_XSVF_UNKNOWN,       "XSVF Error 1: unknown XSVF error"                 },
   {VI_ERROR_LC100_XSVF_TDOMISMATCH,   "XSVF Error 2: TDO mismatch"                       },
   {VI_ERROR_LC100_XSVF_MAXRETRIES,    "XSVF Error 3: TDO mismatch after max. retries"    },
   {VI_ERROR_LC100_XSVF_ILLEGALCMD,    "XSVF Error 4: illegal XSVF command"               },
   {VI_ERROR_LC100_XSVF_ILLEGALSTATE,  "XSVF Error 5: illegal TAP state"                  },
   {VI_ERROR_LC100_XSVF_DATAOVERFLOW,  "XSVF Error 6: XSVF record length too big"         },

   // LC100 other errors
   {VI_ERROR_LC100_I2C_NACK,           "Internal IIC bus error: NACK"                     },
   {VI_ERROR_LC100_I2C_ERR,            "Internal IIC bus error: ERR"                      },
   {VI_ERROR_LC100_I2C_SIZE,           "Internal IIC bus error: SIZE"                     },
   {VI_ERROR_LC100_FIFOS_NOT_EMPTY,    "Cleared LC100 FIFOs while they were not empty"    },
   {VI_ERROR_LC100_WRONG_OPMODE,       "Command not allowed in this operating mode"       },
   {VI_ERROR_LC100_NVMEM_BAD,          "No response from EEPROM attached to FPGA"         },
   {VI_ERROR_LC100_NVMEM_FIRST,        "This is the first time the LC100 booted!"         },
   {VI_ERROR_LC100_NVMEM_VER_CHANGED,  "NVMEM Version changed since last boot"            },

   {VI_ERROR_LC100_SERVICE_MODE_ONLY,  "Command allowed in service mode only"             },
   {VI_ERROR_LC100_EVAL_BOX,           "Set eval box parameter mismatch"                  },
   {VI_ERROR_LC100_INVALID_USER_DATA,  "Given user data is invalid"                       },

   // LC100 driver errors
   {VI_ERROR_LC100_DRV_INV_USER_DATA,  "Given user data is invalid"                       },
   {VI_ERROR_LC100_NO_USER_DATA,       "No user data available"                           },
   {0 , VI_NULL}  // termination

};





/*===========================================================================
 Prototypes
===========================================================================*/
// Cleanup
static ViStatus initClose (ViSession instr, ViStatus stat);

// NVMEM helper functions
static ViStatus nv_read(ViSession instr, uint16_t adr, void *dat, size_t len, uint16_t index);
static ViStatus nv_write(ViSession instr, uint16_t adr, const void *dat, size_t len, uint16_t index);
static ViStatus nv_read_version(LC100_data_t *data);

// Endianness functions
static ViStatus endian_convert16(uint16_t *data, uint32_t len);

// Wavelength - Pixel - Relation functions
static ViStatus LC100_getUserWavelengthParameters (ViSession instr);
static ViStatus LC100_getFactoryWavelengthParameters (ViSession instr);
static ViStatus LC100_readPolyCoeff   (ViSession instr, int target, LC100_wl_cal_t *cal);
static ViStatus LC100_writePolyCoeff  (ViSession instr, int target, LC100_wl_cal_t *cal);
static ViStatus LC100_readCalPoints   (ViSession instr, int target, ViInt32 pixelDataArray[], ViReal64 wavelengthDataArray[], ViInt32 *bufferLength);
static ViStatus LC100_writeCalPoints  (ViSession instr, int target, ViInt32 pixelDataArray[], ViReal64 wavelengthDataArray[], ViInt32 bufferLength);
static ViStatus LC100_readWLCalValid  (ViSession instr, int target, ViUInt16 *flag);
static ViStatus LC100_writeWLCalValid (ViSession instr, int target, ViUInt16 flag);
static ViStatus LC100_applyPixelWavelengthPoints (LC100_data_t *data, int target, ViInt32 pixelDataArray[], ViReal64 wavelengthDataArray[], ViInt32 bufferLength);
static ViStatus LC100_copyPolyData (LC100_data_t *data, int target, LC100_wl_cal_t *cal);
static ViStatus LC100_checkNodes(ViInt32 pixel[], ViReal64 wl[], ViInt32 cnt);
static ViStatus LC100_nodes2poly(ViInt32 pixel[], ViReal64 wl[], ViInt32 cnt, ViReal64 poly[]);
static ViStatus LC100_poly2wlArray(LC100_wl_cal_t *wl);
static int LeastSquareInterpolation (int * PixelArray, double * WaveLengthArray, int iLength, double Coefficients[]);
static double Summation(double *pdata, int values);
static double SpecSummation(double *pcorrel,int pwr,int values);
static double Spec2Summation(double *pcorrel,int pwr,int values);
static void MergeMC(double s[MATRIX_ROWS][MATRIX_COLS], double i[MATRIX_ROWS], double d[MATRIX_ROWS][MATRIX_COLS], int column);
static double Determinant(double mt[MATRIX_ROWS][MATRIX_COLS]);


/*===========================================================================

 GLOBAL USER-CALLABLE FUNCTION DECLARATIONS (Exportable Functions)

===========================================================================*/
/*===========================================================================

 Init/close

===========================================================================*/
/*---------------------------------------------------------------------------
   Function:   Initialize
   Purpose:    This function initializes the instrument driver session and
               returns an instrument handle which is used in subsequent calls.

   Parameters:

   ViRsrc resourceName:    The visa resource string.
   ViBoolean IDQuery:      Boolean to query the ID or not.
   ViBoolean resetDevice:  Boolean to reset the device or not.
   ViPSession instrHandle: Pointer to opened device.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_init (ViRsrc resourceName, ViBoolean IDQuery, ViBoolean resetDevice, ViSession *pInstr)
{
   ViStatus                err;
   ViSession               rm = VI_NULL;
   ViUInt16                vid, pid;
   LC100_data_t            *data;
   const LC100_sensData_t  *sens;

   //Open instrument session
   *pInstr = VI_NULL;
   if((err = viOpenDefaultRM(&rm))) return (err);
   if((err = viOpen(rm, resourceName, LC100_LOCK_STATE, VI_NULL, pInstr)))
   {
      viClose(rm);
      return (err);
   }
   if((err = viSetAttribute(*pInstr, VI_ATTR_USER_DATA, (ViAttrState)VI_NULL)))
   {
      viClose(*pInstr);
      viClose(rm);
      return (err);
   }

   // Is it a Thorlabs LC100 series device
   if((err = viGetAttribute(*pInstr, VI_ATTR_MANF_ID,    &vid)))  return (initClose(*pInstr, err));
   if((err = viGetAttribute(*pInstr, VI_ATTR_MODEL_CODE, &pid)))  return (initClose(*pInstr, err));

   if(vid != LC100_VID) return (initClose(*pInstr, VI_ERROR_FAIL_ID_QUERY));
   switch(pid)
   {
      case LC100_PID_ILX554B:
         sens = &LC100_sensData_ILX554B;
         break;

      case LC100_PID_RESERVED_1:
      case LC100_PID_RESERVED_2:
      case LC100_PID_RESERVED_3:
      case LC100_PID_RESERVED_4:
      case LC100_PID_RESERVED_5:
      case LC100_PID_RESERVED_6:
      case LC100_PID_RESERVED_7:
         return (initClose(*pInstr, VI_ERROR_NSUP_DEVICE));

      default:
         return (initClose(*pInstr, VI_ERROR_FAIL_ID_QUERY));
   }
   if((err = viFlush (*pInstr, VI_WRITE_BUF_DISCARD | VI_READ_BUF_DISCARD)))     return (initClose(*pInstr, err));
   if ((err = viSetAttribute(*pInstr, VI_ATTR_USB_END_IN, VI_USB_END_NONE)))     return (initClose(*pInstr, err));

   // Private driver data
   if((data         = (LC100_data_t*)malloc(sizeof(LC100_data_t))) == NULL)      return (initClose(*pInstr, VI_ERROR_SYSTEM_ERROR));
   memset(data, 0, sizeof(LC100_data_t));
   if((err = viSetAttribute(*pInstr, VI_ATTR_USER_DATA, (ViAttrState)data)))     return (initClose(*pInstr, err));
   data->instr    = *pInstr;
   data->reset    = resetDevice;
   data->idQuery  = IDQuery;
   data->pid      = pid;
   data->sensData = sens;

   // Read NVMEM version (= version of data format of NVMEM)
   if((err = nv_read_version(data)))                                             return (initClose(*pInstr, err));

   // Reset device
   if((err = viClear(*pInstr)))                                                  return (initClose(*pInstr, err));

   // get wavelength to pixel calculation parameters
   if((err = LC100_getFactoryWavelengthParameters (*pInstr)))                    return (initClose(*pInstr, err));
   if((err = LC100_getUserWavelengthParameters (*pInstr)))                       return (initClose(*pInstr, err));


   
   
   return (VI_SUCCESS);
}

/*---------------------------------------------------------------------------
   Function:   Close
   Purpose:    This function closes an instrument driver session.

   Parameters:

   ViSession instrHandle:  The session to close.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_close (ViSession instrumentHandle)
{
   return (initClose(instrumentHandle, VI_SUCCESS));
}


/*===========================================================================

 Configuration Functions.

===========================================================================*/

#define REG_INT_TIME          12             // R/W integration time (us)
#define REG_INT_TIME_MIN      13             // RO  minimum integration time (us)
#define REG_INT_TIME_MAX      14             // RO  maximum integration time (us)
#define REG_INT_TIME_DEF      15             // RO  default integration time (us)
#define REG_TRG_DELAY_SET     16             // R/W trigger delay
#define REG_TRG_DELAY_MIN     17             // RO minimum trigger delay
#define REG_TRG_DELAY_MAX     18             // RO maximum trigger delay
#define REG_FLASH_DELAY_SET   19             // R/W flash delay
#define REG_FLASH_DELAY_MIN   20             // RO minimum flash delay
#define REG_FLASH_DELAY_MAX   21             // RO maximum flash delay
#define REG_FLASH_DURAT_SET   22             // R/W flash duration
#define REG_FLASH_DURAT_MIN   23             // RO minimum flash duration
#define REG_FLASH_DURAT_MAX   24             // RO maximum flash duration
#define REG_AVG_BITS          25             // R/W averaging bits (0=1x 1=2x ... 9=512x)
#define REG_AVG_BITS_MIN      26             // RO  minimum averaging bits (=0)
#define REG_AVG_BITS_MAX      27             // RO  maximum averaging bits (=9)
#define REG_AVG_BITS_DEF      28             // RO  default averaging bits (=0, no averaging)


/*---------------------------------------------------------------------------
   Function:   Set Integration Time
   Purpose:    This function sets the optical integration time in seconds.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViReal64 integrationTime:  The optical integration time in seconds.
                              Use "LC100_getIntegrationTime" function to
                              retrieve the minimum and maximum integration
                              time.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_setIntegrationTime (ViSession instr, ViReal64 val)
{
   ViStatus err;
   ViUInt32 itime_us;

   // convert the integration time from seconds to micro seconds
   itime_us = (ViUInt32)(val * 1000000.0);

   // then transfer to device
   err = LC100_usbOut(instr, LC100_WCMD_CCD, 0, REG_INT_TIME, sizeof(ViUInt32), (ViBuf)&itime_us);

   return err;
}


/*---------------------------------------------------------------------------
   Function:   Get Integration Time
   Purpose:    This function returns the optical integration time in seconds.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt16 attribute:         The desired integration time. Valid values:
                              LC100_ATTR_ACT   to get the actual value
                              LC100_ATTR_MIN   to get the minimum value
                              LC100_ATTR_MAX   to get the maximum value
                              LC100_ATTR_DEF   to get the default value
   ViPReal64 integrationTime: The integration time.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getIntegrationTime (ViSession instr, ViInt16 attribute, ViReal64 *val)
{
   ViStatus err = VI_SUCCESS;
   ViUInt16 bytes_read;
   ViUInt16 wIndex;
   ViUInt32 itime_us;

   switch(attribute)
   {
      case LC100_ATTR_ACT  :
                              wIndex = REG_INT_TIME;
                              break;
      case LC100_ATTR_MIN  :
                              wIndex = REG_INT_TIME_MIN;
                              break;
      case LC100_ATTR_MAX  :
                              wIndex = REG_INT_TIME_MAX;
                              break;
      case LC100_ATTR_DEF  :
                              wIndex = REG_INT_TIME_DEF;
                              break;
      default              :
                              err = VI_ERROR_PARAMETER2;
                              break;
   }

   // request the data
   if(err == VI_SUCCESS)
   {
      err =    LC100_usbIn(instr,                       LC100_RCMD_CCD,   0,               wIndex,          sizeof(ViUInt32), (ViBuf)&itime_us, &bytes_read);
   }
   if(err == VI_SUCCESS)
   {
      if(bytes_read != sizeof(ViUInt32))  err = VI_ERROR_INV_RESPONSE;
   }

   // check for errors
   if(err != VI_SUCCESS) return (err);

   // calculate the integration time
   if(val)     // check for NULL pointer
   {
      *val = (ViReal64)itime_us / 1000000.0;
   }

   return err;
}


/*---------------------------------------------------------------------------
   Function:   Set Operating Mode
   Purpose:    This function sets the operating mode.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt32 opMode:            The desired operation mode. Valid values:
                              OPMODE_IDLE             - idle mode, camera does nothing
                              OPMODE_SW_SINGLE_SHOT   - waits for a software trigger
                              OPMODE_SW_LOOP          - internally triggermode, as fast as possible
                              OPMODE_HW_SINGLE_SHOT   - waits for a hardware trigger
                              OPMODE_HW_LOOP          - externally triggermode
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_setOperatingMode (ViSession instr, ViInt32 opMode)
{
   ViStatus err;
   ViUInt16 mode;

   mode = (ViUInt16)opMode;
   switch(mode)
   {
      case OPMODE_IDLE           :
      case OPMODE_SW_SINGLE_SHOT :
      case OPMODE_HW_SINGLE_SHOT :
      case OPMODE_SW_LOOP        :
      case OPMODE_HW_LOOP        :
                                    break;
      default                    :
                                    return VI_ERROR_PARAMETER2;
   }

   err = LC100_usbOut(instr, LC100_WCMD_SET_OPMODE, 0, mode, 0, NULL);

   return (err);
}

/*---------------------------------------------------------------------------
   Function:   Get Operating Mode
   Purpose:    This function returns the actual operating mode.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViPInt32 opMode:           The actual operation mode. See "LC100_setOperatingMode"
                              function for valid values.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getOperatingMode (ViSession instr, ViInt32 *opMode)
{
   ViStatus    err      = VI_SUCCESS;
   ViChar      buf[sizeof(ViUInt16)];
   ViUInt16    bytes_read;
   union16_t   t_mode;


   err = LC100_usbIn (instr,  LC100_RCMD_GET_OPMODE, 0, 0, sizeof(ViUInt16), buf, &bytes_read);
   if((bytes_read != sizeof(ViUInt16)) && (!err) ) err = VI_ERROR_INV_RESPONSE;
   if(!err)
   {
      t_mode.two_bytes.hb = buf[0];
      t_mode.two_bytes.lb = buf[1];

      if(opMode) *opMode = (ViInt32)t_mode.one_word;
   }

   return (err);
}



/*---------------------------------------------------------------------------
   Function:   Set Hardware Averaging Mode
   Purpose:    This function sets the Hardware Averaging Mode

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt32 hwAvgMode:         The desired Hardware Averaging Mode.
                              Valid values are 0 to 9 which represent
                              an averaging of 2**0 = 1 to 2**9 = 512 scans 
                              to average

   Note:
   Hardware Averaging works only in continuous operating modes
   OPMODE_SW_LOOP, OPMODE_HW_LOOP, see above
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_setHWAveragingMode (ViSession instr, ViInt32 hwAvgMode)
{
   ViStatus err;
   

            err = LC100_usbOut(instr, LC100_WCMD_CCD, 0, REG_AVG_BITS, sizeof(ViUInt32), (ViBuf)&hwAvgMode);

   //Delay(0.002);
   // reset the device to clear old FIFO contents
   if(!err) err = LC100_usbOut(instr, LC100_WCMD_RESET, 0, 0, 0, NULL);
   // camouflage the FIFO NOT EMPTY ERROR
   if(err == VI_ERROR_LC100_FIFOS_NOT_EMPTY) err = VI_SUCCESS;
      
   return (err);
}

/*---------------------------------------------------------------------------
   Function:   Get Hardware Averaging Mode
   Purpose:    This function returns the Hardware Averaging Mode

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt16 attribute:         The desired averaging mode. Valid values:
                              LC100_ATTR_ACT   to get the actual value
                              LC100_ATTR_MIN   to get the minimum value
                              LC100_ATTR_MAX   to get the maximum value
                              LC100_ATTR_DEF   to get the default value
   ViPInt32 hwAvgMode:        The desired Hardware Averaging Mode.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getHWAveragingMode (ViSession instr, ViInt16 attribute, ViInt32 *hwAvgMode)
{
   ViStatus    err      = VI_SUCCESS;
   ViChar      buf[sizeof(ViUInt32)];
   ViUInt16    bytes_read;
   ViUInt16    wIndex;
   union32_t   t_mode;

   switch(attribute)
   {
      case LC100_ATTR_ACT  :
                              wIndex = REG_AVG_BITS;
                              break;
      case LC100_ATTR_MIN  :
                              wIndex = REG_AVG_BITS_MIN;
                              break;
      case LC100_ATTR_MAX  :
                              wIndex = REG_AVG_BITS_MAX;
                              break;
      case LC100_ATTR_DEF  :
                              wIndex = REG_AVG_BITS_DEF;
                              break;
      default              :
                              err = VI_ERROR_PARAMETER2;
                              break;
   }

   // request the data
   if(err == VI_SUCCESS)
   {
      err = LC100_usbIn (instr,  LC100_RCMD_CCD, 0, wIndex, sizeof(ViUInt32), buf, &bytes_read);
   }
   if(err == VI_SUCCESS)
   {
      if(bytes_read != sizeof(ViUInt32)) err = VI_ERROR_INV_RESPONSE;
   }
   
   // check for errors
   if(err != VI_SUCCESS) return (err);

   if(hwAvgMode)
   {
      t_mode.four_bytes.hhb = buf[0];
      t_mode.four_bytes.hlb = buf[1];
      t_mode.four_bytes.lhb = buf[2];
      t_mode.four_bytes.llb = buf[3];

      *hwAvgMode = (ViInt32)t_mode.one_dword;
   }

   return (VI_SUCCESS);
}



/*---------------------------------------------------------------------------
   Function:   Set Intensity Correction
   Purpose:    This function sets intensity correction values. Each value represents
               the multiplier for the corresponding pixel.

   Parameters:

   ViSession instrHandle:        The actual session to opened device.
   ViReal64 correctionValues[]:  The intensity correction values array
                                 (LC100_NUM_PIXELS elements).
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_setIntensityCorr (ViSession instr, ViReal64 arry[])
{
   return (VI_SUCCESS);
}

/*---------------------------------------------------------------------------
   Function:   Get Intensity Correction
   Purpose:    This function returns the intensity correction values. Each value represents
               the multiplier for the corresponding pixel.

   Parameters:

   ViSession instrHandle:        The actual session to opened device.
   ViReal64 correctionValues[]:  The intensity correction values array
                                 (LC100_NUM_PIXELS elements).
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getIntensityCorr (ViSession instr, ViReal64 arry[])
{
   return (VI_SUCCESS);
}

/*---------------------------------------------------------------------------
   Function:   Reset Intensity Correction
   Purpose:    This function reset the intensity correction.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_resetIntensityCorr (ViSession instr)
{
   return (VI_SUCCESS);
}



/*---------------------------------------------------------------------------
   Function:   Set Evaluation Box
   Purpose:    This function sets the range/interval of a specified evaluation box.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt16 EvalBox:           The desired evaluation box (0 to LC100_NUM_EVALBOX).
   ViInt32 x1:                The start pixel.
   ViInt32 x2:                The end pixel.
   ViReal64 y1:               The lower level.
   ViReal64 y2:               The upper level.
---------------------------------------------------------------------------*/
#define CMD_SET_EVALBOX_BUF_SIZE    (4*sizeof(ViUInt16))

ViStatus _VI_FUNC LC100_setEvalBox (ViSession instr, ViInt16 num, ViInt32 x1, ViInt32 x2, ViReal64 y1, ViReal64 y2)
{
   ViStatus       err      = VI_SUCCESS;
   LC100_data_t   *data;
   union16_t      lc100_x1, lc100_x2, lc100_y1, lc100_y2;
   unsigned char  buf[CMD_SET_EVALBOX_BUF_SIZE];

   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   // check if parameters are in allowed range
   if((num < 0  ) || (num          >= LC100_NUM_EVALBOX))      return  VI_ERROR_PARAMETER2;
   if((x1  < 0  ) || ((ViUInt16)x1 >= data->sensData->numPix)) return  VI_ERROR_PARAMETER3;
   if((x2  < 0  ) || ((ViUInt16)x2 >= data->sensData->numPix)) return  VI_ERROR_PARAMETER4;
   if((y1  < 0.0) || (y1           >  1.0))                    return  VI_ERROR_PARAMETER5;
   if((y2  < 0.0) || (y2           >  1.0))                    return  VI_ERROR_PARAMETER6;

   // use unions to pick bytes from multi-byte variables
   lc100_x1.one_word = (ViUInt16)x1;
   lc100_x2.one_word = (ViUInt16)x2;
   lc100_y1.one_word = (ViUInt16)(y1 * LC100_REAL_TO_UINT16);
   lc100_y2.one_word = (ViUInt16)(y2 * LC100_REAL_TO_UINT16);

   buf[0] = lc100_x1.two_bytes.hb;
   buf[1] = lc100_x1.two_bytes.lb;
   buf[2] = lc100_x2.two_bytes.hb;
   buf[3] = lc100_x2.two_bytes.lb;

   buf[4] = lc100_y1.two_bytes.hb;
   buf[5] = lc100_y1.two_bytes.lb;
   buf[6] = lc100_y2.two_bytes.hb;
   buf[7] = lc100_y2.two_bytes.lb;

   err = LC100_usbOut(instr, LC100_WCMD_WRITE_EVALBOX, 0, (ViUInt16)num, CMD_SET_EVALBOX_BUF_SIZE, buf);

   return (err);
}

/*---------------------------------------------------------------------------
   Function:   Get Evaluation Box
   Purpose:    This function returns the range/interval of a specified evaluation box.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt16 EvalBox:           The desired evaluation box (0 to LC100_NUM_EVALBOX).
   ViPInt32 x1:               The start pixel.
   ViPInt32 x2:               The end pixel.
   ViPReal64 y1:              The lower level.
   ViPReal64 y2:              The upper level.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getEvalBox (ViSession instr, ViInt16 EvalBox, ViPInt32 x1, ViPInt32 x2, ViPReal64 y1, ViPReal64 y2)
{
   ViStatus       err      = VI_SUCCESS;
   LC100_data_t   *data;
   union16_t      lc100_x1, lc100_x2, lc100_y1, lc100_y2;
   unsigned char  buf[CMD_SET_EVALBOX_BUF_SIZE];
   ViUInt16       bytes_read;

   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   // check if parameter 'EvalBox' is in allowed range
   if((EvalBox < 0  ) || (EvalBox >= LC100_NUM_EVALBOX))             return  VI_ERROR_PARAMETER2;

   err = LC100_usbIn(instr, LC100_WCMD_WRITE_EVALBOX, 0, (ViUInt16)EvalBox, CMD_SET_EVALBOX_BUF_SIZE, buf, &bytes_read);
   if((bytes_read != CMD_SET_EVALBOX_BUF_SIZE) && (!err) ) err = VI_ERROR_INV_RESPONSE;


   // use unions to pack bytes to multi-byte variables
   lc100_x1.two_bytes.hb = buf[0];
   lc100_x1.two_bytes.lb = buf[1];
   lc100_x2.two_bytes.hb = buf[2];
   lc100_x2.two_bytes.lb = buf[3];

   lc100_y1.two_bytes.hb = buf[4];
   lc100_y1.two_bytes.lb = buf[5];
   lc100_y2.two_bytes.hb = buf[6];
   lc100_y2.two_bytes.lb = buf[7];

   if(x1 != NULL) *x1 = (ViInt32)lc100_x1.one_word;
   if(x2 != NULL) *x2 = (ViInt32)lc100_x2.one_word;
   if(y1 != NULL) *y1 = (ViReal64)lc100_y1.one_word  * LC100_UINT16_TO_REAL;
   if(y2 != NULL) *y2 = (ViReal64)lc100_y2.one_word  * LC100_UINT16_TO_REAL;


   return (err);
}


/*---------------------------------------------------------------------------
   Function:   Set GPIO Feed
   Purpose:    This function sets the GPIO mapping.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt16 GPIO:              The GPIO number (0 to LC100_NUM_GPIO - 1).
   ViInt16 feed:              The desired feed.
                              Range 0 to LC100_NUM_EVALBOX - 1 (0 - 15) correlates to eval
                              boxes (e.g. 0 means eval box 1, 4 means eval box 5 and so on).
                              Range LC100_NUM_EVALBOX to LC100_NUM_EVALBOX + LC100_NUM_GPIO - 1 (16 - 20)
                              correlates to additional GPIO inputs (e.g. 16 means GPIO1 and so on).
   ViInt16 mode:              GPIO_FEED_MODE_IGNORE to GPIO_FEED_MODE_LCH are
                              correlated to evaluation boxes.
                              GPIO_FEED_MODE_IGNORE, GPIO_FEED_GPIN_NONINV and GPIO_FEED_GPIN_INV
                              are correlated to addiitonal GPIO inputs.
                              See macro definitions for detailed information.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_setGPIOFeed (ViSession instr, ViInt16 GPIO, ViInt16 feed, ViInt16 mode)
{
   ViStatus    err      = VI_SUCCESS;
   ViUInt16    index;

   // check if parameters are in allowed range
   if(GPIO >=                      LC100_NUM_GPIO )   return  VI_ERROR_PARAMETER2;
   if(feed >= (LC100_NUM_EVALBOX + LC100_NUM_GPIO))   return  VI_ERROR_PARAMETER3;

   index = (ViUInt16)feed + 256*(ViUInt16)GPIO;

   if(feed < LC100_NUM_EVALBOX)
   {
      switch(mode)
      {
         case GPIO_FEED_MODE_IGNORE       :
         case GPIO_FEED_MODE_L            :
         case GPIO_FEED_MODE_LC           :
         case GPIO_FEED_MODE_C            :
         case GPIO_FEED_MODE_CH           :
         case GPIO_FEED_MODE_H            :
         case GPIO_FEED_MODE_LCH          :
                                             break;
         default                          :
                                             return VI_ERROR_PARAMETER4;
      }
   }
   else
   {
      switch(mode)
      {
         case GPIO_FEED_MODE_IGNORE       :
         case GPIO_FEED_GPIN_NONINV       :
         case GPIO_FEED_GPIN_INV          :
                                             break;
         default                          :
                                             return VI_ERROR_PARAMETER4;
      }
   }
   err = LC100_usbOut(instr, LC100_WCMD_GPIO_FEED, (ViUInt16)mode, index, 0, NULL);

   return (err);
}

/*---------------------------------------------------------------------------
   Function:   Get GPIO Feed
   Purpose:    This function returns the GPIO mapping.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt16 GPIO:              The GPIO number (0 to LC100_NUM_GPIO - 1).
   ViInt16 feed:              The desired feed.
   ViPInt16 mode:             The feed mode.

   See "LC100_setGPIOFeed" function for more information.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getGPIOFeed (ViSession instr, ViInt16 GPIO, ViInt16 feed, ViInt16 *mode)
{
   ViStatus    err      = VI_SUCCESS;
   ViUInt16    index;
   ViChar      buf[sizeof(ViInt16)];
   ViUInt16    bytes_read;
   union16_t   t_mode;

   // check if parameters are in allowed range
   if(GPIO >=                      LC100_NUM_GPIO )   return  VI_ERROR_PARAMETER2;
   if(feed >= (LC100_NUM_EVALBOX + LC100_NUM_GPIO))   return  VI_ERROR_PARAMETER3;

   index = (ViUInt16)feed + 256*(ViUInt16)GPIO;


   err = LC100_usbIn (instr, LC100_RCMD_GPIO_FEED, 0, index, sizeof(ViInt16), buf, &bytes_read);
   if((bytes_read != sizeof(ViInt16)) && (!err) ) err = VI_ERROR_INV_RESPONSE;
   if(!err)
   {
      t_mode.two_bytes.hb = buf[0];
      t_mode.two_bytes.lb = buf[1];

      if(mode) *mode = t_mode.one_word;
   }

   // DebugPrintf("LC100_getGPIOFeed(err=%08X) HB=%02X  LB=%02X  mode=%04X\n", err, buf[0], buf[1], *mode);

   return (err);
}

/*---------------------------------------------------------------------------
   Function:   Set GPIO Mode
   Purpose:    This function sets the GPIO mode.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt16 GPIO:              The GPIO number (0 to LC100_NUM_GPIO - 1).
   ViInt16 mode:              The GPIO mode. See GPIO mode macros for
                              detailed information.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_setGPIOMode (ViSession instr, ViInt16 GPIO, ViInt16 mode)
{
   ViStatus    err      = VI_SUCCESS;
   ViUInt16    index;

   // check if parameters are in allowed range
   if(GPIO > LC100_NUM_GPIO)  return  VI_ERROR_PARAMETER2;

// index = (ViUInt16)evalBox + 256*(ViUInt16)GPIO;
   index =        0          + 256*(ViUInt16)GPIO;

   switch(mode)
   {
      case GPIO_MODE_INPUT             :
      case GPIO_MODE_OUTPUT_POS        :
      case GPIO_MODE_OUTPUT_NEG        :
      case GPIO_MODE_OUT_EVAL_POS      :
      case GPIO_MODE_OUT_EVAL_NEG      :
      case GPIO_MODE_OUT_EXPOSURE_POS  :
      case GPIO_MODE_OUT_EXPOSURE_NEG  :
      case GPIO_MODE_OUT_FLASH_POS     :
      case GPIO_MODE_OUT_FLASH_NEG     :

                                          break;
      default                          :
                                          return VI_ERROR_PARAMETER3;
   }

   err = LC100_usbOut(instr, LC100_WCMD_GPIO_MODE, (ViUInt16)mode, index, 0, NULL);

   return (err);
}

/*---------------------------------------------------------------------------
   Function:   Get GPIO Mode
   Purpose:    This function returns the GPIO mode.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt16 GPIO:              The GPIO number (0 to LC100_NUM_GPIO - 1).
   ViPInt16 mode:             The GPIO mode. See GPIO mode macros for
                              detailed information.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getGPIOMode (ViSession instr, ViInt16 GPIO, ViInt16 *mode)
{
   ViStatus    err      = VI_SUCCESS;
   ViUInt16    index;
   ViChar      buf[sizeof(ViInt16)];
   ViUInt16    bytes_read;
   union16_t   t_mode;

   // check if parameters are in allowed range
   if(GPIO >= LC100_NUM_GPIO) return  VI_ERROR_PARAMETER2;

// index = (ViUInt16)evalBox + 256*(ViUInt16)GPIO;
   index =        0          + 256*(ViUInt16)GPIO;


   err = LC100_usbIn (instr, LC100_RCMD_GPIO_MODE, 0, index, sizeof(ViInt16), buf, &bytes_read);
   if((bytes_read != sizeof(ViInt16)) && (!err) ) err = VI_ERROR_INV_RESPONSE;
   if(!err)
   {
      t_mode.two_bytes.hb = buf[0];
      t_mode.two_bytes.lb = buf[1];

      if(mode) *mode = t_mode.one_word;
   }

   return (err);
}

/*---------------------------------------------------------------------------
 Get state of GPIOs (read the input value of pin)
 when given parameter GPIO is in range 0...4 (for GPIO 1...5)
 the function will return 0 or 1 corresponding to the pin state
 when given parameter GPIO is another value
 the function will return a value in the range of 0...31
 corresponding to the values of all 5 GPIO inputs
 state = GPIO1 + 2*GPIO2 + 4*GPIO3 + 8*GPIO3 + 16*GPIO4
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getGPIOState (ViSession instr, ViInt16 GPIO, ViUInt32 *state)
{
   ViStatus    err      = VI_SUCCESS;
   ViUInt16    index;
   ViChar      buf[sizeof(ViUInt32)];
   ViUInt16    bytes_read;
   union32_t   t_state;

// index = 0          + 256*(ViUInt16)GPIO;
   index = 0;


   err = LC100_usbIn (instr, LC100_RCMD_GPIO_STATE, 0, index, sizeof(ViUInt32), buf, &bytes_read);
   if((bytes_read != sizeof(ViUInt32)) && (!err) ) err = VI_ERROR_INV_RESPONSE;
   // check if we have no error AND given state is NOT NULL
   if(!err && state)
   {
      t_state.four_bytes.hhb = buf[0];
      t_state.four_bytes.hlb = buf[1];
      t_state.four_bytes.lhb = buf[2];
      t_state.four_bytes.llb = buf[3];

      // preset value, change it later when necessary
      *state = 0;
      switch(GPIO)
      {
         case 0   :  if(t_state.one_dword & 0x00000001)  *state = 1; break;
         case 1   :  if(t_state.one_dword & 0x00000002)  *state = 1; break;
         case 2   :  if(t_state.one_dword & 0x00000004)  *state = 1; break;
         case 3   :  if(t_state.one_dword & 0x00000008)  *state = 1; break;
         case 4   :  if(t_state.one_dword & 0x00000010)  *state = 1; break;
         default  :  *state = (t_state.one_dword & 0x0000001F);      break;
      }
   }

   return (err);
}

/*---------------------------------------------------------------------------
   Function:   Set Analog Output Mode
   Purpose:    This function sets the mode of the analog output.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViUInt32  mode:            The analog output mode. It can be either set to
                              a fixed value (LC100_AOUT_MODE_FIXED) or it can
                              be configured as a variable output (LC100_AOUT_MODE_VARIABLE).
                              See macros below for further information.
---------------------------------------------------------------------------*/

#define LC100_CMD_AOUT_VOLT      0
#define LC100_CMD_AOUT_MODE      1
#define LC100_CMD_AOUT_PIXEL     2

#define CMD_AOUT_PIXEL_LEN       (sizeof(ViUInt16) + 2*sizeof(ViUInt32))

ViStatus _VI_FUNC LC100_setAnalogOutputMode (ViSession instr, ViUInt32 mode)
{
   ViStatus    err      = VI_SUCCESS;

   err = LC100_usbOut(instr, LC100_WCMD_AOUT, 0, LC100_CMD_AOUT_MODE, sizeof(ViUInt32), (ViBuf)&mode);

   return (err);
}

/*---------------------------------------------------------------------------
   Function:   Get Analog Output Mode
   Purpose:    This function returns the mode of the analog output.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViPUInt32  mode:           The analog output mode. See "LC100_setAnalogOutputMode"
                              function for more information.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getAnalogOutputMode (ViSession instr, ViUInt32 *mode)
{
   ViStatus    err   = VI_SUCCESS;
   ViChar      buf[sizeof(ViUInt32)];
   ViUInt16    bytes_read;
   union32_t   t_result;


   err = LC100_usbIn (instr, LC100_RCMD_AOUT, 0, LC100_CMD_AOUT_MODE, sizeof(ViUInt32), buf, &bytes_read);
   if((bytes_read != sizeof(ViUInt32)) && (!err) ) err = VI_ERROR_INV_RESPONSE;

   if(!err)
   {
      t_result.four_bytes.hhb = buf[0];
      t_result.four_bytes.hlb = buf[1];
      t_result.four_bytes.lhb = buf[2];
      t_result.four_bytes.llb = buf[3];

      if(mode)
      {
         *mode = t_result.one_dword;
      }
   }

   return (err);
}

/*---------------------------------------------------------------------------
   Function:   Set Analog Output Voltage
   Purpose:    This function sets either the fixed analog output voltage or
               the lower and upper limit for the analog output voltage when
               a pixel intensity is monitored.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt16 select:            The voltage to set. Valid values are:
                              LC100_AOUT_SET_VOLT, LC100_AOUT_VOLT_IMIN and
                              LC100_AOUT_VOLT_IMAX. See macros below for more
                              information.
   ViReal64 voltage:          The voltage in Volt.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_setAnalogOutputVoltage (ViSession instr, ViInt16 select, ViReal64 voltage)
{
   ViStatus    err      = VI_SUCCESS;
   ViUInt32    voltage_ui32;


   switch(select)
   {
      case LC100_AOUT_SET_VOLT         :
      case LC100_AOUT_VOLT_IMIN        :
      case LC100_AOUT_VOLT_IMAX        :
                                          break;
      default                          :
                                          return VI_ERROR_PARAMETER2;
   }

   if(voltage < 0.0) return VI_ERROR_PARAMETER3;
   voltage_ui32 = (ViUInt32)(voltage * 1000000.0);    // we actually transmit uVolts
   err = LC100_usbOut(instr, LC100_WCMD_AOUT, select, LC100_CMD_AOUT_VOLT, sizeof(ViUInt32), (ViBuf)&voltage_ui32);

   return (err);
}

/*---------------------------------------------------------------------------
   Function:   Get Analog Output Voltage
   Purpose:    This function returns a specified analog output voltage.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt16 select:            The voltage to get. See "LC100_setAnalogOutputVoltage"
                              function for more information.
   ViPReal64 voltage:         The voltage in Volt.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getAnalogOutputVoltage (ViSession instr, ViInt16 select, ViReal64 *voltage)
{
   ViStatus    err   = VI_SUCCESS;
   ViChar      buf[sizeof(ViUInt32)];
   ViUInt16    bytes_read;
   union32_t   t_result;

   switch(select)
   {
      case LC100_AOUT_SET_VOLT   :
      case LC100_AOUT_MIN_VOLT   :
      case LC100_AOUT_MAX_VOLT   :
      case LC100_AOUT_VOLT_IMIN  :
      case LC100_AOUT_VOLT_IMAX  :  break;
      default                    :  return VI_ERROR_PARAMETER2;
   }

   err = LC100_usbIn (instr, LC100_RCMD_AOUT, select, LC100_CMD_AOUT_VOLT, sizeof(ViUInt32), buf, &bytes_read);
   if((bytes_read != sizeof(ViUInt32)) && (!err) ) err = VI_ERROR_INV_RESPONSE;

   if(!err)
   {
      t_result.four_bytes.hhb = buf[0];
      t_result.four_bytes.hlb = buf[1];
      t_result.four_bytes.lhb = buf[2];
      t_result.four_bytes.llb = buf[3];

      if(voltage)
      {
         *voltage = (ViReal64)t_result.one_dword / 1000000.0;
      }
   }

   return (err);
}

/*---------------------------------------------------------------------------
   Function:   Set Analog Output Pixel
   Purpose:    This function sets the pixel to be monitored and the lower
               and upper intensitiy limit.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt16 pixel  :           The pixel to be monitored (0 to LC100_NUM_PIXELS -1).
   ViReal64  imax:            The maximum intensity (all values above will result to output LC100_AOUT_VOLT_IMAX at analog output)
   ViReal64  imin:            The minimum intensity (all values below will result to output LC100_AOUT_VOLT_IMIN at analog output)
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_setAnalogOutputPixel (ViSession instr, ViInt16 pixel, ViReal64 imax, ViReal64 imin)
{
   ViStatus    err      = VI_SUCCESS;
   ViChar      buf[CMD_AOUT_PIXEL_LEN];      // buf[10]
   ViUInt32    intensity_u32[2];

   intensity_u32[0] = (ViUInt32)(imax * 1000000.0);
   intensity_u32[1] = (ViUInt32)(imin * 1000000.0);

   memcpy( buf,    (void *)&pixel,            sizeof(ViInt16) );
   memcpy(&buf[2], (void *)intensity_u32, (2*sizeof(ViUInt32)));

   err = LC100_usbOut(instr, LC100_WCMD_AOUT, 0, LC100_CMD_AOUT_PIXEL, CMD_AOUT_PIXEL_LEN, buf);

   return (err);
}

/*---------------------------------------------------------------------------
   Function:   Get Analog Output Pixel
   Purpose:    This function returns the pixel which is monitored and the lower
               and upper intensitiy limit.

   Note:
      If you do not need either of these values you may pass NULL.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViPInt16 pixel :           The actual monitored pixel.
   ViPReal64  imax:           The maximum intensity.
   ViPReal64  imin:           The minimum intensity.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getAnalogOutputPixel (ViSession instr, ViInt16 *pixel, ViReal64 *imax, ViReal64 *imin)
{
   ViStatus    err   = VI_SUCCESS;
   ViChar      buf[CMD_AOUT_PIXEL_LEN];
   ViUInt16    bytes_read;
   union16_t   t_result16;
   union32_t   t_result32;

   err = LC100_usbIn (instr, LC100_RCMD_AOUT, 0, LC100_CMD_AOUT_PIXEL, CMD_AOUT_PIXEL_LEN, buf, &bytes_read);
   if((bytes_read != CMD_AOUT_PIXEL_LEN) && (!err) ) err = VI_ERROR_INV_RESPONSE;

   if(!err)
   {
      if(pixel)
      {
         t_result16.two_bytes.hb = buf[0];
         t_result16.two_bytes.lb = buf[1];
         *pixel = (ViInt16)t_result16.one_word;
      }

      if(imax)
      {
         t_result32.four_bytes.hhb = buf[2];
         t_result32.four_bytes.hlb = buf[3];
         t_result32.four_bytes.lhb = buf[4];
         t_result32.four_bytes.llb = buf[5];

         *imax = (ViReal64)t_result32.one_dword / 1000000.0;
      }

      if(imin)
      {
         t_result32.four_bytes.hhb = buf[6];
         t_result32.four_bytes.hlb = buf[7];
         t_result32.four_bytes.lhb = buf[8];
         t_result32.four_bytes.llb = buf[9];

         *imin = (ViReal64)t_result32.one_dword / 1000000.0;
      }
   }

   return (err);
}



/*---------------------------------------------------------------------------
   Function:   Set Trigger Delay
   Purpose:    This function sets the trigger delay in seconds.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViReal64 delay:            The trigger delay in seconds. The
                              allowed range for this parameter can be queried by
                              "LC100_getTriggerDelay".
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_setTriggerDelay (ViSession instrHandle, ViReal64 delay)
{
   ViStatus    err   = VI_SUCCESS;
   ViUInt32    value = 0;

   // calculate the number of clock periods for the selected time
   value = (ViUInt32)(delay / LC100_CLOCK_PERIOD);

   // send buffer to device
   err = LC100_usbOut(instrHandle, LC100_WCMD_CCD, 0, REG_TRG_DELAY_SET, sizeof(ViUInt32), (ViBuf)&value);

   return err;
}


/*---------------------------------------------------------------------------
   Function:   Get Trigger Delay
   Purpose:    This function queries the trigger delay.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt16 select:            The trigger delay to retrieve. See macros below
                              for more information.
   ViPReal64 delay:           The trigger delay in seconds.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getTriggerDelay (ViSession instrHandle, ViInt16 select, ViPReal64 delay)
{
   ViStatus    err = VI_SUCCESS;
   ViUInt16    bytes_read = 0;
   ViUInt16    wIndex = REG_TRG_DELAY_SET;
   ViUInt32    tmp = 0;

   // check select variable
   switch(select)
   {
      case LC100_TRIG_DELAY_SET   : wIndex = REG_TRG_DELAY_SET;
                                    break;
      case LC100_TRIG_DELAY_MIN   : wIndex = REG_TRG_DELAY_MIN;
                                    break;
      case LC100_TRIG_DELAY_MAX   : wIndex = REG_TRG_DELAY_MAX;
                                    break;
      default                     : return VI_ERROR_PARAMETER2;
   }

   // request selected trigger delay
   err = LC100_usbIn(instrHandle, LC100_RCMD_CCD, 0, wIndex, sizeof(ViUInt32), (ViBuf)&tmp, &bytes_read);
   if((bytes_read != sizeof(ViUInt32)) && (!err) ) err = VI_ERROR_INV_RESPONSE;

   if((!err) && (delay))
   {
      *delay = (ViReal64)tmp * LC100_CLOCK_PERIOD;
   }

   return err;
}


/*---------------------------------------------------------------------------
   Function:   Set Flash Parameters
   Purpose:    This function sets the flash parameters (duration and delay).

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt16 select:            Either LC100_FLASH_DURATION_SET to set the flash
                              duration or LC100_FLASH_DELAY_SET to set the flash
                              delay. See macros below for more information.
   ViReal64 value:            The flash duration or delay in seconds. The allowed
                              range for these parameters can be queried by
                              "LC100_getFlashParams".
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_setFlashParams (ViSession instrHandle, ViInt16 select, ViReal64 value)
{
   ViStatus    err   = VI_SUCCESS;
   ViUInt16    wIndex = LC100_FLASH_DELAY_SET;
   ViUInt32    tmp = 0;

   switch(select)
   {
      case LC100_FLASH_DELAY_SET    :  wIndex = REG_FLASH_DELAY_SET;
                                       break;
      case LC100_FLASH_DURATION_SET :  wIndex = REG_FLASH_DURAT_SET;
                                       break;
      default                       :  return VI_ERROR_PARAMETER2;
   }

   // calculate the number of clock periods for the selected time
   tmp = (ViUInt32)(value / LC100_CLOCK_PERIOD);

   // send buffer to device
   err = LC100_usbOut(instrHandle, LC100_WCMD_CCD, 0, wIndex, sizeof(ViUInt32), (ViBuf)&tmp);

   return err;
}


/*---------------------------------------------------------------------------
   Function:   Get Flash Parameters
   Purpose:    This function queries the flash parameters.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt16 select:            The flash parameter to query. See "LC100_setFlashParams"
                              function for more information about valid values.
   ViPReal64 value:           The requested flash parameter in seconds.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getFlashParams (ViSession instrHandle, ViInt16 select, ViPReal64 value)
{
   ViStatus    err = VI_SUCCESS;
   ViUInt16    bytes_read = 0;
   ViUInt16    wIndex = REG_FLASH_DELAY_SET;
   ViUInt32    tmp = 0;

   // check select variable
   switch(select)
   {
      case LC100_FLASH_DELAY_SET   :   wIndex = REG_FLASH_DELAY_SET;
                                       break;
      case LC100_FLASH_DELAY_MIN   :   wIndex = REG_FLASH_DELAY_MIN;
                                       break;
      case LC100_FLASH_DELAY_MAX   :   wIndex = REG_FLASH_DELAY_MAX;
                                       break;
      case LC100_FLASH_DURATION_SET:   wIndex = REG_FLASH_DURAT_SET;
                                       break;
      case LC100_FLASH_DURATION_MIN:   wIndex = REG_FLASH_DURAT_MIN;
                                       break;
      case LC100_FLASH_DURATION_MAX:   wIndex = REG_FLASH_DURAT_MAX;
                                       break;
      default                     :    return VI_ERROR_PARAMETER2;
   }

   // request selected trigger delay
   err = LC100_usbIn(instrHandle, LC100_RCMD_CCD, 0, wIndex, sizeof(ViUInt32), (ViBuf)&tmp, &bytes_read);
   if((bytes_read != sizeof(ViUInt32)) && (!err) ) err = VI_ERROR_INV_RESPONSE;

   if((!err) && (value))
   {
      *value = (ViReal64)tmp * LC100_CLOCK_PERIOD;
   }

   return err;
}


/*===========================================================================

 Class: Action/Status Functions.

===========================================================================*/
/*---------------------------------------------------------------------------
   Function:   Start Scan
   Purpose:    This function triggers the the camera to take one single scan.

   Note:
   The scan data can be read out with the function 'Get Scan Data'
   Use 'Get Device Status' to check the scan status.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
---------------------------------------------------------------------------*/
// macros for exec scan
#define EXEC_SCAN_SINGLE               0
#define EXEC_SCAN_CONT                 1
#define EXEC_SCAN_EXT_TRIG             2
#define EXEC_SCAN_EXT_TRIG_CONT        3
#define EXEC_SCAN_CANCEL_TRIGGER       4

ViStatus _VI_FUNC LC100_startScan (ViSession instr)
{
   ViStatus err;

   err = LC100_usbOut(instr, LC100_WCMD_EXEC_SCAN, 0, EXEC_SCAN_SINGLE, 0, NULL);

   return (err);
}

/*---------------------------------------------------------------------------
   Function:   Start Continuous Scan
   Purpose:    This function starts the free running mode.

   Note:
   The scan data can be read out with the function 'Get Scan Data'
   Use 'Get Device Status' to check the scan status.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_startScanCont (ViSession instr)
{
   ViStatus err;

   err = LC100_usbOut(instr, LC100_WCMD_EXEC_SCAN, 0, EXEC_SCAN_CONT, 0, NULL);

   return (err);
}

/*---------------------------------------------------------------------------
   Function:   Start Extern Scan
   Purpose:    This function arms the external trigger of the camera. A
               following low to high transition at the trigger input of the
               camera then starts a scan.

   Note:
   The scan data can be read out with the function 'Get Scan Data'
   Use 'Get Device Status' to check the scan status.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_startScanExtTrg (ViSession instr)
{
   ViStatus err;

   err = LC100_usbOut(instr, LC100_WCMD_EXEC_SCAN, 0, EXEC_SCAN_EXT_TRIG, 0, NULL);

   return (err);
}

/*---------------------------------------------------------------------------
   Function:   Start Continuous Extern Scan
   Purpose:    This function arms the camera for scanning after an external
               trigger. A following low to high transition at the trigger input
               starts a scan. The camera will rearm immediately after the scan
               data is readout.

   Note:
   The scan data can be read out with the function 'Get Scan Data'
   Use 'Get Device Status' to check the scan status.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_startScanContExtTrg (ViSession instr)
{
   ViStatus err;

   err = LC100_usbOut(instr, LC100_WCMD_EXEC_SCAN, 0, EXEC_SCAN_EXT_TRIG_CONT, 0, NULL);

   return (err);
}

/*---------------------------------------------------------------------------
 Function:     Get Device Status
 Purpose:      This function returns 32bit status information.

Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViPInt32 deviceStatus:     The status register.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getDeviceStatus (ViSession instr, ViInt32 *deviceStatus)
{
   ViStatus    err;
   ViChar      buf[sizeof(ViUInt32)];
   ViUInt16    bytes_read;
   union32_t   t_status;


   err = LC100_usbIn (instr,  LC100_RCMD_GET_STATUS, 0, 0, sizeof(ViUInt32), buf, &bytes_read);
   if((bytes_read != sizeof(ViUInt32)) && (!err) ) err = VI_ERROR_INV_RESPONSE;
   if(!err)
   {
      t_status.four_bytes.hhb = buf[0];
      t_status.four_bytes.hlb = buf[1];
      t_status.four_bytes.lhb = buf[2];
      t_status.four_bytes.llb = buf[3];

      if(deviceStatus) *deviceStatus = (ViInt32)t_status.one_dword;
   }

   return (err);
}


/*===========================================================================

 Class: Data Functions.
 
===========================================================================*/

/*---------------------------------------------------------------------------
   Function:   Setup
   Purpose:    This function makes the LC100 to save a setup to the non
               volatile memory, resets to factory default settings, etc.
               depending on the parameter 'mode'

   Parameters:

   ViSession instr:           The actual session to opened device.
   ViUInt16  mode:            This parameter determines what setup operation
                              in the LC100 shall be done, see macros below
   ViUInt16  filter:          This parameter determines what part of the setup
                              will be affected, see macros below. For all part2
                              at once use the macro FILTER_USER_SETUP, you can
                              logically or the filters to work on more than one
                              filter at a time
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_Setup (ViSession instr, ViUInt16 mode, ViUInt16 filter)
{
   ViStatus          err = VI_SUCCESS;        // error level
   LC100_data_t      *data;
   ViUInt16          wValue;
   
   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   switch(mode)
   {
      case LC100_SETUP_STORE_RAM :  wValue = NVMEM_STORE_RAM;           break;
      case LC100_SETUP_RESET_RAM :  wValue = NVMEM_STORE_RESET_RAM;     break;
      case LC100_SETUP_RESET_NV  :  wValue = NVMEM_STORE_RESET_EEPROM;  break;
      case LC100_SETUP_RELOAD    :  wValue = NVMEM_STORE_RELOAD;        break;
      default                    :  return VI_ERROR_INV_PARAMETER;
   }

   // check parameter filter, only lowest five bits (0x1F) are allowed
   if(filter > FILTER_USER_SETUP)
   {
      VI_ERROR_INV_PARAMETER;
   }
   
   // shift left 'filter'
   wValue |= (filter << 8);
   err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, wValue, LC100_NVMEM_STORE, 0, NULL);
   return (err);
    
}

/*---------------------------------------------------------------------------
   Function:   Get Scan Data
   Purpose:    This function reads out the processed scan data.

   Parameters:

   ViSession instr:           The actual session to opened device.
   ViUInt16 data[]:           The measurement array (LC100_NUM_PIXELS elements).
                              Each pixel can have a value from 0 to 65535.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getScanData (ViSession instr, ViUInt16 _VI_FAR scanData[])
{
   ViStatus          err = VI_SUCCESS;        // error level
   LC100_data_t      *data;

   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   err = LC100_USB_read(instr, (unsigned char*)scanData, (sizeof(ViUInt16) * data->sensData->numPix));

   return (err);
}



/*---------------------------------------------------------------------------
   Function:   Set Wavelength Data
   Purpose:    This function stores data for pixel-wavelength
               correlation to the instruments nonvolatile memory.

               The given data pair arrays are used to interpolate the
               pixel-wavelength correlation array returned by the
               LC100_getWavelengthData function.

   Note: This function is for future use.

   Note: In case the interpolated pixel-wavelength correlation
   contains negative values, or the values are not strictly
   increasing the function returns with error VI_ERROR_INV_USER_DATA.

   Parameters:

   ViSession instr:                 The actual session to opened device.
   ViInt16 dataSet:                 The desired data set (see #defines in header file).
   ViInt32 pixelDataArray[]:        The pixel data.
   ViReal64 wavelengthDataArray[]:  The wavelength data.
   ViInt32 bufferLength:            The length of both arrays.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_setWavelengthData (ViSession instr, ViInt16 dataSet, ViInt32 pixelDataArray[],
                                           ViReal64 wavelengthDataArray[], ViInt32 bufferLength)
{
   ViStatus          err    = VI_SUCCESS;
   int               target = TARGET_FACT;
   int               to_nvmem = 0;
   LC100_data_t      *data;
   LC100_wl_cal_t    cal;
   
   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   if((!pixelDataArray) || (!wavelengthDataArray)) return VI_ERROR_INV_PARAMETER;

   // check valid buffer length and determine target
   if((bufferLength < LC100_MIN_NUM_USR_ADJ) || (bufferLength > LC100_MAX_NUM_USR_ADJ))   return VI_ERROR_INV_PARAMETER;

   switch (dataSet)
   {
      case LC100_CAL_DATA_SET_FACTORY:
      case LC100_CAL_DATA_SET_FACTORY_NVMEM: target = TARGET_FACT;         break;
      case LC100_CAL_DATA_SET_USER:
      case LC100_CAL_DATA_SET_USER_NVMEM:    target = TARGET_USER;         break;
      default:                               err = VI_ERROR_INV_PARAMETER; break;
   }

   switch (dataSet)
   {
      case LC100_CAL_DATA_SET_FACTORY_NVMEM:
      case LC100_CAL_DATA_SET_USER_NVMEM:    to_nvmem = 1;  break;
      case LC100_CAL_DATA_SET_FACTORY:
      case LC100_CAL_DATA_SET_USER:          to_nvmem = 0;  break;
      default:                               err = VI_ERROR_INV_PARAMETER; break;
   }

   
   // check nodes array
   if((err = LC100_checkNodes(pixelDataArray, wavelengthDataArray, bufferLength)) != VI_SUCCESS)  return err;
   // calculate new coefficients...
   if((err = LC100_nodes2poly(pixelDataArray, wavelengthDataArray, bufferLength, cal.poly)) != VI_SUCCESS) return err;
   // use the coefficients to calculate the new wavelength array...
   if((err = LC100_poly2wlArray(&cal)) != VI_SUCCESS)   return err;


   // wehn directed write new data to NVMEM and data structure
   if(to_nvmem)
   {
      // write polynomical coefficients to eeprom
      if(!err) err = LC100_writePolyCoeff(instr, target, &cal);
      // write user  calibration points
      if(!err) err = LC100_writeCalPoints(instr, target, pixelDataArray, wavelengthDataArray, bufferLength);
      // write valid flags to NVMEM
      if(!err)
      {
         cal.valid = 1;
         err = LC100_writeWLCalValid (instr, target, cal.valid);
      }
   }
   // store the new data in the LC100_data_t structure
   if(!err) err = LC100_applyPixelWavelengthPoints (data, target, pixelDataArray, wavelengthDataArray, bufferLength);
   if(!err) err = LC100_copyPolyData (data, target, &cal);
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Get Wavelength Data
   Purpose:    This function returns data for the pixel-wavelength correlation.
               The maximum and the minimum wavelength are additionally provided
               in two separate return values.

   Note: This function is for future use.

   Note:
   If you do not need either of these values you may pass NULL.

   The value returned in Wavelength_Data_Array[0] is the wavelength
   at pixel 0, this is also the minimum wavelength, the value
   returned in Wavelength_Data_Array[1] is the wavelength at pixel
   1 and so on until Wavelength_Data_Array[LC100_NUM_PIXELS-1]
   which provides the wavelength at pixel LC100_NUM_PIXELS-1
   (2047). This is the maximum wavelength.

   Parameters:

   ViSession instr:                 The actual session to opened device.
   ViInt16 dataSet:                 The desired data set (see #defines at function LC100_setWavelengthData()).
   ViReal64 wavelengthArray[]:      The wavelength data.
   ViPReal64 minimumWavelength:     The minimum wavelength.
   ViPReal64 maximumWavelength:     The maximum wavelength.
   ---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getWavelengthData (ViSession instr, ViInt16 dataSet, ViReal64 wavelengthArray[],
                                           ViPReal64 minimumWavelength, ViPReal64 maximumWavelength)
{
   ViStatus       err    = VI_SUCCESS;
   LC100_data_t   *data;
   
   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   switch (dataSet)
   {
      case LC100_CAL_DATA_SET_FACTORY:
         // copy wavelength array form factory data
         if(wavelengthArray != NULL)   memcpy(wavelengthArray, data->factory_cal.wl, (LC100_NUM_PIXELS * sizeof(ViReal64)));
         if(minimumWavelength != NULL) *minimumWavelength = data->factory_cal.min;
         if(maximumWavelength != NULL) *maximumWavelength = data->factory_cal.max;
         break;

      case LC100_CAL_DATA_SET_USER:
         // copy wavelength arry from user data
         if(!data->user_cal.valid) return VI_ERROR_LC100_NO_USER_DATA;
         if(wavelengthArray != NULL)   memcpy(wavelengthArray, data->user_cal.wl, (LC100_NUM_PIXELS * sizeof(ViReal64)));
         if(minimumWavelength != NULL) *minimumWavelength = data->user_cal.min;
         if(maximumWavelength != NULL) *maximumWavelength = data->user_cal.max;
         break;

      case LC100_CAL_DATA_SET_FACTORY_NVMEM:
         // re-read factory adjustment coefficients from EEPROM
         err = LC100_readPolyCoeff(instr, TARGET_FACT, &(data->factory_cal));
   
         if(!err) LC100_poly2wlArray(&(data->factory_cal));
         // copy wavelength array form factory data
         if(wavelengthArray != NULL)   memcpy(wavelengthArray, data->factory_cal.wl, (LC100_NUM_PIXELS * sizeof(ViReal64)));
         if(minimumWavelength != NULL) *minimumWavelength = data->factory_cal.min;
         if(maximumWavelength != NULL) *maximumWavelength = data->factory_cal.max;
         break;

      case LC100_CAL_DATA_SET_USER_NVMEM:
         // re-read user adjustment coefficients from EEPROM
         err = LC100_readPolyCoeff(instr, TARGET_USER, &(data->user_cal));
   
         if(!err) LC100_poly2wlArray(&(data->user_cal));
         // copy wavelength array form factory data
         if(wavelengthArray != NULL)   memcpy(wavelengthArray, data->user_cal.wl, (LC100_NUM_PIXELS * sizeof(ViReal64)));
         if(minimumWavelength != NULL) *minimumWavelength = data->user_cal.min;
         if(maximumWavelength != NULL) *maximumWavelength = data->user_cal.max;
         break;
         
      default:
         err = VI_ERROR_INV_PARAMETER;
         break;
   }

   return err;
}

/*---------------------------------------------------------------------------
   Function:   Get Calibration Points
   Purpose:    This function returns the pixel-wavelength
               correlation points. These given data pair arrays are
               used by the driver to interpolate the pixel-wavelength
               correlation array returned by the LC100_getWavelengthData
               function.

   Note: This function is for future use.

   Note:
   If you do not need either of these values you may pass NULL.
   The function returns with error VI_ERROR_NO_USER_DATA if no user
   calibration data is present in the instruments nonvolatile
   memory.

   Parameters:

   ViSession instrHandle:           The actual session to opened device.
   ViInt16 dataSet:                 The desired data set (see #defines at function LC100_setWavelengthData()).
   ViInt32 pixelDataArray[]:        The pixel data array.
   ViReal64 wavelengthDataArray[]:  The wavelength data array.
   ViPInt32 bufferLength:           The number of user calibration points.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getCalibrationPoints (ViSession instr,  ViInt16 dataSet, ViInt32 pixelDataArray[],
                                              ViReal64 wavelengthDataArray[], ViPInt32 bufferLength)
{
   ViStatus       err = VI_SUCCESS;
   LC100_data_t   *data;

   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   switch(dataSet)
   {
      case LC100_CAL_DATA_SET_FACTORY:
         if(data->factory_cal.valid)
         {
            if(pixelDataArray)      memcpy(pixelDataArray,      data->factory_points.cal_node_pixel, data->factory_points.cal_node_cnt * sizeof(ViUInt32));
            if(wavelengthDataArray) memcpy(wavelengthDataArray, data->factory_points.cal_node_wl,    data->factory_points.cal_node_cnt * sizeof(ViReal64));
            if(bufferLength)        *bufferLength = data->factory_points.cal_node_cnt;
         }
         else
         {
            err = VI_ERROR_LC100_NO_USER_DATA;
         }
         break;

      case LC100_CAL_DATA_SET_USER:
         if(data->user_cal.valid)
         {
            if(pixelDataArray)      memcpy(pixelDataArray,      data->user_points.cal_node_pixel, data->user_points.cal_node_cnt * sizeof(ViUInt32));
            if(wavelengthDataArray) memcpy(wavelengthDataArray, data->user_points.cal_node_wl,    data->user_points.cal_node_cnt * sizeof(ViReal64));
            if(bufferLength)        *bufferLength = data->user_points.cal_node_cnt;
         }
         else
         {
            err = VI_ERROR_LC100_NO_USER_DATA;
         }
         break;

      case LC100_CAL_DATA_SET_FACTORY_NVMEM:
         // re-read factory adjustment pixel-wavelength pairs (nodes) from EEPROM
         if(!err) err = LC100_readCalPoints (instr, TARGET_FACT, pixelDataArray, wavelengthDataArray, bufferLength);
         break;

      case LC100_CAL_DATA_SET_USER_NVMEM:
         // re-read user adjustment pixel-wavelength pairs (nodes) from EEPROM
         if(!err) err = LC100_readCalPoints (instr, TARGET_USER, pixelDataArray, wavelengthDataArray, bufferLength);
         // just read do not store in data structure (do not 'apply')
         // if(!err) err = LC100_applyPixelWavelengthPoints (data, TARGET_USER, pixelDataArray, wavelengthDataArray, bufferLength);

         break;
         
      default:
         err = VI_ERROR_INV_PARAMETER;
         break;
   }

   return err;
}


/*---------------------------------------------------------------------------
   Function:   Set Amplitude Data
   Purpose:    This function stores data for user-defined amplitude
               correction factors to nonvolatile memory of LC100 device.

               The given data array will be used to correct the
               amplitude information returned by the
               LC100_getScanData function.

   Note: In case the correction factors are out of the range
   LC100_AMP_CORR_FACT_MIN (0.001) ... LC100_AMP_CORR_FACT_MAX (15.9997560)
   the function returns with error VI_ERROR_LC100_INVALID_USER_DATA. 

   Parameters:
   
   ViSession instr:                 The actual session to opened device.
   ViReal64 _VI_FAR AmpCorrFact[]:  The array with amplitude correction factors
   ViInt32 bufferLength:            The length of the array.
   ViInt32 bufferStart:             The start index for the array.
   ViInt32 mode                     With mode one can select if the new data will be applied to current measurements
                                    only (ACOR_APPLY_TO_MEAS) or additionally goes to non volatile memory of the
                                    device too (ACOR_APPLY_TO_MEAS_NVMEM).
                                    If mode is not one of the two predefined macros the function returns VI_ERROR_INV_PARAMETER
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_setAmplitudeData (ViSession instr, ViReal64 AmpCorrFact[], ViInt32 bufferStart, ViInt32 bufferLength, ViInt32 mode)
{
   ViStatus          err      = VI_SUCCESS;
   LC100_data_t      *data;
   int               target = TARGET_FACT;
   int               i;
   ViUInt16          acor[LC100_NUM_PIXELS];
   

   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // check valid buffer start and determine target
   if((bufferStart >= 0) && (bufferStart < LC100_NUM_PIXELS))
   {
      target = TARGET_USER; // target is user adjustment data
   }
   else if ((bufferStart >= FACTORY_SET_START) && (bufferStart < (FACTORY_SET_START + LC100_NUM_PIXELS)))
   {
      bufferStart -= FACTORY_SET_START;
      target = TARGET_FACT; // target is factory adjustment data
   }
   else
   {
      return VI_ERROR_INV_PARAMETER;
   }

   // check valid buffer length
   if((bufferStart + bufferLength) < 1)                return VI_ERROR_INV_PARAMETER;
   if((bufferStart + bufferLength) > LC100_NUM_PIXELS) return VI_ERROR_INV_PARAMETER;

   // check for data out of range
   for(i=0; i<bufferLength; i++)
   {
      if(AmpCorrFact[i] < LC100_AMP_CORR_FACT_MIN)  return VI_ERROR_LC100_INVALID_USER_DATA;
      if(AmpCorrFact[i] > LC100_AMP_CORR_FACT_MAX)  return VI_ERROR_LC100_INVALID_USER_DATA;
   }

   // check for correct mode
   switch(mode)
   {
      case ACOR_APPLY_TO_MEAS:
      case ACOR_APPLY_TO_MEAS_NVMEM:
      case ACOR_APPLY_TO_NVMEM_ONLY:
            break;
      default:
            return VI_ERROR_INV_PARAMETER;
   }
      
   // copy new values to intermediate buffer and ...
   for(i=0; i<bufferLength; i++)
   {
      acor[bufferStart + i] = (ViUInt16)(AmpCorrFact[i] * (double)0x1000);          // 0x1000 is 1.0 in the FPGA
   }

   err = endian_convert16(acor, bufferLength);
   
   if(target == TARGET_USER)     ////  target is user adjustment data  ////
   {
      // write data to LC100 scratch RAM first ..
      if(!err) err = nv_write(instr, (bufferStart  * sizeof(uint16_t)), (const void *)acor, (bufferLength * sizeof(uint16_t)), LC100_NVMEM_LUT_SCRATCH);

      switch(mode)
      {
         case ACOR_APPLY_TO_MEAS_NVMEM:
               // .. then make LC100 to copy the received data from scratch RAM to user NVMEM and ..
               if(!err) err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, NVMEM_SCRATCH_TO_NV_USER, LC100_NVMEM_STORE, 0, NULL);
               // .. then make LC100 to copy the received data from scratch RAM to LUT user and ..
               if(!err) err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, NVMEM_SCRATCH_TO_USER, LC100_NVMEM_STORE, 0, NULL);
               // .. then make LC100 to apply the data to the CCD machine.
               if(!err) err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, NVMEM_APPLY_LUT, LC100_NVMEM_STORE, 0, NULL);
               break;
               
         case ACOR_APPLY_TO_MEAS:
               // .. then make LC100 to copy the received data from scratch RAM to LUT user and ..
               if(!err) err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, NVMEM_SCRATCH_TO_USER, LC100_NVMEM_STORE, 0, NULL);
               // .. then make LC100 to apply the data to the CCD machine.
               if(!err) err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, NVMEM_APPLY_LUT, LC100_NVMEM_STORE, 0, NULL);
               break;

         case ACOR_APPLY_TO_NVMEM_ONLY:
               // .. then make LC100 to copy the received data from scratch RAM to user NVMEM.
               if(!err) err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, NVMEM_SCRATCH_TO_NV_USER, LC100_NVMEM_STORE, 0, NULL);
               break;
               
         default:
               return VI_ERROR_INV_PARAMETER;
      }
   }
   else                             ////  target is factory adjustment data   ////
   {
      // write data to LC100 scratch RAM first ..
      if(!err) err = nv_write(instr, (bufferStart  * sizeof(uint16_t)), (const void *)acor, (bufferLength * sizeof(uint16_t)), LC100_NVMEM_LUT_SCRATCH);

      switch(mode)
      {
         case ACOR_APPLY_TO_MEAS_NVMEM:
               // .. then make LC100 to copy the received data from scratch RAM to factory NVMEM and ..
               if(!err) err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, NVMEM_SCRATCH_TO_NV_FACT, LC100_NVMEM_STORE, 0, NULL);
               // .. then make LC100 to copy the received data from scratch RAM to LUT Factory and ..
               if(!err) err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, NVMEM_SCRATCH_TO_FACT, LC100_NVMEM_STORE, 0, NULL);
               // .. then make LC100 to apply the data to the CCD machine.
               if(!err) err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, NVMEM_APPLY_LUT, LC100_NVMEM_STORE, 0, NULL);
               break;

         case ACOR_APPLY_TO_MEAS:
               // .. then make LC100 to copy the received data from scratch RAM to LUT Factory and ..
               if(!err) err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, NVMEM_SCRATCH_TO_FACT, LC100_NVMEM_STORE, 0, NULL);
               // .. then make LC100 to apply the data to the CCD machine.
               if(!err) err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, NVMEM_APPLY_LUT, LC100_NVMEM_STORE, 0, NULL);
               break;

         case ACOR_APPLY_TO_NVMEM_ONLY:
               // .. then make LC100 to copy the received data from scratch RAM to factory NVMEM.
               if(!err) err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, NVMEM_SCRATCH_TO_NV_FACT, LC100_NVMEM_STORE, 0, NULL);
               break;
               
         default:
               return VI_ERROR_INV_PARAMETER;
      }
   }
   
   return err;
}




/*---------------------------------------------------------------------------
   Function:   Get Amplitude Data
   Purpose:    This function retrieves data for user-defined amplitude
               correction factors from nonvolatile memory of the LC100 device.

               The given data array can be used to correct the
               amplitude information returned by the
               LC100_getScanData function.

   Parameters:
   
   ViSession instr:                 The actual session to opened device.
   ViReal64 AmpCorrFact[]:          The array with amplitude correction factors
   ViInt32 bufferLength:            The length of the array.
   ViInt32 bufferStart:             The start index for the array.
   ViInt32 mode                     With mode one can select if the data will be a copy of the currently used
                                    amplitude correction factors (ACOR_FROM_CURRENT) or if the data is read out from the
                                    device non volatile memory (ACOR_FROM_NVMEM)
                                    If mode is not one of the two predefined macros the function returns VI_ERROR_INV_PARAMETER
---------------------------------------------------------------------------*/

ViStatus _VI_FUNC LC100_getAmplitudeData (ViSession instr, ViReal64 AmpCorrFact[], ViInt32 bufferStart, ViInt32 bufferLength, ViInt32 mode)
{
   ViStatus          err      = VI_SUCCESS;
   LC100_data_t      *data;
   int               target = TARGET_FACT;
   int               i;
   uint16_t          nv_mode;
   ViUInt16          acor[LC100_NUM_PIXELS];
   
   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // check valid buffer start and determine target
   if((bufferStart >= 0) && (bufferStart < LC100_NUM_PIXELS))
   {
      target = TARGET_USER;
   }
   else if ((bufferStart >= FACTORY_SET_START) && (bufferStart < (FACTORY_SET_START + LC100_NUM_PIXELS)))
   {
      bufferStart -= FACTORY_SET_START;
      target = TARGET_FACT;
   }
   else
   {
      return VI_ERROR_INV_PARAMETER;
   }

   // check valid buffer length
   if((bufferStart + bufferLength) < 1)                  return VI_ERROR_INV_PARAMETER;
   if((bufferStart + bufferLength) > LC100_NUM_PIXELS)   return VI_ERROR_INV_PARAMETER;

   // check for correct mode
   switch(mode)
   {
      case ACOR_FROM_CURRENT:
            if(target == TARGET_USER)  nv_mode = LC100_NVMEM_LUT_USER;
            else                       nv_mode = LC100_NVMEM_LUT_FACTORY;
            break;

      case ACOR_FROM_NVMEM:
            if(target == TARGET_USER)
            {
               // first make LC100 to copy the data from its NVMEM to an LC100 internal scratch RAM
               err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, NVMEM_NV_USER_TO_SCRATCH, LC100_NVMEM_STORE, 0, NULL);
            }
            else
            {
               // first make LC100 to copy the data from its NVMEM to an LC100 internal scratch RAM
               err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, NVMEM_NV_USER_TO_SCRATCH, LC100_NVMEM_STORE, 0, NULL);
            }
            // then read from scratch memory
            nv_mode = LC100_NVMEM_LUT_SCRATCH;
            break;
            
      case ACOR_FROM_APPLIED:
            // copy applied LUT to scratch RAM
            err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, NVMEM_APPLIED_TO_SCRATCH, LC100_NVMEM_STORE, 0, NULL);
            // then read from scratch memory
            nv_mode = LC100_NVMEM_LUT_SCRATCH;
            break;

      default:
            return VI_ERROR_INV_PARAMETER;
   }

    
   if(!err) err = nv_read(instr, (bufferStart * sizeof(uint16_t)), (void *)acor, (size_t)(bufferLength * sizeof(uint16_t)), nv_mode);
   if(!err) err = endian_convert16(acor, bufferLength);

   if(!err)
   {
      // copy new values
      for(i=0; i<bufferLength; i++)
      {
         AmpCorrFact[i] = (ViReal64)acor[i + bufferStart] / (ViReal64)0x1000;
      }
   }

   return err;
}



/*===========================================================================

 Class: Utility Functions.

===========================================================================*/
/*---------------------------------------------------------------------------
   Function:   Identification Query
   Purpose:    This function returns the device identification information.

   Parameters:

   ViSession instrHandle:           The actual session to opened device.
   ViChar manufacturerName[]:       The manufacturers name.
   ViChar deviceName[]:             The device name.
   ViChar serialNumber[]:           The serial number.
   ViChar firmwareRevision[]:       The firmware revision.
---------------------------------------------------------------------------*/
#define RCMD_GET_USB_FIRMWARE    0
#define RCMD_GET_FPGA_VER        2

ViStatus _VI_FUNC LC100_identificationQuery (ViSession instr, ViChar manuf[], ViChar device[], ViChar serial[], ViChar fwRev[])
{
   ViStatus err;
   ViUInt16 retCnt = 0;
   ViUInt16 verUsbFw = 0;
   ViUInt16 verFpgaFw = 0;

   if(manuf)
   {
      if((err = viGetAttribute (instr, VI_ATTR_MANF_NAME, manuf))) return (err);
   }
   if(device)
   {
      if((err = viGetAttribute (instr, VI_ATTR_MODEL_NAME, device))) return (err);
   }
   if(serial)
   {
      if((err = viGetAttribute (instr, VI_ATTR_USB_SERIAL_NUM, serial))) return (err);
   }
   if(fwRev)
   {
      if((err = LC100_usbIn(instr, LC100_RCMD_GET_VERSION, 0, RCMD_GET_USB_FIRMWARE, 2, (ViBuf)&verUsbFw, &retCnt)) < 0) return (err);
      if(retCnt != 2) return VI_ERROR_INV_RESPONSE;

      if((err = LC100_usbIn(instr, LC100_RCMD_GET_VERSION, 0, RCMD_GET_FPGA_VER, 2, (ViBuf)&verFpgaFw, &retCnt)) < 0) return (err);
      if(retCnt != 2) return VI_ERROR_INV_RESPONSE;

      sprintf(fwRev, "%d.%d.%d/%d.%d.%d", ((verUsbFw & 0xFF00) >> 8), ((verUsbFw & 0x00F0) >> 4), (verUsbFw & 0x000F) , ((verFpgaFw & 0xFF00) >> 8), ((verFpgaFw & 0x00F0) >> 4), (verFpgaFw & 0x000F));
   }
   return (VI_SUCCESS);
}


/*---------------------------------------------------------------------------
   Function:   Read Sensor Information
   Purpose:    This function returns sensor specific data.

   Parameters:

   ViSession instrHandle:           The actual session to opened device.
   ViPInt16 numberOfPixels:         The number of pixels.
   ViPReal64 minIntegrationTime:    The minimum integration time in seconds.
   ViPReal64 maxIntegrationTime:    The maximum integration time in seconds.
   ViChar description[]:            A description string.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_sensorTypeQuery (ViSession instr, ViInt16 *numberOfPixels, ViReal64 *minIntegrationTime, ViReal64 *maxIntegrationTime, ViChar description[])
{
   ViStatus       err;
   LC100_data_t   *data;

   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   if(numberOfPixels)      *numberOfPixels = data->sensData->numPix;
   if(minIntegrationTime)  *minIntegrationTime = data->sensData->minIntTim * 1.0E-6;
   if(maxIntegrationTime)  *maxIntegrationTime = data->sensData->maxIntTim * 1.0E-6;
   if(description)         strcpy(description, data->sensData->descr);
   return (VI_SUCCESS);
}

/*---------------------------------------------------------------------------
   Function:   Revision Query
   Purpose:    This function returns the revision numbers of the instrument
               driver and the device firmware.

   Parameters:

   ViSession instrHandle:              The actual session to opened device.
   ViChar instrumentDriverRevision[]:  The instrument driver revision.
   ViChar firmwareRevision[]:          The firmware revision.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_revisionQuery (ViSession instr, ViChar driverRev[], ViChar fwRev[])
{
   ViStatus err;

   if(driverRev)
   {
      strcpy(driverRev, DRIVER_REVISION_TXT);
   }
   if(fwRev)
   {
      if(instr)
      {
         if((err = LC100_identificationQuery(instr, VI_NULL, VI_NULL, VI_NULL, fwRev))) return err;
      }
      else
      {
         *fwRev = '\0';
      }
   }
   return VI_SUCCESS;
}


/*---------------------------------------------------------------------------
   Function:   Reset
   Purpose:    This function resets the device.

   Parameters:

   ViSession instr:  The actual session to opened device.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_reset (ViSession instrumentHandle)
{
   ViStatus err = VI_SUCCESS;

   err = LC100_usbOut(instrumentHandle, LC100_WCMD_RESET, 0, 0, 0, NULL);
   return err;
}



/*---------------------------------------------------------------------------
   Function:   Error Message
   Purpose:    This function translates the error return value from the
               instrument driver into a user-readable string.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViStatus statusCode:       The error/status code for which a description is looked for
   ViChar description[]:      The buffer to which the error message is written to
                              size must be at least LC100_ERR_DESCR_BUFFER_SIZE.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_errorMessage(ViSession instr, ViStatus statusCode, ViChar description[])
{
   const errDescrStat_t *ptr;

   // VISA errors
   if(viStatusDesc(instr, statusCode, description) != VI_WARN_UNKNOWN_STATUS) return VI_SUCCESS;

   // static driver errors
   ptr = LC100_errDescrStat;
   while(ptr->descr != VI_NULL)
   {
      if(ptr->err == statusCode)
      {
         strcpy(description, ptr->descr);
         return (VI_SUCCESS);
      }
      ptr ++;
   }

   // not found
   viStatusDesc(instr, VI_WARN_UNKNOWN_STATUS, description);
   return VI_WARN_UNKNOWN_STATUS;
}


/*---------------------------------------------------------------------------
   Function:   Set Attribute
   Purpose:    This function sets a specified attribute.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViAttr attribute:          The attribute to set. See "Driver attributes"
                              section for available attributes and further
                              information.
   ViUInt32 value:            The attribute value.
---------------------------------------------------------------------------*/
//ViStatus _VI_FUNC LC100_setAttribute (ViSession instrumentHandle, ViAttr attribute, ViAttrState value)
//{
//   ViStatus       err;
//   LC100_data_t   *data;
//
//   if((err = viGetAttribute(instrumentHandle, VI_ATTR_USER_DATA, &data))) return (err);
//   switch(attribute)
//   {
//      case LC100_ATTR_USER_DATA:
//         data->userData = value;
//         break;
//
//      default:
//         return (VI_ERROR_PARAMETER2);
//   }
//   return (VI_SUCCESS);
//}

/*---------------------------------------------------------------------------
   Function:   Get Attribute
   Purpose:    This function returns a specified attribute.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViAttr attribute:          The attribute to get. See "Driver attributes"
                              section for available attributes and further
                              information.
   ViUInt32 value:            The attribute value.
---------------------------------------------------------------------------*/
//ViStatus _VI_FUNC LC100_getAttribute (ViSession instrumentHandle, ViAttr attribute, ViAttrState *value)
//{
//   ViStatus       err;
//   LC100_data_t   *data;
//
//   if((err = viGetAttribute(instrumentHandle, VI_ATTR_USER_DATA, &data))) return (err);
//   switch(attribute)
//   {
//      case LC100_ATTR_USER_DATA:
//         *value = data->userData;
//         break;
//
//      default:
//         return (VI_ERROR_PARAMETER2);
//   }
//   return (VI_SUCCESS);
//}


/*---------------------------------------------------------------------------
   Function:   Set User Text
   Purpose:    This function writes the given string to the novolatile memory of
               the LC100.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViChar userText:           The new user text. The string will be truncated
                              to 64 characters maximum.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_setUserText (ViSession instr, ViChar userText[])
{
   ViStatus       err = VI_SUCCESS;
   LC100_data_t   *data;
   char           buf[LC100_MAX_USER_NAME_SIZE+1];

   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data))) return (err);

   if(!userText)  return VI_ERROR_INV_PARAMETER;

   // copy the string
   strncpy(buf, userText, LC100_MAX_USER_NAME_SIZE);   // strncpy will fill with '\0' when userText is smaller than (USER_LABEL_LENGTH-1)


   // write to eeprom
   err = LC100_usbOut(instr, LC100_WCMD_USER_TEXT, 0, 0, LC100_MAX_USER_NAME_SIZE, buf);

   return err;
}


/*---------------------------------------------------------------------------
   Function:   Get User Text
   Purpose:    This function reads the user text from the novolatile memory of
               the LC100.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViChar userText:           The user text. The buffer must contain at least
                              65 elements. 64 characters plus a trailing zero ('\0').
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getUserText (ViSession instr, ViChar userText[])
{
   ViStatus       err = VI_SUCCESS;
   ViUInt16       bytes_read;
   LC100_data_t   *data;

   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;

   if(!userText)  return VI_ERROR_INV_PARAMETER;

   err = LC100_usbIn(instr, LC100_RCMD_USER_TEXT, 0, 0, LC100_MAX_USER_NAME_SIZE, userText, &bytes_read);
   if((bytes_read != LC100_MAX_USER_NAME_SIZE) && (!err)) err = VI_ERROR_INV_RESPONSE;

   if(err)
   {
      memset(userText, 0, LC100_MAX_USER_NAME_SIZE);

      // in case there is no user text...
      strncpy(userText, DEFAULT_USER_TEXT, LC100_MAX_USER_NAME_SIZE);
   }
   else
   {
      userText[LC100_MAX_USER_NAME_SIZE] = '\0';   // terminate string
   }

   return err;
}


/*===========================================================================

 SubClass: Raw I/O
 NOTE: Do not use this functions.

===========================================================================*/
/*---------------------------------------------------------------------------
  USB Out - encapsulates the VISA function 'viUsbControlOut()'. When LC100
  stalls the error VI_ERROR_IO will be returned by 'viUsbControlOut()'.
  Then USB Out will issue the VISA function 'viUsbControlIn' and tries to
  read one Byte from the LC100, if this succeeds the obtained Byte contains
  the error code from the LC100. This means we have NO communications error.

  Parameters
  ViSession Instrument_Handle :  the handle obtained by 'LC100_init()'
  ViInt16 bRequest            :  the command sent to the LC100 device
  ViUInt16 wValue             :  arbitrary parameter, can be used for additional information
  ViUInt16 wIndex             :  arbitrary parameter, can be used for additional information
  ViUInt16 wLength            :  size of Buffer
  ViBuf Buffer                :  buffer of max. 64 Bytes

  Result                      :  Error
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_usbOut(ViSession Instrument_Handle, ViInt16 bRequest, ViUInt16 wValue, ViUInt16 wIndex, ViUInt16 wLength, ViBuf Buffer)
{
   ViStatus       err;
   unsigned char  LC100_Error;

   err = viUsbControlOut (Instrument_Handle, 0x40, bRequest, wValue, wIndex, wLength, Buffer);

   if(err == VI_ERROR_IO)
   {
      err = viUsbControlIn (Instrument_Handle, 0xC0, LC100_RCMD_GET_ERROR, 0, 0, 1, &LC100_Error, NULL);
      if(!err)
      {
         // this is the 'self-created' LC100 error
         err = VI_ERROR_USBCOMM_OFFSET + (ViStatus)LC100_Error;
         // the range is from VI_ERROR_USBCOMM_OFFSET to VI_ERROR_USBCOMM_OFFSET + 0xFF
         // = 0xBFFC0B00 ... 0xBFFC0BFF
      }
   }

   return err;
}

/*---------------------------------------------------------------------------
  USB In - encapsulates the VISA function 'viUsbControlIn()'. When LC100 stalls
  the error VI_ERROR_IO will be returned by 'viUsbControlIn()'. Then USB Out
  will issue the VISA function 'viUsbControlIn' and tries to read one Byte
  from the LC100, if this succeeds the obtained Byte contains the error code
  from the LC100. This means we have NO communications error.

  Parameters
  ViSession Instrument_Handle :  the handle obtained by 'LC100_init()'
  ViInt16 bRequest            :  the command sent to the LC100 device
  ViUInt16 wValue             :  arbitrary parameter, can be used for additional information
  ViUInt16 wIndex             :  arbitrary parameter, can be used for additional information
  ViUInt16 wLength            :  size of Buffer
  ViBuf Buffer                :  buffer of max. 64 Bytes
  ViPUInt16 Read_Bytes        :  number of bytes actually read out

  Result                      :  Error
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_usbIn(ViSession Instrument_Handle, ViInt16 bRequest, ViUInt16 wValue, ViUInt16 wIndex, ViUInt16 wLength, ViBuf Buffer, ViPUInt16 Read_Bytes)
{
   ViStatus err;
   unsigned char LC100_Error;
   ViUInt16 return_count;

   err = viUsbControlIn (Instrument_Handle, 0xC0, bRequest, wValue, wIndex, wLength, Buffer, &return_count);
   if(Read_Bytes != NULL)  *Read_Bytes = return_count;
   if(err == VI_ERROR_IO)
   {
      err = viUsbControlIn (Instrument_Handle, 0xC0, LC100_RCMD_GET_ERROR, 0, 0, 1, &LC100_Error, NULL);
      if(!err)
      {
         // this is the 'self-created' LC100 error
         err = VI_ERROR_USBCOMM_OFFSET + (ViStatus)LC100_Error;
         // the range is from VI_ERROR_USBCOMM_OFFSET to VI_ERROR_USBCOMM_OFFSET + 0xFF
         // = 0xBFFC0B00 ... 0xBFFC0BFF
      }
   }

   return err;
}

/*---------------------------------------------------------------------------
  USB Write - encapsulates the VISA function 'viWrite'.

  Parameters
  ViSession Instrument_Handle :  the handle obtained by 'LC100_init()'
  ViBuf Buffer                :  buffer to send to device
  ViUInt32 Count              :  number of Bytes to send from buffer to device

  Return Value
  ViUInt32 *ReturnCount       :  number of Bytes actually sent to device
                                 You may pass NULL if you do not need the value

  Result                      :  Error
---------------------------------------------------------------------------*/
/*
ViStatus _VI_FUNC LC100_USB_write(ViSession Instrument_Handle, ViBuf Buffer, ViUInt32 Count, ViUInt32 *ReturnCount)
{
   ViStatus err;
   ViUInt32 retcount;

   err = viWrite (Instrument_Handle, Buffer, Count, &retcount);

   if((err == VI_SUCCESS) && (ReturnCount != NULL))
      *ReturnCount = retcount;

   return err;
}
*/

/*---------------------------------------------------------------------------
  USB Read - encapsulates the VISA function 'viRead'.

  Parameters
  ViSession Instrument_Handle :  the handle obtained by 'LC100_init()'
  unsigned char *ReceiveData  :  pointer to a buffer where received data is put to
  ViUInt32 Count              :  number of Bytes to read from device to buffer

  Return Value

  Result                      :  Error
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_USB_read(ViSession Instrument_Handle, unsigned char *ReceiveData, ViUInt32 Count)
{
   ViStatus err;
   ViUInt32 retcount;
   ViUInt32 index = 0;

   do
   {
      err = viRead (Instrument_Handle, &ReceiveData[index], Count, &retcount);
      if(!err)
      {
         index += retcount;
         Count -= retcount;
      }
   }
   while(!err && Count > 0);

   return err;
}


/*===========================================================================

 SubClass: Test and Maintenance

===========================================================================*/

/*===========================================================================

 Auxiliary Functions

===========================================================================*/
/*---------------------------------------------------------------------------
 Cleanup
---------------------------------------------------------------------------*/
static ViStatus initClose (ViSession instr, ViStatus stat)
{
   ViStatus       err   = VI_SUCCESS;
   ViSession      rm    = VI_NULL;
   LC100_data_t   *data = VI_NULL;

   // Get resource manager session and private data pointer
   viGetAttribute(instr, VI_ATTR_RM_SESSION, &rm);
   viGetAttribute(instr, VI_ATTR_USER_DATA,  &data);

   // Free private data
   if(data != NULL) free(data);

   // Close sessions
   if(instr) err = viClose(instr);
   if(rm) viClose(rm);

   return ((stat != VI_SUCCESS) ? stat : err);
}



/*===========================================================================
 NV memory record reading/writing
===========================================================================*/
static ViStatus nv_read_version(LC100_data_t *data)
{
   ViStatus err;
   uint16_t nVVer;
   
   err = nv_read(data->instr, 0, (void *)&nVVer, sizeof(uint16_t), LC100_NVMEM_VERSION);

   if(!err) data->nvVersion = nVVer;
   // we have to connect even on a not working FPGA, maybe we havt to load an FPGA stream later
   // so do not prevent driver from connecting here when the error is VI_ERROR_LC100_FPGA_COMM_ERR
   if(err == VI_ERROR_LC100_FPGA_COMM_ERR)
   {
      data->nvVersion = 0xFACE;     // value 0xFACE acts here as an indicator for a non configured FPGA
      err = VI_SUCCESS;
   }
   return err;
}

/*===========================================================================
 Non Volatile Memory (NVMEM) access
===========================================================================*/

static ViStatus nv_read(ViSession instr, uint16_t adr, void *dest, size_t len, uint16_t index)
{
   ViStatus err = VI_SUCCESS;
   uint8_t  *ptr = (uint8_t*)dest;
   size_t   size;
   ViUInt16 retCnt;

   while(!err && len)
   {
      size = (len > EZUSB_EP0_TRANSFER_SIZE) ? EZUSB_EP0_TRANSFER_SIZE : len;
      err = LC100_usbIn(instr, LC100_RCMD_NV_MEMORY, adr, index, size, ptr, &retCnt);
      ptr += size;
      adr += size;
      len -= size;
   }
   return (err);
}

static ViStatus nv_write(ViSession instr, uint16_t adr, const void *src, size_t len, uint16_t index)
{
   ViStatus err = VI_SUCCESS;
   uint8_t  *ptr = (uint8_t*)src;
   size_t   size;

   while(!err && len)
   {
      size = (len > EZUSB_EP0_TRANSFER_SIZE) ? EZUSB_EP0_TRANSFER_SIZE : len;
      err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, adr, index, size, ptr);
      ptr += size;
      adr += size;
      len -= size;
   }
   return (err);
}

/*===========================================================================
 Endianness related functions
 (Unfortunately the CPU in the LC100 has another Endian that x86 here)
===========================================================================*/
static ViStatus endian_convert16(uint16_t *data, uint32_t len)
{
   union16_t helper;
   uint8_t   small_helper;
   
   while(len)
   {
      helper.one_word      = *data;
      small_helper         = helper.two_bytes.hb;
      helper.two_bytes.hb  = helper.two_bytes.lb;
      helper.two_bytes.lb  = small_helper;
      *data                = helper.one_word;
      data++;
      len--;
   };
   
   return VI_SUCCESS;
}

/*===========================================================================
 Pixel versus Wavelength related functions
 enables user to store adjustment data and using LC100 as a spectrometer
===========================================================================*/
/*---------------------------------------------------------------------------
   Function:   Get User Wavelength Parameters
   Purpose:    This function reads the parameters necessary to calculate from
               pixels to wavelength and vice versa stored in NVMEM of the
               LC100 and stores the values in the LC100_data_t structure.
---------------------------------------------------------------------------*/
static ViStatus LC100_getUserWavelengthParameters (ViSession instr)
{
   LC100_data_t      *data;
   ViStatus       err = VI_SUCCESS;
   ViInt32        bufferLength;
   ViInt32        pixelDataArray[LC100_MAX_NUM_USR_ADJ];
   ViReal64       wavelengthDataArray[LC100_MAX_NUM_USR_ADJ];
   ViUInt16       flag_valid;
   
   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // set the user calibration valid flag to false
   data->user_cal.valid = 0;

   // read user adjustment coefficients from EEPROM
   err = LC100_readPolyCoeff(instr, TARGET_USER, &(data->user_cal));
   if(!err) LC100_poly2wlArray(&(data->user_cal));

   // read user adjustment nodes from EEPROM and calculate coefficients and wavelength array
   if(!err) err = LC100_readCalPoints (instr, TARGET_USER, pixelDataArray, wavelengthDataArray, &bufferLength);
   if(!err) err = LC100_readWLCalValid(instr, TARGET_USER, &flag_valid);
   if(!err) err = LC100_applyPixelWavelengthPoints (data, TARGET_USER, pixelDataArray, wavelengthDataArray, bufferLength);

   // we do not overwrite the recently read out poly-coeffs
   // if(!err) err = LC100_nodes2poly(data->user_points.cal_node_pixel, data->user_points.cal_node_wl, data->user_points.cal_node_cnt, data->user_cal.poly);
   if(!err) err = LC100_poly2wlArray(&(data->user_cal));
   if(!err && flag_valid) data->user_cal.valid = 1;
                                        
   return VI_SUCCESS;   // errors ignored by intention here, eventually missing Pixel-Wavelength parameters should not stop init(); from working
}


/*---------------------------------------------------------------------------
   Function:   Get Factory Wavelength Parameters
   Purpose:    This function reads the parameters necessary to calculate from
               pixels to wavelength and vice versa stored in NVMEM of the
               LC100 and stores the values in the LC100_data_t structure.
---------------------------------------------------------------------------*/
static ViStatus LC100_getFactoryWavelengthParameters (ViSession instr)
{
   LC100_data_t   *data;
   ViStatus       err = VI_SUCCESS;
   ViInt32        bufferLength;
   ViInt32        pixelDataArray[LC100_MAX_NUM_USR_ADJ];
   ViReal64       wavelengthDataArray[LC100_MAX_NUM_USR_ADJ];
   ViUInt16       flag_valid;
   
   // get private data
   if((err = viGetAttribute(instr, VI_ATTR_USER_DATA, &data)) != VI_SUCCESS) return err;
   
   // set the factory calibration valid flag to false
   data->factory_cal.valid = 0;
   
   // read factory adjustment coefficients from EEPROM
   err = LC100_readPolyCoeff(instr, TARGET_FACT, &(data->factory_cal));
   
   if(!err) LC100_poly2wlArray(&(data->factory_cal));
   // read factory adjustment nodes from NVMEM and calculate coefficients and wavelength array
   if(!err) err = LC100_readCalPoints (instr, TARGET_FACT, pixelDataArray, wavelengthDataArray, &bufferLength);
   if(!err) err = LC100_readWLCalValid(instr, TARGET_FACT, &flag_valid);
   if(!err) err = LC100_applyPixelWavelengthPoints (data, TARGET_FACT, pixelDataArray, wavelengthDataArray, bufferLength);
   
   // we do not overwrite the recently read out poly-coeffs
   // if(!err) err = LC100_nodes2poly(data->factory_points.cal_node_pixel, data->factory_points.cal_node_wl, data->factory_points.cal_node_cnt, data->factory_cal.poly);
   if(!err) err = LC100_poly2wlArray(&(data->factory_cal));
   if(!err && flag_valid) data->factory_cal.valid = 1;
   
   return VI_SUCCESS;   // errors ignored by intention here, eventually missing Pixel-Wavelength parameters should not stop init(); from working
}


/*---------------------------------------------------------------------------
   Function:   Read Polynomial Coefficients
   Purpose:    This function reads the coefficients needed for a pixel-
               wavelength correlation from the LC100's non volatile memory
---------------------------------------------------------------------------*/
static ViStatus LC100_readPolyCoeff   (ViSession instr, int target, LC100_wl_cal_t *cal)
{
   ViStatus       err = VI_SUCCESS;
   ViUInt16       To_Read_Bytes, Read_Bytes;
   
   To_Read_Bytes = LC100_NUM_POLY_POINTS * sizeof(ViReal64);
   // read polynomical coefficients from NVMEM
   switch(target)
   {
      case TARGET_USER  :
            err = LC100_usbIn(instr, LC100_WCMD_NV_MEMORY, 0, LC100_NVMEM_USER_WL_POLY, To_Read_Bytes, (uint8_t *)&(cal->poly), &Read_Bytes);
            break;
            
      case TARGET_FACT  :
            err = LC100_usbIn(instr, LC100_WCMD_NV_MEMORY, 0, LC100_NVMEM_FACT_WL_POLY, To_Read_Bytes, (uint8_t *)&(cal->poly), &Read_Bytes);
            break;
            
      default           :
            err = VI_ERROR_LC100_DRV_INV_USER_DATA;
            break;
   }

   // if(To_Read_Bytes != Read_Bytes)  err = ...      /* we can do this later */
   
   return err;
}

/*---------------------------------------------------------------------------
   Function:   Write Polynomial Coefficients
   Purpose:    This function writes the coefficients needed for a pixel-
               wavelength correlation to the LC100's non volatile memory
---------------------------------------------------------------------------*/
static ViStatus LC100_writePolyCoeff  (ViSession instr, int target, LC100_wl_cal_t *cal)
{
   ViStatus       err = VI_SUCCESS;
   
   // write polynomical coefficients to NVMEM
   switch(target)
   {
      case TARGET_USER  :
            err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, 0, LC100_NVMEM_USER_WL_POLY, LC100_NUM_POLY_POINTS * sizeof(ViReal64), (uint8_t *)&(cal->poly));
            break;
            
      case TARGET_FACT  :
            err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, 0, LC100_NVMEM_FACT_WL_POLY, LC100_NUM_POLY_POINTS * sizeof(ViReal64), (uint8_t *)&(cal->poly));
            break;
            
      default           :
            err = VI_ERROR_LC100_DRV_INV_USER_DATA;
            break;
   }
   
   return err;
}

/*---------------------------------------------------------------------------
   Function:   Read Pixel-Wavelength Calibration Points
   Purpose:    This function reads the pixel-wavelength pairs
               from the LC100's non volatile memory
---------------------------------------------------------------------------*/
static ViStatus LC100_readCalPoints   (ViSession instr, int target, ViInt32 pixelDataArray[],
                                       ViReal64 wavelengthDataArray[], ViInt32 *bufferLength)
{
   ViStatus err = VI_SUCCESS;
   ViInt32  cnt;
   char     buf[sizeof(ViUInt32) + sizeof(ViReal64)];
   ViUInt16 index_pt, index_num;
   ViUInt16 To_Read_Bytes, Read_Bytes, blen;
   
      
   switch(target)
   {
      case TARGET_USER  :  index_pt  = LC100_NVMEM_USER_WL_POINTS;
                           index_num = LC100_NVMEM_USER_WL_NPTS;     break;
      case TARGET_FACT  :  index_pt  = LC100_NVMEM_FACT_WL_POINTS;
                           index_num = LC100_NVMEM_FACT_WL_NPTS;     break;
      default           :  err = VI_ERROR_LC100_DRV_INV_USER_DATA;   break;
   }
   
   // read number of calibration points to NVMEM
   To_Read_Bytes = sizeof(ViUInt16);
   if(!err) err = LC100_usbIn(instr, LC100_WCMD_NV_MEMORY, 0, index_num, To_Read_Bytes, (uint8_t *)&blen, &Read_Bytes);
   // if(To_Read_Bytes != Read_Bytes)  err = ...      /* we can do this later */

   if(!err)
   {
      // when the number of points is out of the bounds here, do not read ot any points
      if(blen > LC100_MAX_NUM_USR_ADJ) err = VI_ERROR_LC100_DRV_INV_USER_DATA;
      if(blen < LC100_MIN_NUM_USR_ADJ) err = VI_ERROR_LC100_DRV_INV_USER_DATA;

      if(!err && bufferLength)   *bufferLength = (ViInt32)blen;
   }  

   // write user  calibration points
   cnt = 0;
   To_Read_Bytes = sizeof(ViUInt32) + sizeof(ViReal64);
   while((!err) && (cnt < blen))
   {
      err = LC100_usbIn(instr, LC100_WCMD_NV_MEMORY, cnt, index_pt, To_Read_Bytes, buf, &Read_Bytes);
      memcpy((void*)&(pixelDataArray     [cnt]), &buf[0],                sizeof(ViUInt32));
      memcpy((void*)&(wavelengthDataArray[cnt]), &buf[sizeof(ViUInt32)], sizeof(ViReal64));
      cnt++;
   }

   
   return err;
}

/*---------------------------------------------------------------------------
   Function:   Write Pixel-Wavelength Calibration Points
   Purpose:    This function writes the pixel-wavelength pairs
               to the LC100's non volatile memory
---------------------------------------------------------------------------*/
 
static ViStatus LC100_writeCalPoints  (ViSession instr, int target, ViInt32 pixelDataArray[],
                                       ViReal64 wavelengthDataArray[], ViInt32 bufferLength)
{
   ViStatus err = VI_SUCCESS;
   ViInt32  cnt;
   char     buf[sizeof(ViUInt32) + sizeof(ViReal64)];
   ViUInt16 index_pt, index_num, blen;
      
   switch(target)
   {
      case TARGET_USER  :  index_pt  = LC100_NVMEM_USER_WL_POINTS;
                           index_num = LC100_NVMEM_USER_WL_NPTS;     break;
      case TARGET_FACT  :  index_pt  = LC100_NVMEM_FACT_WL_POINTS;
                           index_num = LC100_NVMEM_FACT_WL_NPTS;     break;
      default           :  err = VI_ERROR_LC100_DRV_INV_USER_DATA;   break;
   }
   
   if(bufferLength > LC100_MAX_NUM_USR_ADJ)  err = VI_ERROR_LC100_DRV_INV_USER_DATA;
   if(bufferLength < LC100_MIN_NUM_USR_ADJ)  err = VI_ERROR_LC100_DRV_INV_USER_DATA;
   
   blen = (ViUInt16)bufferLength;
   
   // write user calibration points
   cnt = 0;
   while((!err) && (cnt < blen))
   {
      memcpy(&buf[0],                (void*)&(pixelDataArray     [cnt]), sizeof(ViUInt32));
      memcpy(&buf[sizeof(ViUInt32)], (void*)&(wavelengthDataArray[cnt]), sizeof(ViReal64));
      err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, cnt, index_pt, (sizeof(ViUInt32) + sizeof(ViReal64)), buf);
      cnt++;
   }

   // write number of calibration points to NVMEM
   if(!err) err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, 0, index_num, sizeof(ViUInt16), (uint8_t *)&blen);

   return err;
}


/*---------------------------------------------------------------------------
   Function:   Read Pixel-Wavelength Data Set Valid Flag
   Purpose:    This function reads the flag that shows if the stored pixel-
               wavelength correlation data from the LC100's non volatile memory is valid (!= 0)
---------------------------------------------------------------------------*/
static ViStatus LC100_readWLCalValid  (ViSession instr, int target, ViUInt16 *flag)
{
   ViStatus    err = VI_SUCCESS;
   ViUInt16    To_Read_Bytes, Read_Bytes;
   ViUInt16    index;
      
   switch(target)
   {
      case TARGET_USER  :  index = LC100_NVMEM_USER_WL_FLAG;         break;
      case TARGET_FACT  :  index = LC100_NVMEM_FACT_WL_FLAG;         break;
      default           :  err = VI_ERROR_LC100_DRV_INV_USER_DATA;   break;
   }

   // read valid flag from NVMEM
   To_Read_Bytes = sizeof(ViUInt16);
   if(!err) err = LC100_usbIn(instr, LC100_WCMD_NV_MEMORY, 0, index, To_Read_Bytes, (uint8_t *)flag, &Read_Bytes);
   // if(To_Read_Bytes != Read_Bytes)  err = ...      /* we can do this later */
   
   return err;
}

/*---------------------------------------------------------------------------
   Function:   Write Pixel-Wavelength Data Set Valid Flag
   Purpose:    This function writes the flag that maeks the stored pixel-
               wavelength correlation data in the LC100's non volatile memory
               as valid (!= 0) or invalid (=0)
---------------------------------------------------------------------------*/
static ViStatus LC100_writeWLCalValid (ViSession instr, int target, ViUInt16 flag)
{
   ViStatus    err = VI_SUCCESS;
   ViUInt16    index;
      
   switch(target)
   {
      case TARGET_USER  :  index = LC100_NVMEM_USER_WL_FLAG;         break;
      case TARGET_FACT  :  index = LC100_NVMEM_FACT_WL_FLAG;         break;
      default           :  err = VI_ERROR_LC100_DRV_INV_USER_DATA;   break;
   }

   // write valid flag to NVMEM
   if(!err) err = LC100_usbOut(instr, LC100_WCMD_NV_MEMORY, 0, index, sizeof(ViUInt16), (uint8_t *)&flag);
   
   return err;
}


/*---------------------------------------------------------------------------
   Function:   Apply Pixel-Wavelength Data Set
   Purpose:    This function copies the 'bufferLength' (read out from NVMEM)
               given pixel-wavelength pairs from 'pixelDataArray' and
               'wavelengthDataArray' as well as the given polynomial coefficients
               (in form of a pointer to a   struct) to either the USER or the FACTORY part of 
---------------------------------------------------------------------------*/
static ViStatus LC100_applyPixelWavelengthPoints (LC100_data_t *data, int target, ViInt32 pixelDataArray[],
                                                   ViReal64 wavelengthDataArray[], ViInt32 bufferLength)
{
   ViStatus    err = VI_SUCCESS;

   switch(target)
   {
      case TARGET_USER  :
            // copy new values
            memcpy(data->user_points.cal_node_pixel, pixelDataArray,      sizeof(ViUInt32) * bufferLength);
            memcpy(data->user_points.cal_node_wl,    wavelengthDataArray, sizeof(ViReal64) * bufferLength);
            data->user_points.cal_node_cnt = bufferLength;
            break;

      case TARGET_FACT  :
            // copy new values
            memcpy(data->factory_points.cal_node_pixel, pixelDataArray,      sizeof(ViUInt32) * bufferLength);
            memcpy(data->factory_points.cal_node_wl,    wavelengthDataArray, sizeof(ViReal64) * bufferLength);
            data->factory_points.cal_node_cnt = bufferLength;
            break;
            
      default           :  err = VI_ERROR_LC100_DRV_INV_USER_DATA;   break;
   }

   return err;
}

/*---------------------------------------------------------------------------
   Function:   Copy Polynomial Data
   Purpose:    This function copies the given LC100_wl_cal_t struct to the
               LC100_data_t structure onn user or factory place
---------------------------------------------------------------------------*/
static ViStatus LC100_copyPolyData (LC100_data_t *data, int target, LC100_wl_cal_t *cal)
{
   ViStatus    err = VI_SUCCESS;

   switch(target)
   {
      case TARGET_USER  :
            memcpy(&(data->user_cal),    cal, sizeof(LC100_wl_cal_t));
            break;

      case TARGET_FACT  :
            memcpy(&(data->factory_cal), cal, sizeof(LC100_wl_cal_t));
            break;
            
      default           :  err = VI_ERROR_LC100_DRV_INV_USER_DATA;   break;
   }

   return err;
}


/*---------------------------------------------------------------------------
   Function:   Check Nodes
   Purpose:    Checks the calibration nodes for ascending.
---------------------------------------------------------------------------*/
static ViStatus LC100_checkNodes(ViInt32 pixel[], ViReal64 wl[], ViInt32 cnt)
{
   int      i;
   ViInt32  p;
   ViReal64 d;
   int      iDirectionFlagWl = 0;   // 1 means increasing, -1 means decreasing, 0 is an error
   int      iDirectionFlagPx = 0;   // 1 means increasing, -1 means decreasing, 0 is an error

   // check valid buffer length and determine target
   if((cnt < LC100_MIN_NUM_USR_ADJ) || (cnt > LC100_MAX_NUM_USR_ADJ)) return VI_ERROR_LC100_DRV_INV_USER_DATA;

   // check if values are decreasing
   if(wl[0] < wl[1])
   {
      iDirectionFlagWl = 1;   
   }
   else if(wl[0] > wl[1])  
   {
      iDirectionFlagWl = -1;  
   }
   else
      return VI_ERROR_LC100_DRV_INV_USER_DATA;
   
   
   if(pixel[0] < pixel[1])
   {
      iDirectionFlagPx = 1;   
   }
   else if(pixel[0] > pixel[1])  
   {
      iDirectionFlagPx = -1;  
   }
   else
      return VI_ERROR_LC100_DRV_INV_USER_DATA;
   
   
   // check pixel range
   if((pixel[0] < 0) || (pixel[cnt - 1] > (LC100_NUM_PIXELS - 1))) return VI_ERROR_LC100_DRV_INV_USER_DATA;
   
   // check wavelength range
   if(wl[0] <= 0.0)  return VI_ERROR_LC100_DRV_INV_USER_DATA; 
   
   // check monoton ascending wavelength and pixel values    
   p = pixel[0];
   d = wl[0];
   
   for(i = 1; i < cnt; i++)
   {
      // check increasing pixels...
      if(iDirectionFlagPx == 1)
      {
         if(pixel[i] <= p) return VI_ERROR_LC100_DRV_INV_USER_DATA;
      }
      else
      {
         if(pixel[i] >= p) return VI_ERROR_LC100_DRV_INV_USER_DATA;
      }
         
         
      if(iDirectionFlagWl == 1)  // increasing
      {
         if(wl[i] <= d) return VI_ERROR_LC100_DRV_INV_USER_DATA; 
      }
      else
      {
         if(wl[i] >= d) return VI_ERROR_LC100_DRV_INV_USER_DATA;
      }

      p = pixel[i];
      d = wl[i];
   } 
   
   /*
   for(i = 1; i < cnt; i++)
   {
      // check increasing pixels...
      if(pixel[i] <= p) return VI_ERROR_CCS_SERIES_INV_USER_DATA;
      
      if(iDirectionFlag == 1) // increasing
      {
         if(wl[i] <= d) return VI_ERROR_CCS_SERIES_INV_USER_DATA; 
      }
      else
      {
         if(wl[i] >= d) return VI_ERROR_CCS_SERIES_INV_USER_DATA;
      }

      p = pixel[i];
      d = wl[i];
   } 
   */
   
   return VI_SUCCESS;
}

/*---------------------------------------------------------------------------
   Function:   Nodes to Polynome
   Purpose:    Calculates polynome coefficients from user defined supporting
               points.
---------------------------------------------------------------------------*/
static ViStatus LC100_nodes2poly(ViInt32 pixel[], ViReal64 wl[], ViInt32 cnt, ViReal64 poly[])
{
   if(LeastSquareInterpolation ((int *)pixel, (double *)wl, (int)cnt, (double *)poly)) return VI_ERROR_LC100_DRV_INV_USER_DATA;
   return VI_SUCCESS;
}

/*---------------------------------------------------------------------------
   Function:   Polynom to Wavelength Array
   Purpose:    Calculates wavelenth array from polynom coefficients.
               The poly array must contain 4 elements.
---------------------------------------------------------------------------*/
static ViStatus LC100_poly2wlArray(LC100_wl_cal_t *wl)
{
   int      i;
   ViReal64 d = 0.0;
   int      iDirectionFlag = 0; // 1 means increasing, -1 means decreasing, 0 is an error
   
   for (i = 0; i < LC100_NUM_PIXELS; i++)
   {
      wl->wl[i] = wl->poly[0] + (double)i * (wl->poly[1] + (double)i * (wl->poly[2] + (double)i * wl->poly[3]));
   }
   
   // check if values are decreasing
   if(wl->wl[0] < wl->wl[1])
   {
      iDirectionFlag = 1;  
   }
   else if(wl->wl[0] > wl->wl[1])  
   {
      iDirectionFlag = -1; 
   }
   else
      return VI_ERROR_LC100_DRV_INV_USER_DATA;
   
   
   d = wl->wl[0];
   for(i = 1; i < LC100_NUM_PIXELS; i++)
   {
      if(iDirectionFlag == 1) // increasing
      {
         if(wl->wl[i] <= d)   return VI_ERROR_LC100_DRV_INV_USER_DATA; 
      }
      else
      {
         if(wl->wl[i] >= d)   return VI_ERROR_LC100_DRV_INV_USER_DATA;
      }

      d = wl->wl[i];
   }
   
   if(iDirectionFlag == 1)
   {
      wl->min     = wl->poly[0];
      wl->max     = wl->wl[LC100_NUM_PIXELS - 1];
   }
   else
   {
      wl->min     = wl->wl[LC100_NUM_PIXELS - 1]; 
      wl->max     = wl->poly[0];
   }
   
   return VI_SUCCESS;
}


/*-----------------------------------------------------------------------------
  Least Square logarithm.
-----------------------------------------------------------------------------*/
static int LeastSquareInterpolation (int * PixelArray, double * WaveLengthArray, int iLength, double Coefficients[])
{
   double B[MATRIX_ROWS];
   double A[MATRIX_ROWS][MATRIX_COLS],C[MATRIX_ROWS][MATRIX_COLS];
   double D[MATRIX_ROWS][MATRIX_COLS],E[MATRIX_ROWS][MATRIX_COLS],F[MATRIX_ROWS][MATRIX_COLS];
   double detA;
   double CompleteWavelengthArray[LC100_NUM_PIXELS];
   int i;
   
   if(iLength < MATRIX_ROWS)
      return -1;
   
   // clear CompleteWavelengthArray
   memset(CompleteWavelengthArray, 0, sizeof(double) * LC100_NUM_PIXELS);
   
   // fill CompleteWavelengthArray with the supporting points
   for(i = 0; i < iLength; i++)
   {
      if((PixelArray[i] < 0) || (PixelArray[i] > LC100_NUM_PIXELS))   break;
      CompleteWavelengthArray[PixelArray[i]] = WaveLengthArray[i];   
   }

   
   A[0][0] = iLength;
   A[0][1] = SpecSummation(CompleteWavelengthArray,1,LC100_NUM_PIXELS);
   A[0][2] = SpecSummation(CompleteWavelengthArray,2,LC100_NUM_PIXELS);
   A[0][3] = SpecSummation(CompleteWavelengthArray,3,LC100_NUM_PIXELS);

   A[1][0] = SpecSummation(CompleteWavelengthArray,1,LC100_NUM_PIXELS);
   A[1][1] = SpecSummation(CompleteWavelengthArray,2,LC100_NUM_PIXELS);
   A[1][2] = SpecSummation(CompleteWavelengthArray,3,LC100_NUM_PIXELS);
   A[1][3] = SpecSummation(CompleteWavelengthArray,4,LC100_NUM_PIXELS);

   A[2][0] = SpecSummation(CompleteWavelengthArray,2,LC100_NUM_PIXELS);
   A[2][1] = SpecSummation(CompleteWavelengthArray,3,LC100_NUM_PIXELS);
   A[2][2] = SpecSummation(CompleteWavelengthArray,4,LC100_NUM_PIXELS);
   A[2][3] = SpecSummation(CompleteWavelengthArray,5,LC100_NUM_PIXELS);

   A[3][0] = SpecSummation(CompleteWavelengthArray,3,LC100_NUM_PIXELS);
   A[3][1] = SpecSummation(CompleteWavelengthArray,4,LC100_NUM_PIXELS);
   A[3][2] = SpecSummation(CompleteWavelengthArray,5,LC100_NUM_PIXELS);
   A[3][3] = SpecSummation(CompleteWavelengthArray,6,LC100_NUM_PIXELS);
   
   
   B[0] = Summation(CompleteWavelengthArray,LC100_NUM_PIXELS);
   B[1] = Spec2Summation(CompleteWavelengthArray,1,LC100_NUM_PIXELS);
   B[2] = Spec2Summation(CompleteWavelengthArray,2,LC100_NUM_PIXELS);
   B[3] = Spec2Summation(CompleteWavelengthArray,3,LC100_NUM_PIXELS);
   
   MergeMC(A,B,C,0);
   MergeMC(A,B,D,1);
   MergeMC(A,B,E,2);
   MergeMC(A,B,F,3);
   
   detA = Determinant(A);
   
   if (detA == 0.0)
   {
      return -1;
   }

   Coefficients[0] = Determinant(C) / detA;
   Coefficients[1] = Determinant(D) / detA;
   Coefficients[2] = Determinant(E) / detA;
   Coefficients[3] = Determinant(F) / detA;
   
   return 0;
}

static double Summation(double *pdata, int values)
{
   double tmp=0.0;

   do
   {
      tmp += *pdata;
      ++pdata;
   }
   while (--values);

   return(tmp);
}


static double SpecSummation(double *pcorrel,int pwr,int values)
{
   double tmp=0.0;
   int pixel=0;

   do
   {
      if (*pcorrel > 0.0) tmp += pow((double)pixel,(double)pwr);
      ++pcorrel;
      ++pixel;
   }
   while (--values);

   return(tmp);
}

static double Spec2Summation(double *pcorrel,int pwr,int values)
{
   double tmp=0.0;
   int pixel=0;

   do
   {
      if (*pcorrel > 0.0) tmp += *pcorrel * pow((double)pixel,(double)pwr);
      ++pcorrel;
      ++pixel;
   }
   while (--values);

   return(tmp);
}


static void MergeMC(double s[MATRIX_ROWS][MATRIX_COLS], double i[MATRIX_ROWS], double d[MATRIX_ROWS][MATRIX_COLS], int column)
{
   int x,y;

   for (x = 0; x < MATRIX_COLS; x++)
   {
      if (x == column) 
      {
         for (y = 0; y < MATRIX_ROWS; y++) 
            d[y][x] = i[y];
      }
      else
      {
         for (y = 0; y < MATRIX_ROWS; y++)
            d[y][x] = s[y][x];
      }
   }
}



static double Determinant(double mt[MATRIX_ROWS][MATRIX_COLS])
{
   double a=mt[0][0],b=mt[0][1],c=mt[0][2],d=mt[0][3];
   double e=mt[1][0],f=mt[1][1],g=mt[1][2],h=mt[1][3];
   double i=mt[2][0],j=mt[2][1],k=mt[2][2],l=mt[2][3];
   double m=mt[3][0],n=mt[3][1],o=mt[3][2],p=mt[3][3];
   double tmp=0.0;

   tmp =  a * f * k * p;
   tmp -= a * f * l * o;
   tmp -= a * j * g * p;
   tmp += a * j * h * o;
   tmp += a * n * g * l;
   tmp -= a * n * h * k;
   tmp -= e * b * k * p;
   tmp += e * b * l * o;
   tmp += e * j * c * p;
   tmp -= e * j * d * o;
   tmp -= e * n * c * l;
   tmp += e * n * d * k;
   tmp += i * b * g * p;
   tmp -= i * b * h * o;
   tmp -= i * f * c * p;
   tmp += i * f * d * o;
   tmp += i * n * c * h;
   tmp -= i * n * d * g;
   tmp -= m * b * g * l;
   tmp += m * b * h * k;
   tmp += m * f * c * l;
   tmp -= m * f * d * k;
   tmp -= m * j * c * h;
   tmp += m * j * d * g;

   return(tmp);
}


/****************************************************************************

  End of file

****************************************************************************/
