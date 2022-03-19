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


   Header file

   Date:          Oct-26-2011
   Built with:    NI LabWindows/CVI 2010
   Software-Nr:   091.93.xxx
   Version:       1.2.3

   Changelog:     see 'readme.rtf'

****************************************************************************/

#ifndef __LC100_DRV_H__
#define __LC100_DRV_H__
#include <ni-visa/visa.h>


#ifdef __cplusplus
extern "C" {
#endif


/*===========================================================================

 Macros

===========================================================================*/
/*---------------------------------------------------------------------------
  LC100 device properties
---------------------------------------------------------------------------*/
#define LC100_NUM_EVALBOX              16
#define LC100_NUM_GPIO                 5

#define LC100_REAL_TO_UINT16           ((double)0xFFFF)
#define LC100_UINT16_TO_REAL           (1.0 / LC100_REAL_TO_UINT16)

#define LC100_CLOCK_FREQUENCY          80000000.0                    // 80 MHz
#define LC100_CLOCK_PERIOD             (1.0 / LC100_CLOCK_FREQUENCY) // 12.5ns

#define LC100_AMP_CORR_FACT_MIN         0.001            // the minimum correction factor
#define LC100_AMP_CORR_FACT_MAX         15.999755859375  // the maximum correction factor, standard is 1.0
// this is (2**16 - 1) / (2**12) as processed in FPGA

/*---------------------------------------------------------------------------
  USB stuff
---------------------------------------------------------------------------*/
#define LC100_VID                      (0x1313)    // Thorlabs
#define LC100_PID_ILX554B              (0x80A0)    // LC100 series PID (ILX 554B sensor)
// reserved for future use
#define LC100_PID_RESERVED_1           (0x80A1)
#define LC100_PID_RESERVED_2           (0x80A2)
#define LC100_PID_RESERVED_3           (0x80A3)
#define LC100_PID_RESERVED_4           (0x80A4)
#define LC100_PID_RESERVED_5           (0x80A5)
#define LC100_PID_RESERVED_6           (0x80A6)
#define LC100_PID_RESERVED_7           (0x80A7)


// Macros for Cypress USB chip
#define EZUSB_STD_BULKOUT              2
#define EZUSB_STD_BULKIN               6
#define EZUSB_EP0_TRANSFER_SIZE        64


/*---------------------------------------------------------------------------
 Find pattern for 'viFindRsrc()'
---------------------------------------------------------------------------*/
#define LC100_FIND_PATTERN             "USB?*RAW{VI_ATTR_MANF_ID==0x1313 && ((VI_ATTR_MODEL_CODE>=0x80A0) && (VI_ATTR_MODEL_CODE<=0x80AF))}"

/*---------------------------------------------------------------------------
 Buffers
---------------------------------------------------------------------------*/
#define LC100_BUFFER_SIZE              256      // General buffer size
#define LC100_ERR_DESCR_BUFFER_SIZE    512      // Buffer size for error messages
#define LC100_SENS_DESCR_BUFFER_SIZE   256      // Buffer size for sensor description text
#define LC100_NUM_PIXELS               2048     // this is the number of effective pixels of CCD
#define LC100_MAX_USER_NAME_SIZE       64       // this includes the trailing '\0', is the size if a single USB control transfer
#define LC100_NUM_RAW_PIXELS           LC100_NUM_PIXELS

#define LC100_MIN_NUM_USR_ADJ          4        // minimum number of user adjustment data points (wavelength <-> pixel)
#define LC100_MAX_NUM_USR_ADJ          10       // maximum number of user adjustment data points (wavelength <-> pixel)

/*---------------------------------------------------------------------------
 Driver attributes
---------------------------------------------------------------------------*/
#define LC100_ATTR_USER_DATA           (VI_ATTR_USER_DATA)  // we use the same value as VISA itself (0x3FFF0007UL)

/*---------------------------------------------------------------------------
 GET / SET attributes
---------------------------------------------------------------------------*/
#define LC100_ATTR_ACT                 (0)         // get/set the actual value
#define LC100_ATTR_MIN                 (1)         // get the minimum value
#define LC100_ATTR_MAX                 (2)         // get the maximum value
#define LC100_ATTR_DEF                 (3)         // get the default value

/*---------------------------------------------------------------------------
 Error/Warning Codes
   Driver Error Codes in the range 0xBFFC0800 ... 0xBFFC0FFF according to
   recommendation 3.15 of VXI plug&play Systems Alliance VPP-3.4 Instrument
   Driver Programmatic Developer Interface Spec. Rev. 2.4
---------------------------------------------------------------------------*/
#define VI_ERROR_NSUP_COMMAND          (_VI_ERROR + 0x3FFC0801L)     // 0xBFFC0801
#define VI_ERROR_LC100_UNKNOWN         (_VI_ERROR + 0x3FFC0802L)     // 0xBFFC0802
#define VI_ERROR_NSUP_DEVICE           (_VI_ERROR + 0x3FFC0803L)     // 0xBFFC0803

#define VI_ERROR_XSVF_SIZE             (_VI_ERROR + 0x3FFC0A00L)     // 0xBFFC0A00
#define VI_ERROR_XSVF_MEMORY           (_VI_ERROR + 0x3FFC0A01L)     // 0xBFFC0A01
#define VI_ERROR_XSVF_FILE             (_VI_ERROR + 0x3FFC0A02L)     // 0xBFFC0A02

#define VI_ERROR_FIRMWARE_SIZE         (_VI_ERROR + 0x3FFC0A10L)     // 0xBFFC0A10
#define VI_ERROR_FIRMWARE_MEMORY       (_VI_ERROR + 0x3FFC0A11L)     // 0xBFFC0A11
#define VI_ERROR_FIRMWARE_FILE         (_VI_ERROR + 0x3FFC0A12L)     // 0xBFFC0A12
#define VI_ERROR_FIRMWARE_CHKSUM       (_VI_ERROR + 0x3FFC0A13L)     // 0xBFFC0A13
#define VI_ERROR_FIRMWARE_BUFOFL       (_VI_ERROR + 0x3FFC0A14L)     // 0xBFFC0A14
#define VI_ERROR_FIRMWARE_MISMATCH     (_VI_ERROR + 0x3FFC0A15L)     // 0xBFFC0A15

#define VI_ERROR_CYEEPROM_SIZE         (_VI_ERROR + 0x3FFC0A20L)     // 0xBFFC0A20
#define VI_ERROR_CYEEPROM_MEMORY       (_VI_ERROR + 0x3FFC0A21L)     // 0xBFFC0A21
#define VI_ERROR_CYEEPROM_FILE         (_VI_ERROR + 0x3FFC0A22L)     // 0xBFFC0A22
#define VI_ERROR_CYEEPROM_CHKSUM       (_VI_ERROR + 0x3FFC0A23L)     // 0xBFFC0A23
#define VI_ERROR_CYEEPROM_BUFOVL       (_VI_ERROR + 0x3FFC0A24L)     // 0xBFFC0A24

#define VI_ERROR_IIC_SIZE              (_VI_ERROR + 0x3FFC0A30L)     // 0xBFFC0A30
#define VI_ERROR_IIC_MEMORY            (_VI_ERROR + 0x3FFC0A31L)     // 0xBFFC0A31
#define VI_ERROR_IIC_FILE              (_VI_ERROR + 0x3FFC0A32L)     // 0xBFFC0A32
#define VI_ERROR_IIC_CHKSUM            (_VI_ERROR + 0x3FFC0A33L)     // 0xBFFC0A33
#define VI_ERROR_IIC_BUFOVL            (_VI_ERROR + 0x3FFC0A34L)     // 0xBFFC0A34

#define VI_ERROR_NVMEM_CHKSUM          (_VI_ERROR + 0x3FFC0A40L)     // 0xBFFC0A40


// this is the offset added to error from asking the device when it stalled
#define VI_ERROR_USBCOMM_OFFSET        (_VI_ERROR + 0x3FFC0B00L)     // 0xBFFC0B00

// LC100 Error Codes (codes that will be returned from the device itself)

#define VI_ERROR_LC100_ENDP0_SIZE        (VI_ERROR_USBCOMM_OFFSET + 0x01) // 0xBFFC0B01
#define VI_ERROR_LC100_EEPROM_ADR_TO_BIG (VI_ERROR_USBCOMM_OFFSET + 0x02) // 0xBFFC0B02
#define VI_ERROR_LC100_INVAL_RCMD        (VI_ERROR_USBCOMM_OFFSET + 0x03) // 0xBFFC0B03
#define VI_ERROR_LC100_INVAL_WCMD        (VI_ERROR_USBCOMM_OFFSET + 0x04) // 0xBFFC0B04
#define VI_ERROR_LC100_INVAL_INDEX       (VI_ERROR_USBCOMM_OFFSET + 0x05) // 0xBFFC0B05
#define VI_ERROR_LC100_INVAL_LEN         (VI_ERROR_USBCOMM_OFFSET + 0x06) // 0xBFFC0B06
#define VI_ERROR_LC100_INVAL_VALUE       (VI_ERROR_USBCOMM_OFFSET + 0x07) // 0xBFFC0B07

#define VI_ERROR_LC100_FPGA_UNKNOWN      (VI_ERROR_USBCOMM_OFFSET + 0x08) // 0xBFFC0B08
#define VI_ERROR_LC100_FPGA_COMM_ERR     (VI_ERROR_USBCOMM_OFFSET + 0x09) // 0xBFFC0B09

#define VI_ERROR_LC100_XSVF_UNKNOWN      (VI_ERROR_USBCOMM_OFFSET + 0x11) // 0xBFFC0B11
#define VI_ERROR_LC100_XSVF_TDOMISMATCH  (VI_ERROR_USBCOMM_OFFSET + 0x12) // 0xBFFC0B12
#define VI_ERROR_LC100_XSVF_MAXRETRIES   (VI_ERROR_USBCOMM_OFFSET + 0x13) // 0xBFFC0B13
#define VI_ERROR_LC100_XSVF_ILLEGALCMD   (VI_ERROR_USBCOMM_OFFSET + 0x14) // 0xBFFC0B14
#define VI_ERROR_LC100_XSVF_ILLEGALSTATE (VI_ERROR_USBCOMM_OFFSET + 0x15) // 0xBFFC0B15
#define VI_ERROR_LC100_XSVF_DATAOVERFLOW (VI_ERROR_USBCOMM_OFFSET + 0x16) // 0xBFFC0B16

#define VI_ERROR_LC100_I2C_NACK          (VI_ERROR_USBCOMM_OFFSET + 0x20) // 0xBFFC0B20
#define VI_ERROR_LC100_I2C_ERR           (VI_ERROR_USBCOMM_OFFSET + 0x21) // 0xBFFC0B21
#define VI_ERROR_LC100_I2C_SIZE          (VI_ERROR_USBCOMM_OFFSET + 0x22) // 0xBFFC0B22
#define VI_ERROR_LC100_FIFOS_NOT_EMPTY   (VI_ERROR_USBCOMM_OFFSET + 0x23) // 0xBFFC0B23
#define VI_ERROR_LC100_WRONG_OPMODE      (VI_ERROR_USBCOMM_OFFSET + 0x24) // 0xBFFC0B24
#define VI_ERROR_LC100_NVMEM_BAD         (VI_ERROR_USBCOMM_OFFSET + 0x25) // 0xBFFC0B25
#define VI_ERROR_LC100_NVMEM_FIRST       (VI_ERROR_USBCOMM_OFFSET + 0x26) // 0xBFFC0B26
#define VI_ERROR_LC100_NVMEM_VER_CHANGED (VI_ERROR_USBCOMM_OFFSET + 0x27) // 0xBFFC0B27

#define VI_ERROR_LC100_SERVICE_MODE_ONLY (VI_ERROR_USBCOMM_OFFSET + 0x2C) // 0xBFFC0B2C
#define VI_ERROR_LC100_EVAL_BOX          (VI_ERROR_USBCOMM_OFFSET + 0x30) // 0xBFFC0B30
#define VI_ERROR_LC100_INVALID_USER_DATA (VI_ERROR_USBCOMM_OFFSET + 0x31) // 0xBFFC0B31

// LC100 Driver Error Codes (codes that will be returned from the driver)
#define VI_ERROR_LC100_DRV_INV_USER_DATA (VI_ERROR_USBCOMM_OFFSET + 0x80) // 0xBFFC0B80
#define VI_ERROR_LC100_NO_USER_DATA      (VI_ERROR_USBCOMM_OFFSET + 0x81) // 0xBFFC0B81


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
ViStatus _VI_FUNC LC100_init (ViRsrc resourceName, ViBoolean IDQuery, ViBoolean resetDevice, ViSession *instrHandle);


/*---------------------------------------------------------------------------
   Function:   Close
   Purpose:    This function closes an instrument driver session.

   Parameters:

   ViSession instrHandle:  The session to close.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_close (ViSession instrHandle);


/*===========================================================================

 Configuration Functions.

===========================================================================*/
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
ViStatus _VI_FUNC LC100_setIntegrationTime (ViSession instrHandle, ViReal64 integrationTime);


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
ViStatus _VI_FUNC LC100_getIntegrationTime (ViSession instrHandle, ViInt16 attribute, ViReal64 *integrationTime);


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
// macros for operating mode
#define OPMODE_IDLE                    0
#define OPMODE_SW_SINGLE_SHOT          1
#define OPMODE_SW_LOOP                 2
#define OPMODE_HW_SINGLE_SHOT          3
#define OPMODE_HW_LOOP                 4

ViStatus _VI_FUNC LC100_setOperatingMode (ViSession instrHandle, ViInt32 opMode);

/*---------------------------------------------------------------------------
   Function:   Get Operating Mode
   Purpose:    This function returns the actual operating mode.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViPInt32 opMode:           The actual operation mode. See "LC100_setOperatingMode"
                              function for valid values.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getOperatingMode (ViSession instrHandle, ViInt32 *opMode);


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
ViStatus _VI_FUNC LC100_setHWAveragingMode (ViSession instr, ViInt32 hwAvgMode);

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
ViStatus _VI_FUNC LC100_getHWAveragingMode (ViSession instr, ViInt16 attribute, ViInt32 *hwAvgMode);

/*---------------------------------------------------------------------------
   Function:   Set Intensity Correction
   Purpose:    This function sets intensity correction values. Each value represents
               the multiplier for the corresponding pixel.

   Parameters:

   ViSession instrHandle:        The actual session to opened device.
   ViReal64 correctionValues[]:  The intensity correction values array
                                 (LC100_NUM_PIXELS elements).
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_setIntensityCorr (ViSession instrHandle, ViReal64 correctionValues[]);


/*---------------------------------------------------------------------------
   Function:   Get Intensity Correction
   Purpose:    This function returns the intensity correction values. Each value represents
               the multiplier for the corresponding pixel.

   Parameters:

   ViSession instrHandle:        The actual session to opened device.
   ViReal64 correctionValues[]:  The intensity correction values array
                                 (LC100_NUM_PIXELS elements).
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getIntensityCorr (ViSession instrHandle, ViReal64 correctionValues[]);


/*---------------------------------------------------------------------------
   Function:   Reset Intensity Correction
   Purpose:    This function reset the intensity correction.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_resetIntensityCorr (ViSession instrHandle);


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
ViStatus _VI_FUNC LC100_setEvalBox (ViSession instrHandle, ViInt16 EvalBox, ViInt32 x1, ViInt32 x2, ViReal64 y1, ViReal64 y2);


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
ViStatus _VI_FUNC LC100_getEvalBox (ViSession instrHandle, ViInt16 EvalBox, ViPInt32 x1, ViPInt32 x2, ViPReal64 y1, ViPReal64 y2);


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
// macros for GPIO Feed Mode (from EVALBOXes)
#define GPIO_FEED_MODE_IGNORE       0  // no evaluation
#define GPIO_FEED_MODE_L            1  // LOW condition; is valid when measurement signal is below the lower limit of the complete eval box
#define GPIO_FEED_MODE_LC           2  // LOW-CENTER condition; is valid when measurement signal is below or inside the limits of the complete eval box
#define GPIO_FEED_MODE_C            3  // CENTER condition; is valid when measurement signal is only inside the limits of the complete eval box
#define GPIO_FEED_MODE_CH           4  // CENTER-HIGH condition; is valid when measurement signal is above or inside the limits of the complete eval box
#define GPIO_FEED_MODE_H            5  // HIGH condition; is valid when measurement signal is only above the limits of the complete eval box
#define GPIO_FEED_MODE_LCH          6  // LOW-CENTER-HIGH condition; is valid when measurement signal is below or inside or above the limits of the complete eval box
#define GPIO_FEED_GPIN_NONINV       7  // additional GPIO to evaluate; signal non-inverted
#define GPIO_FEED_GPIN_INV          8  // additional GPIO to evaluate; signal inverted

ViStatus _VI_FUNC LC100_setGPIOFeed (ViSession instrHandle, ViInt16 GPIO, ViInt16 feed, ViInt16 mode);

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
ViStatus _VI_FUNC LC100_getGPIOFeed (ViSession instrHandle, ViInt16 GPIO, ViInt16 feed, ViInt16 *mode);


/*---------------------------------------------------------------------------
   Function:   Set GPIO Mode
   Purpose:    This function sets the GPIO mode.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt16 GPIO:              The GPIO number (0 to LC100_NUM_GPIO - 1).
   ViInt16 mode:              The GPIO mode. See GPIO mode macros for
                              detailed information.
---------------------------------------------------------------------------*/
// macros for GPIO Mode
#define GPIO_MODE_INPUT             0 // GPIO is used as input
#define GPIO_MODE_OUTPUT_POS        1 // always high
#define GPIO_MODE_OUTPUT_NEG        2 // always low
#define GPIO_MODE_OUT_EVAL_POS      3 // high when evaluation pattern is found
#define GPIO_MODE_OUT_EVAL_NEG      4 // low when evaluation pattern is found
#define GPIO_MODE_OUT_EXPOSURE_POS  5 // high during exposure
#define GPIO_MODE_OUT_EXPOSURE_NEG  6 // low during exposure
#define GPIO_MODE_OUT_FLASH_POS     7 // high for specified strobe/flash parameters
#define GPIO_MODE_OUT_FLASH_NEG     8 // low for specified strobe/flash parameters

ViStatus _VI_FUNC LC100_setGPIOMode (ViSession instrHandle, ViInt16 GPIO, ViInt16 mode);


/*---------------------------------------------------------------------------
   Function:   Get GPIO Mode
   Purpose:    This function returns the GPIO mode.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt16 GPIO:              The GPIO number (0 to LC100_NUM_GPIO - 1).
   ViPInt16 mode:             The GPIO mode. See GPIO mode macros for
                              detailed information.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getGPIOMode (ViSession instrHandle, ViInt16 GPIO, ViInt16 *mode);


/*---------------------------------------------------------------------------
   Function:   Get GPIO State
   Purpose:    This function returns the state of one or all GPIOs.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt16 GPIO:              The desired GPIO number.
                              0...4 for GPIO1...5
                              any other value will request the GPIO status register.
   ViPUInt32 state:           Either the state of the specified GPIO (0 or 1) or
                              the GPIO state register.
                              Bit 0 of the state register represents the state of
                              GPIO 1, Bit 1 represents the state of GPIO 2 and so on.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getGPIOState (ViSession instr, ViInt16 GPIO, ViUInt32 *state);


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
// macros for analog output mode
#define LC100_AOUT_MODE_FIXED       0  // fixed voltage output between 0V and 4V
#define LC100_AOUT_MODE_VARIABLE    1  // variable voltage output at a specified pixel

ViStatus _VI_FUNC LC100_setAnalogOutputMode (ViSession instrHandle, ViUInt32  mode);


/*---------------------------------------------------------------------------
   Function:   Get Analog Output Mode
   Purpose:    This function returns the mode of the analog output.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViPUInt32  mode:           The analog output mode. See "LC100_setAnalogOutputMode"
                              function for more information.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getAnalogOutputMode (ViSession instrHandle, ViUInt32 *mode);


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
// macros analog output configuration
#define LC100_AOUT_SET_VOLT         0  // fixed output voltage (output mode must be set to "LC100_AOUT_MODE_FIXED"
#define LC100_AOUT_MIN_VOLT         1  // minimal analog output voltage (read-only)
#define LC100_AOUT_MAX_VOLT         2  // maximal analog output voltage (read-only)
#define LC100_AOUT_VOLT_IMIN        3  // minimal voltage which is outputed when the monitored pixel has an intensity below (or equal) the configured level
#define LC100_AOUT_VOLT_IMAX        4  // maximal voltage which is outputed when the monitored pixel has an intensity above (or equal) the configured level

ViStatus _VI_FUNC LC100_setAnalogOutputVoltage (ViSession instrHandle, ViInt16 select, ViReal64  voltage);


/*---------------------------------------------------------------------------
   Function:   Get Analog Output Voltage
   Purpose:    This function returns a specified analog output voltage.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt16 select:            The voltage to get. See "LC100_setAnalogOutputVoltage"
                              function for more information.
   ViPReal64 voltage:         The voltage in Volt.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getAnalogOutputVoltage (ViSession instrHandle, ViInt16 select, ViReal64 *voltage);


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
ViStatus _VI_FUNC LC100_setAnalogOutputPixel (ViSession instrHandle, ViInt16  pixel, ViReal64  imax, ViReal64  imin);


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
ViStatus _VI_FUNC LC100_getAnalogOutputPixel (ViSession instrHandle, ViInt16 *pixel, ViReal64 *imax, ViReal64 *imin);


/*---------------------------------------------------------------------------
   Function:   Set Trigger Delay
   Purpose:    This function sets the trigger delay in seconds.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViReal64 delay:            The trigger delay in seconds. The
                              allowed range for this parameter can be queried by
                              "LC100_getTriggerDelay".
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_setTriggerDelay (ViSession instrHandle, ViReal64 delay);


/*---------------------------------------------------------------------------
   Function:   Get Trigger Delay
   Purpose:    This function queries the trigger delay.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt16 select:            The trigger delay to retrieve. See macros below
                              for more information.
   ViPReal64 delay:           The trigger delay in seconds.
---------------------------------------------------------------------------*/
#define LC100_TRIG_DELAY_SET  0  // the actual set trigger delay
#define LC100_TRIG_DELAY_MIN  1  // the minimum trigger delay
#define LC100_TRIG_DELAY_MAX  2  // the maximum trigger delay

ViStatus _VI_FUNC LC100_getTriggerDelay (ViSession instrHandle, ViInt16 select, ViPReal64 delay);


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
#define LC100_FLASH_DELAY_SET    3  // the actual flash delay
#define LC100_FLASH_DELAY_MIN    4  // the minimum flash delay (read-only)
#define LC100_FLASH_DELAY_MAX    5  // the maximum flash delay (read-only)
#define LC100_FLASH_DURATION_SET 6  // the actual flash duration
#define LC100_FLASH_DURATION_MIN 7  // the minimum flash duration (read-only)
#define LC100_FLASH_DURATION_MAX 8  // the maximum flash duration (read-only)

ViStatus _VI_FUNC LC100_setFlashParams (ViSession instrHandle, ViInt16 select, ViReal64 value);


/*---------------------------------------------------------------------------
   Function:   Get Flash Parameters
   Purpose:    This function queries the flash parameters.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViInt16 select:            The flash parameter to query. See "LC100_setFlashParams"
                              function for more information about valid values.
   ViPReal64 value:           The requested flash parameter in seconds.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getFlashParams (ViSession instrHandle, ViInt16 select, ViPReal64 value);


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
ViStatus _VI_FUNC LC100_startScan (ViSession instrHandle);


/*---------------------------------------------------------------------------
   Function:   Start Continuous Scan
   Purpose:    This function starts the free running mode.

   Note:
   The scan data can be read out with the function 'Get Scan Data'
   Use 'Get Device Status' to check the scan status.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_startScanCont (ViSession instrHandle);


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
ViStatus _VI_FUNC LC100_startScanExtTrg (ViSession instrHandle);


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
ViStatus _VI_FUNC LC100_startScanContExtTrg (ViSession instrHandle);


/*---------------------------------------------------------------------------
 Function:     Get Device Status
 Purpose:      This function returns 32bit status information.

Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViPInt32 deviceStatus:     The status register.
---------------------------------------------------------------------------*/
#define LC100_STATUS_IDLE        0x00000000  // camera is idle
#define LC100_STATUS_TRANSFER    0x00000001  // scan is done, waiting for data transfer to PC
#define LC100_STATUS_TRIGGERED   0x00000002  // scan is in progress
#define LC100_STATUS_ARMED       0x00000003  // external trigger is armed and camera waits for trigger

ViStatus _VI_FUNC LC100_getDeviceStatus (ViSession instrHandle, ViInt32 *deviceStatus);

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
#define LC100_SETUP_RESET_RAM 0     // set the LC100 to its factory settings, current session only, do not alter NVMEM, next power up will run with old settings
#define LC100_SETUP_STORE_RAM 1     // store the current session setting to NVMEM, next power up will run with these settings
#define LC100_SETUP_RESET_NV  2     // set the LC100 to its factory settings in RAM as well as in NVMEM, next power up will run with factory default settings
#define LC100_SETUP_RELOAD    3     // set the LC100 to the settings of the last power up, i.e. the NVMEM settings, like a reboot

#define FILTER_IOSETUP        0x01  // affects settings set with LC100_setGPIOFeed(); and LC100_setGPIOMode();
#define FILTER_EVALBOX        0x02  // affects settings set with LC100_setEvalBox();
#define FILTER_TRIGGER        0x04  // affects settings set with LC100_setTriggerDelay(); and LC100_setFlashParams();
#define FILTER_DACOUT         0x08  // affects settings set with LC100_setAnalogOutputMode();, LC100_setAnalogOutputVoltage(); and  LC100_setAnalogOutputPixel();
#define FILTER_CCD            0x10  // affects settings set with LC100_setIntegrationTime(); and LC100_setOperatingMode();
#define FILTER_USER_SETUP     0x1F  // affects alls settings above

ViStatus _VI_FUNC LC100_Setup (ViSession instrHandle, ViUInt16 mode, ViUInt16 filter);

/*---------------------------------------------------------------------------
   Function:   Get Scan Data
   Purpose:    This function reads out the processed scan data.

   Parameters:

   ViSession instr:           The actual session to opened device.
   ViUInt16 data[]:           The measurement array (LC100_NUM_PIXELS elements).
                              Each pixel can have a value from 0 to 65535.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getScanData (ViSession instrHandle, ViUInt16 data[]);


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
   ViInt16 dataSet:                 The desired data set (see #defines below).
   ViInt32  pixelDataArray[]:       The pixel data.
   ViReal64 wavelengthDataArray[]:  The wavelength data.
   ViInt32 bufferLength:            The length of both arrays.
---------------------------------------------------------------------------*/
#define LC100_CAL_DATA_SET_FACTORY        0
#define LC100_CAL_DATA_SET_USER           1
#define LC100_CAL_DATA_SET_FACTORY_NVMEM  2
#define LC100_CAL_DATA_SET_USER_NVMEM     3

ViStatus _VI_FUNC LC100_setWavelengthData (ViSession instrHandle, ViInt16 dataSet, ViInt32 pixelDataArray[],
                                           ViReal64 wavelengthDataArray[], ViInt32 bufferLength);

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
   which provides the wavelength at pixel LC100_NUM_PIXELS-1.
   This is the maximum wavelength.

   Parameters:

   ViSession instrHandle:           The actual session to opened device.
   ViInt16 dataSet:                 The desired data set (see #defines at function LC100_setWavelengthData()).
   ViReal64 wavelengthArray[]:      The wavelength data.
   ViPReal64 minimumWavelength:     The minimum wavelength.
   ViPReal64 maximumWavelength:     The maximum wavelength.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getWavelengthData (ViSession instrHandle, ViInt16 dataSet, ViReal64 wavelengthArray[],
                                           ViPReal64 minimumWavelength, ViPReal64 maximumWavelength);

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
ViStatus _VI_FUNC LC100_getCalibrationPoints (ViSession instrHandle, ViInt16 dataSet, ViInt32 pixelDataArray[],
                                              ViReal64 wavelengthDataArray[], ViPInt32 bufferLength);


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
   ViReal64 AmpCorrFact[]:          The array with amplitude correction factors
   ViInt32 bufferLength:            The length of the array.
   ViInt32 bufferStart:             The start index for the array.
   ViInt32 mode                     With mode one can select if the new data will be applied to current measurements
                                    only (ACOR_APPLY_TO_MEAS) or additionally goes to non volatile memory of the
                                    device too (ACOR_APPLY_TO_MEAS_NVMEM) or goes to non volatile memory
                                    only (ACOR_APPLY_TO_NVMEM_ONLY).
                                    If mode is not one of the two predefined macros the function returns VI_ERROR_INV_PARAMETER
---------------------------------------------------------------------------*/
#define FACTORY_SET_START           19901201 // macro needed for service only, do not use

#define ACOR_APPLY_TO_MEAS          1
#define ACOR_APPLY_TO_MEAS_NVMEM    2
#define ACOR_APPLY_TO_NVMEM_ONLY    3

ViStatus _VI_FUNC LC100_setAmplitudeData (ViSession instr, ViReal64 AmpCorrFact[],
                                          ViInt32 bufferStart, ViInt32 bufferLength, ViInt32 mode);



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
                                    If mode is not one of the predefined macros the function returns VI_ERROR_INV_PARAMETER
---------------------------------------------------------------------------*/
#define ACOR_FROM_CURRENT           1
#define ACOR_FROM_NVMEM             2
#define ACOR_FROM_APPLIED           3  // do not use, for service only


ViStatus _VI_FUNC LC100_getAmplitudeData (ViSession instr, ViReal64 AmpCorrFact[],
                                          ViInt32 bufferStart, ViInt32 bufferLength, ViInt32 mode);


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
ViStatus _VI_FUNC LC100_identificationQuery (ViSession instrHandle, ViChar manufacturerName[], ViChar deviceName[], ViChar serialNumber[], ViChar firmwareRevision[]);


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
ViStatus _VI_FUNC LC100_sensorTypeQuery (ViSession instrHandle, ViInt16 *numberOfPixels, ViReal64 *minIntegrationTime, ViReal64 *maxIntegrationTime, ViChar description[]);


/*---------------------------------------------------------------------------
   Function:   Revision Query
   Purpose:    This function returns the revision numbers of the instrument
               driver and the device firmware.

   Parameters:

   ViSession instrHandle:              The actual session to opened device.
   ViChar instrumentDriverRevision[]:  The instrument driver revision.
   ViChar firmwareRevision[]:          The firmware revision.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_revisionQuery (ViSession instrHandle, ViChar instrumentDriverRevision[], ViChar firmwareRevision[]);


/*---------------------------------------------------------------------------
   Function:   Reset
   Purpose:    This function resets the device.

   Parameters:

   ViSession instr:  The actual session to opened device.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_reset (ViSession instrHandle);


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
ViStatus _VI_FUNC LC100_errorMessage(ViSession instrHandle, ViStatus statusCode, ViChar description[]);


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
ViStatus _VI_FUNC LC100_setAttribute (ViSession instrHandle, ViAttr attribute, ViUInt32 value);


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
ViStatus _VI_FUNC LC100_getAttribute (ViSession instrHandle, ViAttr attribute, ViUInt32 *value);


/*---------------------------------------------------------------------------
   Function:   Set User Text
   Purpose:    This function writes the given string to the novolatile memory of
               the LC100.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViChar userText:           The new user text. The string will be truncated
                              to 64 characters maximum.
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_setUserText (ViSession instrHandle, ViChar userText[]);


/*---------------------------------------------------------------------------
   Function:   Get User Text
   Purpose:    This function reads the user text from the novolatile memory of
               the LC100.

   Parameters:

   ViSession instrHandle:     The actual session to opened device.
   ViChar userText:           The user text. The buffer must contain at least
                              65 elements. 64 characters plus a trailing zero ('\0').
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_getUserText (ViSession instrHandle, ViChar userText[]);


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
  ViInt16 bRequest            :  the command sent to the LC100
  ViUInt16 wValue             :  arbitrary parameter, can be used for additional information
  ViUInt16 wIndex             :  arbitrary parameter, can be used for additional information
  ViUInt16 wLength            :  size of Buffer
  ViBuf Buffer                :  buffer of max. 64 Bytes
  ViPUInt16 Read_Bytes        :  number of bytes actually read out

  Result                      :  Error
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_usbOut(ViSession Instrument_Handle, ViInt16 bRequest, ViUInt16 wValue, ViUInt16 wIndex, ViUInt16 wLength, ViBuf Buffer);


/*---------------------------------------------------------------------------
  USB In - encapsulates the VISA function 'viUsbControlIn()'. When LC100 stalls
  the error VI_ERROR_IO will be returned by 'viUsbControlIn()'. Then USB Out
  will issue the VISA function 'viUsbControlIn' and tries to read one Byte
  from the LC100, if this succeeds the obtained Byte contains the error code
  from the LC100. This means we have NO communications error.

  Parameters
  ViSession Instrument_Handle :  the handle obtained by 'LC100_init()'
  ViInt16 bRequest            :  the command sent to the LC100
  ViUInt16 wValue             :  arbitrary parameter, can be used for additional information
  ViUInt16 wIndex             :  arbitrary parameter, can be used for additional information
  ViUInt16 wLength            :  size of Buffer
  ViBuf Buffer                :  buffer of max. 64 Bytes

  Result                      :  Error
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_usbIn(ViSession Instrument_Handle, ViInt16 bRequest, ViUInt16 wValue, ViUInt16 wIndex, ViUInt16 wLength, ViBuf Buffer, ViPUInt16 Read_Bytes);


/*---------------------------------------------------------------------------
  USB Write - encapsulates the VISA function 'viWrite'. When LC100
  stalls the error VI_ERROR_IO will be returned by 'viUsbControlOut()'.
  Then USB Out will issue the VISA function 'viUsbControlIn' and tries to
  read one Byte from the LC100, if this succeeds the obtained Byte contains
  the error code from the LC100. This means we have NO communications error.

  Parameters
  ViSession Instrument_Handle :  the handle obtained by 'LC100_init()'
  ViBuf Buffer                :  buffer to send to device
  ViUInt32 Count              :  number of Bytes to send from buffer to device

  Return Value
  ViUInt32 *ReturnCount       :  number of Bytes actually sent to device
                                 You may pass NULL if you do not need the value

  Result                      :  Error
---------------------------------------------------------------------------*/
//ViStatus _VI_FUNC LC100_USB_write(ViSession Instrument_Handle, ViBuf Buffer, ViUInt32 Count, ViUInt32 *ReturnCount);


/*---------------------------------------------------------------------------
  USB Read - encapsulates the VISA function 'viRead'.

  Parameters
  ViSession Instrument_Handle :  the handle obtained by 'LC100_init()'
  unsigned char *ReceiveData  :  pointer to a buffer where received data is put to
  ViUInt32 Count              :  number of Bytes to read from device to buffer

  Return Value
  Result                      :  Error
---------------------------------------------------------------------------*/
ViStatus _VI_FUNC LC100_USB_read(ViSession Instrument_Handle, unsigned char *ReceiveData, ViUInt32 Count);


#ifdef __cplusplus
}
#endif

#endif  /* __LC100_DRV_H__ */
