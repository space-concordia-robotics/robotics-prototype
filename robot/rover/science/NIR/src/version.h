/*
 * version.h
 *
 * This module has the version number info.
 *
 * Copyright (C) 2015-2016 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
*/

#ifndef VERSION_H
#define VERSION_H

#define GUI_VERSION_MAJOR 2
#define GUI_VERSION_MINOR 1
#define GUI_VERSION_BUILD 0

#define DLPC_VERSION_MAJOR 2
#define DLPC_VERSION_MINOR 0
#define DLPC_VERSION_BUILD 0

#endif //VERSION_H
/*
----------------------------------------------------------------------
KNOWN ISSUES:
----------------------------------------------------------------------
The maximum limit set for digital resolution spin box at the time of creation of
hadamard sections is not accurate.However the max limit verified at the time of saving
the scan configuration is correct.
----------------------------------------------------------------------
VERSION HISTORY:
----------------------------------------------------------------------
* 2.1.0  - Added support for new commands to store and read Model Name from EEPROM
* 2.0.5  - Added checks to limit scan configurations that could cause glitches in output.
* 2.0.4  - Added Keep Lamp ON button during scans on Scan tab.
          - Added entry for number of consecutive back-to-back scans.
          - Added entry for seconds delay between back-to-back scans.
          - Added ability to use automatic or fixed ADC PGA gain values.
          - Some combination of scan configuration parameters can lead to glitches in scan. 
            This version checks for those illegal scan configurations and avoids generating
            the problematic patterns. These tend to occur when the pattern width is small and 
            a large number of wavelength points are used.
          - Added global #define to remove hardcoded wavelength ranges.
          - Improved accuracy of scan time computation
          - Fixed an issue when Bluetooth is enabled at power-up and GUI is connected. 
            When GUI is connected, Bluetooth is now disabled.
          - Reported width on CSV was offset of a list. Now CSV reports actual width
          - Reported scan time on CSV was estimated scan time. Now CSV reports actual scan time.
          - Added UART command support to GUI through FTDI USB cable and expansion connector.
          - Added JCAMP output file format (.jdx) to GUI under the file settings button.       
          - Fixed an issue when there is only one config saved on EVM and no locally saved 
            configs on the PC which caused the GUI to not display the properties of the selected 
            scan config.
 * 2.0.3  - Added calibration verification with NIST SRM-2036 reference
          - Added additional verification of RM-NIR spectrum during calibration
 * 2.0.255 - custom release with extended values for Max supported wavelength, minimum width etc.
           - Minimum column width reduced to 1 mirror i.e 1.17nm.
           - Maximum supported wavelength increased to 1754nm
           - Added a command for fixing the PGA during scan without changing it.
           - Setting a Fixed PGA gain of 1 during the Scan.
 * 2.0.2  - Removed the EEPROM Wipe option and consolidated the version mismatch dialogs.
 * 2.0.1  - GUI display and stability refinements in the modify scan configuration window.
 * 2.0.0  - Public release since 1.2.0. See below for new features and bug fixes.
          - Fixed uninitalized config name in the configs that regression test stores into EEPROM.
          - Changes to regression test file
          - Fixes to display the total number of patterns and max patterns for every edit
          - Corrected maximum pattern size for Hadamard pattern sets in scan config dialog.
          - GUI now allows more than 20 EVM scan configs through copy/move
          - Fixed issue when connected through bluetooth, where the GUI quit button does not work
          - Fixed issue with the Hour glass icon fails to disappear after new reference scan
          - Fixed issue with duplicate scan configs when copying between local and EVM
          - Fixed issue that preveneted editing of the fist scan config in the EVM
          - Fixed issue where a new scan and saved scan directory was not remembered across sessions
          - Fixed issue that prevented the build released of the GUI source code
          - Added capability to set a selected Configuration as active configuration in EVM
          - Added capability to display maximum possible patterns for each section and total patterns across all sections
          - Enabled editing of the first saved scan configuration in the EVM.
          - Added capability to differentiate different slew sections marked with different colors in the plot
          - Fixed issues with remembering the last set Save Scans and Display Scans diretories
          - Added capability to create Slew Scans in the Scan Configuration Dialog.
 * 1.2.0  - Hibernation enable read and write commands added. Control added to Utility tab.
 * 1.1.9  - Included DLP 1.1.5 for bugfix on reference interpretation
 * 1.1.6  - removed pattern source argument from NNO_CMD_DLPC_ENABLE command
 * 1.0.5  - changed the latest develop version to 1.0.5
 * 1.0.4  - Corrected scan config selection error when not in factory mode
 * 0.34.4 - Added LED Test command
 * 0.34.3 - Added Mass EEPROM Erase command
 * 0.34.0 - Serial number setting is mandated before wavelength calibration is allowed.
 * 0.33.0 - EEPROM layout updated. Scan and cfg serial numbering implemented.
 * 0.31.0 - Corrected a duplicate entry in the command table
 * 0.30.0 - Scan command interface cleaned up to not take num_patterns as a parameter.
 * 0.29.0 - Supported added to store scan data in micro SD card and read back.
 * 0.28.0 - Removed the QCutomPlot widget, added Splash Image and added Information tab
 * 0.28.0 - Support added for sequences with black vector after every 24 patterns. To be used with Spectrum Library v0.4
 * 0.27.1 - Updated the reference sample abosorption data for comparison during system test.
 * 0.27.0 - support for user inputs in wavelength units. DLP spectrum library v0.3
 * 0.26.0 - genPatterns and scan_interpret that takes num_patterns as user input.
 * 0.25.0 - Works with v0.2 of spectrum library that has the inital implementation of scan_interpret function
 * 0.14.4 - For wavelength calibration, adjusted DMD top, mid and bottom scan pattern size.
 * 0.14.3 - Fixed the incorrect error pop up that was causing wavelength calibration to fail.
 * 0.14.0 - DLPCEnable without turning ON the lamp supported now for slit and wavelength alignemnt process.
 * 0.13.0 - Wavelength calibration implemented.
 * 0.12.0 - Scans hang in version 0.11.1 due to ADC being not initialized. It is fixed in 0.12.0.
 * 0.11.1 - Option to save only the final averages while average multiple scans or save all individual scans.
 * 0.10.0 - PGA gain setting now working. Also added a Get PGA gain command.
 * 0.9.0 - Tiva will now averaging multiple scans as per num_repeat field in scanConfig. But averaging more than
 *         8 scans can lead to overflows as float not yet implemented in Tiva averaging.
 * 0.8.0 - Calibration scans with patterns generated by Tia on-the-fly and streaming via RGB interface to DLPC150 implemented.
 * 0.7.0 - Tiva now retuns serialized ColumnScanDataStruct when scan result file is read back.
 * 0.6.0 - Import and Export (to device) of user defined scan configs implemented.
 * 0.5.0 - Saving scan data in CSV format
 * 0.4.6 - Added TIVA FW update feature.
 * 0.4.5 - Added user defined scan feature.
 * 0.4.4 - Modified slit alignment feedback sliders to take into account signal strength also in addition to peak width
 * 0.4.1 - GUI crash issues during slit alignment resolved.
 * 0.4.0 - Added some safety checks in dlpspec_calib_findPeaks()function; also changed USB PID to 0x4200.
 * 0.3.1 - Fixed the issue with slit aliggnment calibration rightmost slider not working properly.
 *         SPI clock used to program/read back DLPC150 firmware from flash reduced to 2.5Mhz to avoid occasional checksum mismatches.
 * 0.3.0 - Scan with patterns streaming over RGB interface now working. Also supported scans with > 255 number of patterns.
 * 0.2.4 - Removed the error message pop up during slit alignment calibration when expected number of peaks were not found in scan data.
 *         Added more details to BQ test and added a Bluetooth test.
 * 0.2.3 - Added the option to save test results in a file with filename containing the PCB serial number and date stamp
 * 0.2.2 - Added TMP and HDC tests, added test result log file, fixed the issue of lamp not turning ON on repeated scans
 * 0.2.0 - Version with detector and slit alignment scans working with patterns in flash + a subset of PCB test functions
 * 0.1.0 - Initial version with detector alignement scan working with patterns stored in flash
 *
 */
