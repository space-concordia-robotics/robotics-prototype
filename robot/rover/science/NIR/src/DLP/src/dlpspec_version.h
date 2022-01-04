/*****************************************************************************
**
**  Copyright (c) 2015-2016 Texas Instruments Incorporated.
**
******************************************************************************
**
**  DLP
**
*****************************************************************************/

#ifndef VERSION_H_
#define VERSION_H_

// Version format: MAJOR.MINOR.BUILD
#define DLPSPEC_VERSION_MAJOR 2
#define DLPSPEC_VERSION_MINOR 0
#define DLPSPEC_VERSION_BUILD 3

// Data format versions
#define DLPSPEC_CALIB_VER 1
#define DLPSPEC_REFCAL_VER 3
#define DLPSPEC_CFG_VER 1

/* ----------------------------------------------------------------------
KNOWN ISSUES:
----------------------------------------------------------------------
The following Library functions are not re-entrant:-
1. dlpspec_scan_genPatterns()
2. dlpspec_scan_had_genPatDef()
3. dlpspec_scan_genPatterns()
4. dlpspec_scan_interpret()
5. dlpspec_scan_had_interpret()
6. dlpspec_scan_section_get_adc_data_range()

----------------------------------------------------------------------
VERSION HISTORY:
----------------------------------------------------------------------

* 2.0.3 - Interpolation function added: dlpspec_interpolate_double_positions()
        - Corrected issue with truncation of pointer arithmetic in 64-bit systems
* 2.0.2 - DLL build script added
* 2.0.0 - Public release since 1.1.5. See below for new features and bug fixes.
        - Scan Data structures modified to handle Slew Scans.
        - Hadamard Scan Pattern count computation fixed.
        - Hadamard Max Patterns calculation modified to report appropriate errors.
        - Added validation to ensure that reference being interpreted fully covers the
        - scan config it is being interpreted to.
        - Added logic to skip interpolation if factory and scan configs match.
* 1.1.5 - Fixed an issue where reference interpretation could fail if the scan config
        - being interpreted to has more datapoints than the data being interpreted from.
* 1.1.4 - Fixed a deserialization error checking issue when loading data on targets which
        - pack the associated struct more tightly than the source system that serialized
        - the data
* 1.1.3 - Column scan generation speed increased for parity with Hadamard pattern generation
        - More robust error checking added
        - Error codes now returned for all applicable functions
* 1.0.0 - Initial Public Release

* 0.9   - Bugfixes in Hadamard in genPatDef, requiring both Tiva and host to match

* 0.8   - Hadamard support added

*/

#endif /* VERSION_H_ */
