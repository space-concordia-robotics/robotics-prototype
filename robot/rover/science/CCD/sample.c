//==============================================================================
//
// Title:      sample.c
// Purpose:    Simple pure C program to show how to communicate with
//             Thorlabs LC100 - Smart Line Camera.
//
//             This program will take some scans and store them to a *.txt file
//             in the programs executable directory.
//
// Created on: May-10-2011
// Author:     Lutz Hoerl (lhoerl@thorlabs.com)
// Copyright:  Thorlabs. All Rights Reserved.
//
//==============================================================================

//==============================================================================
// Include files
//===========================================================================
//#include <ansi_c.h>
#include <stdio.h>         // stdio for file operations
#include <time.h>          // time stamps
#include <ni-visa/visa.h>          // we need VISA, typ. found in VXIPNP\include dir.
#include "LC100_Drv.h"     // the device driver header
#include "stdlib.h"
//==============================================================================
// Constants
//===========================================================================

#define MY_INTEGRATION_TIME   0.005          // 5 ms
#define MY_SAMPLE_FILE        "sample.txt"   // the file to store the values to
#define MY_SCAN_COUNT         10             // we take 10 scans

//===========================================================================
// Globals
//===========================================================================

ViSession   instr    = VI_NULL;                 // instrument handle
FILE*       my_file  = NULL;                    // file handling

//===========================================================================
// Prototypes
//===========================================================================

void error_exit(ViStatus err);
void waitKeypress(void);

//==============================================================================
// Main
//==============================================================================
int main (int argc, char *argv[])
{
   ViStatus    err      = VI_SUCCESS;              // error variable
   ViSession   resMgr   = VI_NULL;                 // resource manager
   ViUInt32    i        = 0;                       // a loop variable
   ViUInt32    j        = 0;                       // another loop variable
   ViUInt32    cnt      = 0;                       // counts found devices
   ViUInt32    status   = 0;                       // status variable
   ViUInt16    data[LC100_NUM_PIXELS];             // scan data array
   ViChar      rscStr[VI_FIND_BUFLEN];             // resource string
   ViChar*     rscPtr;                             // pointer to resource string
   time_t      t;                                  // time structure

   // try to open file
   my_file = fopen(MY_SAMPLE_FILE, "w");
   if(my_file == NULL)  return -1;

   printf("Thorlabs LC100 instrument driver sample application\n");

   // Parameter checking / Resource scanning
   if(argc < 2)
   {
      // Find resources
      printf("Scanning for LC100 instruments ...\n");
      if((err = viOpenDefaultRM(&resMgr))) error_exit(err);
      if((err = viFindRsrc(resMgr, LC100_FIND_PATTERN, VI_NULL, &cnt, rscStr))) error_exit(err);
      printf("Found %u instrument%s ...\n\n", cnt, (cnt>1) ? "s" : "");
      rscPtr = rscStr;
      viClose(resMgr);
   }
   else
   {
      // Got resource in command line
      rscPtr = argv[1];
   }

   // try to open LC100
   printf("Opening session to '%s' ...\n\n", rscStr);
   err = LC100_init(rscStr, VI_FALSE,VI_FALSE, &instr);
   // error handling
   if(err)  error_exit(err);

   // set integration time
   err = LC100_setIntegrationTime(instr, MY_INTEGRATION_TIME);
   // error handling
   if(err)  error_exit(err);

   // set operating mode to software triggered single scan
   err = LC100_setOperatingMode (instr, OPMODE_SW_SINGLE_SHOT);
   // error handling
   if(err)  error_exit(err);

   while( i < MY_SCAN_COUNT )
   {
      // request device status
      err = LC100_getDeviceStatus(instr, &status);
      // error handling
      if(err)  error_exit(err);

      // camera is idle -> we can trigger a scan
      if(status == LC100_STATUS_IDLE)
      {
         // trigger scan
         err = LC100_startScan(instr);
         // error handling
         if(err)  error_exit(err);
      }

      // camera has data available for transfer
      if(status == LC100_STATUS_TRANSFER)
      {
         printf("Starting scan %d of %d ...\n\n", i+1, MY_SCAN_COUNT);

         // trigger scan
         err = LC100_getScanData(instr, data);
         // error handling
         if(err)  error_exit(err);

         // add seperator
         fprintf(my_file, "----------------- Scan No. %d -----------------\n", i+1);

         // get time stamp
         t = time(&t);

         // store time stamp to file
         fprintf(my_file, "%s\n", ctime(&t));

         // store data to file
         for(j = 0; j < LC100_NUM_PIXELS; j++)
         {
            fprintf(my_file, "Pixel: %4d - Value: %5d\n", j + 1, data[j]);
         }

         // one scan is done
         i++;
      }
   }

   // number of scans done

   // close camera
   LC100_close(instr);
   // close output file
   fclose(my_file);

   waitKeypress();

   // leave main
   return err;
}


/*---------------------------------------------------------------------------
  Error exit
---------------------------------------------------------------------------*/
void error_exit(ViStatus err)
{
   ViChar ebuf[LC100_ERR_DESCR_BUFFER_SIZE];

   // Print error
   LC100_errorMessage (instr, err, ebuf);
   fprintf(stderr, "ERROR: %s\n", ebuf);

   // Close instrument handle if open
   if(instr != VI_NULL) LC100_close(instr);

   // Close file stream if open
   if(my_file != NULL)  fclose(my_file);

   // Exit program
   waitKeypress();

    exit(err);
}


/*---------------------------------------------------------------------------
  Print keypress message and wait
---------------------------------------------------------------------------*/
void waitKeypress(void)
{
   printf("Press <ENTER> to exit\n");
   while(getchar() == EOF);
}
