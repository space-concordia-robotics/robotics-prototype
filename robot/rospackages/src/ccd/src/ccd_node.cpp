#include "ros/ros.h"
#include <stdio.h>         // stdio for file operations
#include <time.h>          // time stamps
#include <ni-visa/visa.h>          // we need VISA, typ. found in VXIPNP\include dir.
#include "LC100_Drv.h"     // the device driver header
#include "stdlib.h"

#define MY_INTEGRATION_TIME   0.005          // 5 ms
#define MY_SAMPLE_FILE        "sample.txt"   // the file to store the values to
#define MY_SCAN_COUNT         10             // we take 10 scans

ViSession   instr    = VI_NULL;                 // instrument handle
FILE*       my_file  = NULL;                    // file handling


void error_exit(ViStatus err);
void waitKeypress(void);

class CCD{
    ViStatus    err      = VI_SUCCESS;              // error variable
    ViSession   resMgr   = VI_NULL;                 // resource manager
    ViUInt32    cnt      = 0;                       // counts found devices
    ViUInt32    status   = 0;                       // status variable
    ViUInt16    data[LC100_NUM_PIXELS];             // scan data array
    ViChar      rscStr[VI_FIND_BUFLEN];             // resource string
    ViChar*     rscPtr;                             // pointer to resource string
    time_t      t;                                  // time structure

    void open(){
        my_file = fopen(MY_SAMPLE_FILE, "w");
        if(my_file == NULL)  return -1;
    }
    void openCCD(){

        // Find resources
        printf("Scanning for LC100 instruments ...\n");
        if((err = viOpenDefaultRM(&resMgr))) error_exit(err);
        if((err = viFindRsrc(resMgr, LC100_FIND_PATTERN, VI_NULL, &cnt, rscStr))) error_exit(err);
        printf("Found %u instrument%s ...\n\n", cnt, (cnt>1) ? "s" : "");
        rscPtr = rscStr;
        viClose(resMgr);

        printf("Opening session to '%s' ...\n\n", rscStr);
        err = LC100_init(rscStr, VI_FALSE,VI_FALSE, &instr);
        // error handling
        if(err)  error_exit(err);

    }
    void configCCD(){

        // set integration time
        err = LC100_setIntegrationTime(instr, MY_INTEGRATION_TIME);
        // error handling
        if(err)  error_exit(err);

        // set operating mode to software triggered single scan
        err = LC100_setOperatingMode (instr, OPMODE_SW_SINGLE_SHOT);
        if(err)  error_exit(err);

    }

    void scan(){
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
    void close(){
        // close camera
        LC100_close(instr);
        // close output file
        fclose(my_file);

        waitKeypress();

    }
};

int main(int argc, char** argv){
    ros::init(argc,argv,"ccd");


}