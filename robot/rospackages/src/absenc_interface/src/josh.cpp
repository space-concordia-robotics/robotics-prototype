#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

// Start of .h file
#define ERR_SERIAL_FAILURE  1
#define ERR_SLAVE_INVALID   2
#define ERR_NO_RESPONSE     3
#define ERR_FRAME_CORRUPTED 4
typedef struct {
    int error; 
    int cause; 
    int line; 
} ABSENC_Error_t; 

typedef struct {
    uint8_t slvnum; 
    uint16_t status; 
    double angval; 
    double angspd; 
} ABSENC_Meas_t; 

extern ABSENC_Error_t ABSENC_OpenPort(const char * dev, int * fd); 
extern ABSENC_Error_t ABSENC_PollSlave(int fd, int slvnum, ABSENC_Meas_t * meas); 
extern ABSENC_Error_t ABSENC_ClosePort(int fd); 

extern void ABSENC_ReportError(ABSENC_Error_t err); 
extern void ABSENC_PrintMeas(const ABSENC_Meas_t * meas); 
// End of .h file

#define BAUD_RATE B57600
#define no_error ((ABSENC_Error_t){0, 0, __LINE__})

// void ABSENC_ReportError(ABSENC_Error_t obj) {
//     if(!obj.error) return; 
//     if(!obj.cause) {
//         printf("Error %d at line %d.\r\n", obj.error, obj.line); 
//     }
//     else {
//         printf("Error %d at line %d. Cause: %s (%d)\r\n", obj.error, obj.line, strerror(obj.cause), obj.cause); 
//     }
// }

void ABSENC_ReportError(char* statusBuf,ABSENC_Error_t obj) {
    if(!obj.error) return;
    if(!obj.cause) {
        sprintf(statusBuf,"Error %d at line %d.\r\n", obj.error, obj.line); 
    }
    else {
        sprintf(statusBuf,"Error %d at line %d. Cause: %s (%d)\r\n", obj.error, obj.line, strerror(obj.cause), obj.cause); 
    }
}

ABSENC_Error_t ABSENC_OpenPort(const char * fileName, int * fd0) {
    errno = 0; 

    int fd = open(fileName, O_RDWR); 
    if(fd < 0) {
        int errno0 = errno; 
        errno = 0; 
        return (ABSENC_Error_t) {
            ERR_SERIAL_FAILURE, 
            errno0, 
            __LINE__, 
        }; 
    }

    struct termios ttycfg; 
    ttycfg.c_cflag = CS8 | CREAD | CLOCAL; // 8N1, ignore modem signals
    ttycfg.c_lflag = 0; 
    ttycfg.c_iflag = 0; 
    ttycfg.c_oflag = 0; 
    ttycfg.c_line = 0; 
    ttycfg.c_cc[VTIME] = 1; // 100ms timeout
    ttycfg.c_cc[VMIN] = 0; // Return anything read so far
    // Why don't the lines here worK ? 
    // ttycfg.c_ispeed = BAUD_RATE; 
    // ttycfg.c_ospeed = BAUD_RATE; 
    cfsetispeed(&ttycfg,B57600);
    cfsetospeed(&ttycfg,B57600);

    if(tcsetattr(fd, TCSANOW, &ttycfg) > 0) {
        int errno0 = errno; 
        errno = 0; 
        return (ABSENC_Error_t) {
            ERR_SERIAL_FAILURE, 
            errno0, 
            __LINE__, 
        }; 
    }

    *fd0 = fd; 
    return no_error; 
}

ABSENC_Error_t ABSENC_PollSlave(int fd, int slvnum, ABSENC_Meas_t * meas0) {
    if(slvnum < 0 || slvnum > 9) {
        return (ABSENC_Error_t) {
            ERR_SLAVE_INVALID, 
            0, 
            __LINE__, 
        }; 
    }
    tcflush(fd, TCIOFLUSH); 

    char txbuf[2]; 
    txbuf[0] = '#'; 
    txbuf[1] = '0' + slvnum; 
    int nsend = write(fd, txbuf, sizeof(txbuf)); 
    if(nsend < 0) {
        int errno0 = errno; 
        errno = 0; 
        return (ABSENC_Error_t) {
            ERR_SERIAL_FAILURE, 
            errno0, 
            __LINE__, 
        }; 
    }
    tcdrain(fd); // Flush TX buffer? seems not needed

    char sof; 
    for(int i = 0; i < 50; i++) { // Ensure SOF search always ends
        int nrecv = read(fd, &sof, 1); 
        if(nrecv < 0) {
            int errno0 = errno; 
            errno = 0; 
            return (ABSENC_Error_t) {
                ERR_SERIAL_FAILURE, 
                errno0, 
                __LINE__, 
            }; 
        }
        if(nrecv == 0) {
            return (ABSENC_Error_t) {
                ERR_NO_RESPONSE, 
                0, 
                __LINE__, 
            }; 
        }
        if(sof == '#') break; 
        // Not SOF, maybe noise on the bus, search for another one
    }
    if(sof != '#') {
        return (ABSENC_Error_t) {
            ERR_FRAME_CORRUPTED, 
            0, 
            __LINE__, 
        }; 
    }

    char rxbuf[21]; // "X -> AAAA, BBBB, CCCC", ignore the \r\n
    int nrecv = read(fd, rxbuf, sizeof(rxbuf)); 
    if(nrecv < 0) {
        int errno0 = errno; 
        errno = 0; 
        return (ABSENC_Error_t) {
            ERR_SERIAL_FAILURE, 
            errno0, 
            __LINE__, 
        }; 
    }
    if(nrecv < sizeof(rxbuf)) {
        return (ABSENC_Error_t) {
            ERR_FRAME_CORRUPTED, 
            0, 
            __LINE__, 
        }; 
    }

    // Debug code 
    /*
    for(int i = 0; i < nrecv; i++) {
        putchar(rxbuf[i]); 
    }
    puts(""); 
    */

    uint16_t rawdata[3]; 
    int index = 5; 
    for(int i = 0; i < 3; i++) {
        uint16_t val = 0; 
        for(int j = 0; j < 4; j++) {
            uint8_t nib = rxbuf[index++]; 
            if(nib >= '0' && nib <= '9') nib = nib - '0'; 
            else if(nib >= 'A' && nib <= 'F') nib = nib - 'A' + 10; 
            else if(nib >= 'a' && nib <= 'f') nib = nib - 'a' + 10; 
            else return (ABSENC_Error_t) {
                ERR_FRAME_CORRUPTED, 
                0, 
                __LINE__, 
            }; 
            val = (val << 4) | nib; 
        }
        index += 2; 
        rawdata[i] = val; 
    }

    meas0->slvnum = slvnum; 
    meas0->status = rawdata[0]; 
    meas0->angval = ((double)(int16_t)rawdata[1]) / 65536.0 * 360.0; // TODO: fix calibration code on MCU side
    meas0->angspd = (double)(int16_t)rawdata[2]; // TODO: this conversion formula
    
    return no_error; 
}

ABSENC_Error_t ABSENC_ClosePort(int fd) {
    close(fd); 
    return no_error; 
}

void ABSENC_PrintMeas(const ABSENC_Meas_t * meas) {
    printf("#%d: Status: %04X, angle: %f, speed: %f\r\n", meas->slvnum, meas->status, meas->angval, meas->angspd); 
}

int main(void) {
    puts("Program start"); 
    char statusBuffer[1000];
    int fd; 
    ABSENC_Error_t status = ABSENC_OpenPort("/dev/ttyUSB0", &fd); 
    if(status.error !=0 ){
        ABSENC_ReportError(statusBuffer,status);
        puts(statusBuffer);
        return -1;
    }
    puts("Opened serial port"); 

    for(int i = 0; i < 10; i++) {
        usleep(10 * 1000); // 10ms
        for(int slvnum = 1; slvnum <= 3; slvnum++) {
            ABSENC_Meas_t meas; 
            status = ABSENC_PollSlave(fd, slvnum, &meas);
            if(status.error != 0){ 
                ABSENC_ReportError(statusBuffer,status); 
                puts(statusBuffer);
                goto end;
            }
            ABSENC_PrintMeas(&meas); 
        }
        puts(""); 
    }

    end:
    ABSENC_ClosePort(fd); 
    puts("Program terminated"); 
    return 0; 
}


