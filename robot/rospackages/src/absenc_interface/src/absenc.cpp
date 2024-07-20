#include "absenc.h"
#include <iostream>

const char* strAbsencErr(int err) {
    switch(err) {
        case NO_ERROR:
            return "No error occurred";
        case ERR_SERIAL_FAILURE:
            return "Serial failure";
        case ERR_SLAVE_INVALID:
            return "Slave invalid";
        case ERR_NO_RESPONSE:
            return "No response";
        case ERR_FRAME_CORRUPTED:
            return "Frame corrupted";
        default:
            return "Unknown code";
    }
}

ABSENC_Error_t AbsencDriver::OpenPort(const char* fileName, uint16_t baud_rate, int& s_fd){
    errno = 0; 

    s_fd = open(fileName, O_RDWR); 
    if(s_fd < 0) {
        int errno0 = errno; 
        errno = 0; 
        return ABSENC_Error_t{
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
    
    cfsetispeed(&ttycfg,B57600);
    cfsetospeed(&ttycfg,B57600);

    if(tcsetattr(s_fd, TCSANOW, &ttycfg) > 0) {
        int errno0 = errno; 
        errno = 0; 
        return ABSENC_Error_t{
            ERR_SERIAL_FAILURE, 
            errno0, 
            __LINE__, 
        }; 
    }
    return no_error; 
}
ABSENC_Error_t AbsencDriver::PollSlave(int slvnum, ABSENC_Meas_t * meas, int s_fd){
     if(slvnum < 0 || slvnum > 9) {
        return ABSENC_Error_t {
            ERR_SLAVE_INVALID, 
            0, 
            __LINE__, 
        }; 
    }
    tcflush(s_fd, TCIOFLUSH); 

    char txbuf[2]; 
    txbuf[0] = '#'; 
    txbuf[1] = '0' + slvnum; 
    int nsend = write(s_fd, txbuf, sizeof(txbuf)); 
    if(nsend < 0) {
        int errno0 = errno; 
        errno = 0; 
        return ABSENC_Error_t{
            ERR_SERIAL_FAILURE, 
            errno0, 
            __LINE__, 
        }; 
    }
    // tcdrain(s_fd); // Flush TX buffer? seems not needed

    char sof; 
    for(int i = 0; i < 50; i++) { // Ensure SOF search always ends
        int nrecv = read(s_fd, &sof, 1); 
        if(nrecv < 0) {
            
            int errno0 = errno; 
            errno = 0; 
            return ABSENC_Error_t{
                ERR_SERIAL_FAILURE, 
                errno0, 
                __LINE__, 
            }; 
        }
        if(nrecv == 0) {
            return ABSENC_Error_t{
                ERR_NO_RESPONSE, 
                0, 
                __LINE__, 
            }; 
        }
        if(sof == '#') break; 
        // Not SOF, maybe noise on the bus, search for another one
    }
    if(sof != '#') {
        return ABSENC_Error_t{
            ERR_FRAME_CORRUPTED, 
            0, 
            __LINE__, 
        }; 
    }

    char rxbuf[21]; // "X -> AAAA, BBBB, CCCC", ignore the \r\n
    int nrecv = read(s_fd, rxbuf, sizeof(rxbuf)); 
    if(nrecv < 0) {
        int errno0 = errno; 
        errno = 0; 
        return ABSENC_Error_t{
            ERR_SERIAL_FAILURE, 
            errno0, 
            __LINE__, 
        }; 
    }

    // Debug code
    // if (nrecv > 0) {
    //     printf("Received %s\n", rxbuf);
    // } else {
    //     printf("Received nothing\n");
    // }
    
    if(nrecv < (int)sizeof(rxbuf)){
        return ABSENC_Error_t{
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

   /*
    TODO : Explain how this works for the people with a skill issue
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
            else return ABSENC_Error_t{
                ERR_FRAME_CORRUPTED, 
                0, 
                __LINE__, 
            }; 
            val = (val << 4) | nib; 
        }
        index += 2; 
        rawdata[i] = val; 
    }

    meas->slvnum = slvnum; 
    meas->status = rawdata[0]; 
    meas->angval = ((double)(int16_t)rawdata[1]) / 65536.0 * 360.0; // TODO: fix calibration code on MCU side
    meas->angspd = (double)(int16_t)rawdata[2]; // TODO: this conversion formula
    
    return no_error; 
}

ABSENC_Error_t AbsencDriver::ClosePort(int s_fd) {
    close(s_fd); 
    return no_error; 
}
