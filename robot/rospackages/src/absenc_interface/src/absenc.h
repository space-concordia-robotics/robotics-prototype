#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>


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

#define no_error (ABSENC_Error_t{0, 0, __LINE__})

extern void ABSENC_ReportError(ABSENC_Error_t err); 
extern void ABSENC_PrintMeas(const ABSENC_Meas_t * meas); 

class ABSENC{
    inline static int s_fd;
public:
    static ABSENC_Error_t OpenPort(const char* path, uint16_t baud_rate);
    static ABSENC_Error_t PollSlave(int slvnum, ABSENC_Meas_t * meas);
    static ABSENC_Error_t ClosePort();
};