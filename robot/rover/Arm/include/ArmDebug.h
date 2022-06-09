#ifndef ARM_DEBUG_H
#define ARM_DEBUG_H

#define DEBUG

#ifndef DEBUG
#define Serial Serial1
#endif

inline void float2Bytes(byte float_bytes[4],float float_var){
    memcpy(float_bytes, (unsigned char*) (&float_var), 4);
}


#endif