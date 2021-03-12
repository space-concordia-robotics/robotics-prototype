/*
 * SnrData.h
 *
 * contains declarations for SNR scan-related functionality
 *
 * Copyright (C) 2014-2015 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
*/

#ifndef SNRDATA_H
#define SNRDATA_H

// SNR supporting items
#define SNR_PATTERNS 9
#define SNR_HAD_PATTERNS 120
#define SNR_HAD_DEFINED 0

typedef struct
{
    float snr_17ms[SNR_PATTERNS];
    float snr_100ms[SNR_PATTERNS];
    float snr_500ms[SNR_PATTERNS];

}snrData;

typedef struct
{
    float had_120ms;
    float had_1s;
}snrDataHad;


// Hadamard settings
#define HADSNR_LENGTH 140
#define HADSNR_NUM_PATTERNS 97 //should be few enough that hadamard matrices won't overflow length
#define PATTERNS_PER_FRAME 25
#define HADSNR_FRAMES ((HADSNR_LENGTH + PATTERNS_PER_FRAME - 1) / PATTERNS_PER_FRAME)
#define HADSNR_TIME 1000 //in ms
#define HADSNR_FRAMES_PER_S 60
#define HADSNR_BIN_SIZE ((HADSNR_FRAMES_PER_S / HADSNR_FRAMES) * (HADSNR_TIME / 1000))
#define HADSNR_NUM_DATA 36
#define HADSNR_NUM_REPEATS (HADSNR_NUM_DATA * HADSNR_BIN_SIZE)


#endif // SNRDATA_H

