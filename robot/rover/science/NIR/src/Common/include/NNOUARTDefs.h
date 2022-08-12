/*
 * UART related definitions
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
 */

#ifndef NNOUARTDEFS_H_
#define NNOUARTDEFS_H_

#include "NNOCommandDefs.h"

#define UART_START_IND_NUM_BYTES 4

#define UART_START_IND_BYTE_0 0x41
#define UART_START_IND_BYTE_1 0x42
#define UART_START_IND_BYTE_2 0x43
#define UART_START_IND_BYTE_3 0x44

#define UART_END_IND_NUM_BYTES 4

#define UART_END_IND_BYTE_0 UART_START_IND_BYTE_3
#define UART_END_IND_BYTE_1 UART_START_IND_BYTE_2
#define UART_END_IND_BYTE_2 UART_START_IND_BYTE_1
#define UART_END_IND_BYTE_3 UART_START_IND_BYTE_0

typedef struct _uartMessageStruct
{
	unsigned char startInd[UART_START_IND_NUM_BYTES];
	unsigned int chkSum;
	nnoMessageStruct msg;
	unsigned char endInd[UART_END_IND_NUM_BYTES];
} uartMessageStruct;

/*
 * Max packet size would be limited by size of message struct
 */
#define UART_MAX_CMD_MAX_PKT_SZ 			sizeof(uartMessageStruct)

/*
 * Error codes
 */
#define UART_INCOMP_START_END_IND_RECD	-1
#define UART_INPUT_PKT_CHECKSUM_ERROR	-2
#define UART_WRITE_FAILED				-3

#endif /* NNOUARTDEFS_H_ */
