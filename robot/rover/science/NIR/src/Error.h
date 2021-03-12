
#ifndef ERROR_H_
#define ERROR_H_

/*
 * Error.h
 *
 * This module defines the error handling related definitions
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
*/

#include <stdio.h>
#include <errno.h>

#include "Common.h"

#define DEBUG_LEVEL_ERR  1
#define DEBUG_LEVEL_WRN  2
#define DEBUG_LEVEL_MSG  3

#define CFG_DEBUG_LEVEL DEBUG_LEVEL_MSG

#define DEBUG_LEVEL_NONE 0

#define DEBUG_PRN(...)      printf(__VA_ARGS__)

#if CFG_DEBUG_LEVEL >= DEBUG_LEVEL_MSG
#define DEBUG_MSG(...)      printf(__VA_ARGS__)
#else
#define DEBUG_MSG(...)
#endif

#if CFG_DEBUG_LEVEL >= DEBUG_LEVEL_WRN
#define DEBUG_WRN(...)      printf(__VA_ARGS__)
#else
#define DEBUG_WRN(...)
#endif

#if CFG_DEBUG_LEVEL >= DEBUG_LEVEL_ERR
#define DEBUG_ERR(...)      printf(__VA_ARGS__)
#else
#define DEBUG_ERR(...)		(void)0
#endif

#define DEBUG_TRACE()	ERR_LOG_MSG("Trace ..."), getchar()


#define ERR_GET_STR(f, l)		ERR_GET_STR_(f, l)
#define ERR_GET_STR_(f, l)		f ":" #l " >> "
#define ERR_LOC_STR				ERR_GET_STR(__FILE__,__LINE__)

#define ERR_BLOCK_BEGIN
#define ERR_BLOCK_END      goto exit_point_; exit_point_:

#define ERR_THROW_MSG(x, ...)                    \
                           do {                   \
                              ERR_LOG_MSG(__VA_ARGS__);    \
                              x;         \
                              goto exit_point_;  \
                           } while(0)

#define ERR_THROW_S(x)	   do{ (void)(x); goto exit_point_; } while(0)

#define ERR_THROW(x)	   ERR_THROW_MSG(x, #x)

#define ERR_THROW_PREV()   ERR_THROW_MSG((void)0, "")

#define THROW(x)		   do { ERR_LOG_MSG(#x); if(x == ERR_DEVICE_FAIL) \
								DEBUG_ERR("System Error : %s\n", strerror(errno)); \
								return (x); \
							} while(0)
#define THROW_MSG(x,...)   do { ERR_LOG_MSG(__VA_ARGS__);  return (x); } while(0)
#define THROW_S(x)		   return (x)

#define TRY(x)			   do { ErrorCode_t x__ = (x); if(x__ != SUCCESS) { ERR_LOG_MSG(#x); return (x__); } } while(0)
#define ERR_TRY(e)	       do { if((e) != SUCCESS) { ERR_LOG_MSG(#e); goto exit_point_; } } while(0)

#define ERR_BREAK()		   goto exit_point_
#define ERR_BREAK_MSG(m)   do { DEBUG_MSG(m); goto exit_point_; } while(0)

#define ERR_LOG_MSG(...)    DEBUG_ERR(ERR_LOC_STR __VA_ARGS__), DEBUG_ERR("\n")

typedef enum
{
    SUCCESS = 0,
    FAIL,
    ERR_OUT_OF_RESOURCE,
    ERR_INVALID_PARAM,
    ERR_NULL_PTR,
    ERR_NOT_INITIALIZED,
    ERR_DEVICE_FAIL,
    ERR_DEVICE_BUSY,
    ERR_FORMAT_ERROR,
    ERR_TIMEOUT,
    ERR_NOT_SUPPORTED,
    ERR_NOT_FOUND
} ErrorCode_t;


#endif /* ERROR_H_ */