/**
 *
 * This module provides C callable APIs for communication with the PCSerial port.
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
*/


#ifdef _WIN32
#undef UNICODE

#include <windows.h>
#include <tchar.h>
#include <stdio.h>

static HANDLE hSerial;

int Serial_Open(char const *Port)
{
	char PortName[20];
	char const * const Prefix = "\\\\.\\";
	DCB dcbSerialParams = {0};
	COMMTIMEOUTS timeouts={0};

	if(strlen(Port) + strlen(Prefix) + 1 > sizeof(PortName))
	{
		printf("Invalid Port Name : %s\n", PortName);
		return -1;
	}

	strcpy(PortName, Prefix);
	strcat(PortName, Port);

    hSerial = CreateFile(PortName,
			GENERIC_READ | GENERIC_WRITE,
			0,
			0,
			OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
			0);

	if(hSerial==INVALID_HANDLE_VALUE)
	{
		if(GetLastError() == ERROR_FILE_NOT_FOUND)
		{
			printf("Port %s Not Found\n", Port);
		}
		else
		{
			printf("Some error while opening port %s\n", Port);
		}
		return -1;
	}

	dcbSerialParams.DCBlength=sizeof(dcbSerialParams);

	if (!GetCommState(hSerial, &dcbSerialParams)) 
	{
		printf("Error while GetCommState()\n");
		return -1;
	}

	dcbSerialParams.BaudRate=CBR_115200;
	dcbSerialParams.ByteSize=8;
	dcbSerialParams.StopBits=ONESTOPBIT;
	dcbSerialParams.Parity=NOPARITY;

	if(!SetCommState(hSerial, &dcbSerialParams))
	{
		printf("Error while SetCommState()\n");
		return -1;
	}

    timeouts.ReadIntervalTimeout=50;
    timeouts.ReadTotalTimeoutConstant=50;
	timeouts.ReadTotalTimeoutMultiplier=10;
	timeouts.WriteTotalTimeoutConstant=50;
	timeouts.WriteTotalTimeoutMultiplier=10;

	if(!SetCommTimeouts(hSerial, &timeouts))
	{
		printf("Error while SetCommTimeouts()\n");
		return -1;
	}

	return 0;
}

void Serial_Close(void)
{
	CloseHandle(hSerial);
}

void Serial_SetTimeout(int timeout)
{
    COMMTIMEOUTS timeouts={0};

    timeouts.ReadIntervalTimeout=timeout;
    timeouts.ReadTotalTimeoutConstant=timeout;
    timeouts.ReadTotalTimeoutMultiplier=10;
    timeouts.WriteTotalTimeoutConstant=50;
    timeouts.WriteTotalTimeoutMultiplier=10;

    if(!SetCommTimeouts(hSerial, &timeouts))
    {
        printf("Error while SetCommTimeouts()\n");
        return;
    }
}

int Serial_Write(unsigned char *Buffer, unsigned long Size)
{
	DWORD Sent;
	while(Size)
	{
		if(!WriteFile(hSerial, Buffer, Size, &Sent, NULL))
		{
            printf("Error while writing to serial port\n");
			return -1;
		}

		Buffer += Sent;
		Size -= Sent;
	}
    return 1;
}

int Serial_Read(unsigned char *Buffer, unsigned long Size)
{
	DWORD Read;

	while(Size)
	{
		if(!ReadFile(hSerial, Buffer, Size, &Read, NULL))
		{
            printf("Error while writing to serial port\n");
			return -1;
		}
      /*  if(Read == 0)
        {
            return -1; //to avoid infinite loop
        }*/

        Buffer += Read;
		Size -= Read;
	}
    return 1;
}

#else

#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#include "Error.h"
#include "Common.h"

static int UART_Device = -1;

/************************ FUNCTION DEFINITIONS*******************************/
/*
 * return SUCCESS, FAIL
 */
int Serial_Open(char const *DevName)
{
    struct termios Setting;

	UART_Device = open(DevName, O_RDWR | O_NOCTTY | O_NDELAY);

	if(UART_Device < 0)
		THROW(FAIL);

	/*tcgetattr(UART_Device, &oldtio); */ /* save current port settings */

	bzero(&Setting, sizeof(Setting));

	Setting.c_cflag = B115200 | CS8 | CLOCAL | CREAD;

	Setting.c_iflag = 0;
	Setting.c_oflag = 0;

	/* set input mode (non-canonical, no echo,...) */
	Setting.c_lflag = 0;
	Setting.c_cc[VTIME] = (uint8)(10000 / 100);   /* inter-character timer unused */
	Setting.c_cc[VMIN] = 0;   					/* blocking read until 1 chars received */

	tcflush(UART_Device, TCIFLUSH);

	tcsetattr(UART_Device, TCSANOW, &Setting);

	cfsetospeed(&Setting, B115200);
	cfsetispeed(&Setting, B115200);

    return SUCCESS;
}


int Serial_Write(void *Data, unsigned long Length)
{
	if(UART_Device < 0)
		THROW(ERR_NOT_INITIALIZED);

	if(write(UART_Device, Data, Length) < 0)
		THROW(FAIL);

    return SUCCESS;
}

int Serial_WaitForData(uint32 Timeout)
{
	fd_set InputFD;
	struct timeval TimeoutTimer;
	struct timeval *Timer;

	FD_ZERO(&InputFD);
	FD_SET(UART_Device, &InputFD);

	if(Timeout)
	{
		TimeoutTimer.tv_usec = Timeout;
		TimeoutTimer.tv_sec = 0;
		Timer = &TimeoutTimer;
	}
	else
	{
		Timer = NULL;
	}

	if(select(UART_Device + 1, &InputFD, NULL, NULL, Timer) == 0)
		return ERR_TIMEOUT;

    return SUCCESS;
}

int Serial_TimeoutReadData(void *Data, uint32 Length, uint32 InitTimeout, uint32 Timeout)
{
	int Bytes;

	if(UART_Device < 0)
		THROW(ERR_NOT_INITIALIZED);

	while(1)
	{
		if(ioctl(UART_Device, FIONREAD, &Bytes) < 0)
			THROW(FAIL);

		if(Bytes)
		{
			if(Bytes > Length)
				Bytes = Length;

			if(read(UART_Device, Data, Bytes) < 0)
				THROW(FAIL);

			Length -= Bytes;
			Data = (uint8*)Data + Bytes;

			if(Length == 0)
				break;

			InitTimeout = Timeout;
		}

		if(Serial_WaitForData(InitTimeout))
			THROW_S(ERR_TIMEOUT);

	}

    return SUCCESS;
}

int Serial_Read(void *Data, uint32 Length)
{
	return Serial_TimeoutReadData(Data, Length, 0, 0);
}


int UART_ClearRxFIFO(void)
{
	return tcflush(UART_Device, TCIFLUSH) < 0 ? FAIL : SUCCESS;
}

int UART_ClearTxFIFO(void)
{
	return tcflush(UART_Device, TCOFLUSH) < 0 ? FAIL : SUCCESS;
}

void Serial_Close(void)
{
	 if(UART_Device > 0)
	 {
		 close(UART_Device);
		 UART_Device = -1;
	 }
}
#endif
