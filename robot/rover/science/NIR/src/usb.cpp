/*
 * usb.cpp
 *
 * This module has the wrapper functions to access USB driver functions.
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 *
*/
#include <iostream>

using namespace std;
#ifdef Q_OS_WIN32
#include <setupapi.h>
#endif
#include "usb.h"
#include "hidapi.h"
#ifndef NO_TIMER_SUPPORT
//#include <QTime>
#endif
/***************************************************
*                  GLOBAL VARIABLES
****************************************************/
static hid_device *DeviceHandle;	//Handle to write

static bool USBConnected = false;      //Boolean true when device is connected

static int defaultTimeOut = 20000;

unsigned int lastUSBTranstime;

int readTimeOut; //reset timeout in milliseconds

bool g_StartupCompleted;

bool USB_IsConnected()
{
    return USBConnected;
}

int USB_Init(void)
{
    readTimeOut = defaultTimeOut;
    USB_setLastTranstime();
    return hid_init();
}

int USB_Exit(void)
{
    return hid_exit();
}

int USB_Open()
{   struct hid_device_info *devs, *cur_dev;
    	devs = hid_enumerate(0x0, 0x0);
    	cur_dev = devs;
    	while (cur_dev) {
    		printf("Device Found\n  type: %04hx %04hx\n  path: %s\n  serial_number: %ls", cur_dev->vendor_id, cur_dev->product_id, cur_dev->path, cur_dev->serial_number);
    		printf("\n");
    		printf("  Manufacturer: %ls\n", cur_dev->manufacturer_string);
    		printf("  Product:      %ls\n", cur_dev->product_string);
    		printf("  Release:      %hx\n", cur_dev->release_number);
    		printf("  Interface:    %d\n",  cur_dev->interface_number);
    		printf("\n");
    		cur_dev = cur_dev->next;
    	}
    	hid_free_enumeration(devs);

    DeviceHandle = hid_open(MY_VID, MY_PID, NULL);

    if(DeviceHandle == NULL)
    {
        cout << "Handle is null" << endl;
        USBConnected = false;
        return -1;
    }
    USBConnected = true;
    return 0;
}

int USB_Write(void *pBuffer)
{
    if(DeviceHandle == NULL)
        return -1;

    USB_setLastTranstime();
    return hid_write(DeviceHandle, (const unsigned char *)pBuffer, USB_MIN_PACKET_SIZE+1);

}

int USB_Read(void *pBuffer)
{
    if(DeviceHandle == NULL)
        return -1;

    USB_setLastTranstime();
    return hid_read_timeout(DeviceHandle, (unsigned char *)pBuffer, USB_MIN_PACKET_SIZE, readTimeOut);
}

int USB_Close()
{
    hid_close(DeviceHandle);
    USBConnected = false;
    g_StartupCompleted = false;

    return 0;
}

void USB_setLastTranstime()
{

#ifndef NO_TIMER_SUPPORT
    //QDateTime current = QDateTime::currentDateTime();
    //lastUSBTranstime = current.toTime_t();
#endif
}

unsigned int USB_getLastTranstime()
{
    return lastUSBTranstime;
}
void USB_setDefaultTimeOut()
{

    readTimeOut = defaultTimeOut;
}

void USB_setTimeout(int time_ms)
{

    readTimeOut = time_ms;
}

