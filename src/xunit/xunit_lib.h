#ifndef XUNIT_LIB_H
#define XUNIT_LIB_H

#include "xunit_tab_lib.h"
#include <libudev.h>

#define FALSE 0
#define TRUE 1

/* Report Numbers */
#define APPLICATION_READY 	0x12
#define READFIRMWAREVERSION	0x40
#define ENABLEMASTERMODE	0x50
#define ENABLETRIGGERMODE	0x51

#define CAMERA_CONTROL		0x60
#define GET_FOCUS_MODE		0x01
#define SET_FOCUS_MODE		0x02
#define GET_FOCUS_POSITION	0x03
#define SET_FOCUS_POSITION	0x04
#define GET_FOCUS_STATUS	0x05
#define GET_FLASH_MODE		0x06
#define SET_FLASH_MODE		0x07

#define CONTINOUS_FOCUS		0x01
#define MANUAL_FOCUS		0x02
#define SINGLE_TRIG_FOCUS	0x03

#define FOCUS_FAILED		0x00
#define FOCUS_SUCCEEDED		0x01
#define FOCUS_BUSY		0x02

#define FLASH_OFF		0x00
#define FLASH_TORCH		0x01
#define FLASH_STROBE		0x02

#define SET_FLASH_FAIL		0x00
#define SET_FLASH_SUCCESS	0x01

#define GPIO_OPERATION		0x20
#define GPIO_GET_LEVEL		0x01
#define GPIO_SET_LEVEL		0x02
#define GPIO_INPUT_DETECT	0x03
#define GPIO_LOW		0x00
#define GPIO_HIGH		0x01
#define GPIO_LEVEL_FAIL		0x00
#define GPIO_LEVEL_SUCCESS	0x01

#define BUFFER_LENGTH		65
#define TIMEOUT			2000

#define SUCCESS			1
#define FAILURE			-1

#define DEVICE_NAME_MAX				32		// Example: /dev/hidraw0
#define MANUFACTURER_NAME_MAX			64		// Example: e-con Systems
#define PRODUCT_NAME_MAX			128		// Example: e-con's 1MP Monochrome Camera
#define HID_LIST_MAX				32

extern int hid_fd;

unsigned char g_out_packet_buf[BUFFER_LENGTH];
unsigned char g_in_packet_buf[BUFFER_LENGTH];

const char	*hid_device;

/* Function Declarations */

const char *bus_str(int);

unsigned int GetTickCount();

int find_hid_device(const char* bus_info);

#endif




