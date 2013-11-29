#ifndef XUNIT_TAB_LIB_H
#define XUNIT_TAB_LIB_H

#include <stdbool.h>


/* VID and PID of SEE3CAM Products */

#define SEE3CAM_USB_VID		0x2560
#define SEE3CAM_10CUG_M_PID	0xC110
#define SEE3CAM_10CUG_C_PID	0xc111
#define SEE3CAM_80USB_PID	0xc080

/* GPIO pins of SEE3CAM_80USB Products */

#define GPIO_PIN_OUT1	0x14
#define GPIO_PIN_OUT2	0x18
#define GPIO_PIN_IN1	0x13
#define GPIO_PIN_IN2	0x21

typedef bool BOOL;

BOOL g_flash_flag;

typedef unsigned char UINT8;

typedef unsigned short int UINT16;

enum see3cam_device_index
{
	SEE3CAM_10CUG = 1,
	SEE3CAM_80USB,
};


/* Function Declarations */

BOOL InitExtensionUnit();

BOOL UninitExtensionUnit();

BOOL ReadFirmwareVersion (UINT8 *, UINT8 *, UINT16 *, UINT16 *);

BOOL EnableMasterMode();

BOOL EnableTriggerMode();

BOOL SetFocusPosition( UINT16 );

BOOL GetFocusPosition( UINT16 *);

BOOL SetFocusMode( UINT8 , UINT8 *);

BOOL GetFocusStatus();

BOOL GetFocusMode( UINT8 *);

BOOL GetGpioLevel(UINT8 , UINT8 *);

BOOL SetGpioLevel (UINT8 , UINT8 );

BOOL GetFlashMode (UINT8 *);

BOOL SetFlashMode(UINT8);

#endif
