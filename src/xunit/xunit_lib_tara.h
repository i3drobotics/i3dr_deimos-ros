///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2016, e-Con Systems.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS.
// IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
// ANY DIRECT/INDIRECT DAMAGES HOWEVER CAUSED AND ON ANY THEORY OF 
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////
/**********************************************************************
	xunit_lib_tara.cpp : Declares the extension unit functions
						 in the shared library.
	Declarations for all HID commands and MACROS.
**********************************************************************/
#ifndef XUNIT_LIB_H
#define XUNIT_LIB_H

#include <stdbool.h>
#include <libudev.h>
#include <pthread.h>

#define VID					"2560"
#define See3CAM_STEREO		"c114"

typedef bool 	  			 BOOL;
typedef int8_t    			 INT8;
typedef int16_t  			 INT16;
typedef int32_t   			 INT32;
typedef unsigned char		 UINT8;
typedef unsigned short int   UINT16;
typedef unsigned int		 UINT32;

/* Stereo IMU */
typedef struct {
	INT8 IMU_MODE;
    INT8 ACC_AXIS_CONFIG;
    INT8 ACC_SENSITIVITY_CONFIG;
	INT8 GYRO_AXIS_CONFIG;
    INT8 GYRO_SENSITIVITY_CONFIG;
	INT8 IMU_ODR_CONFIG;
} IMUCONFIG_TypeDef;

typedef struct {
    INT8 IMU_UPDATE_MODE;
	UINT16 IMU_NUM_OF_VALUES;
} IMUDATAINPUT_TypeDef;

typedef struct {
	UINT16 IMU_VALUE_ID;
    double accX;
    double accY;
    double accZ;
	double gyroX;
	double gyroY;
	double gyroZ;
} IMUDATAOUTPUT_TypeDef;

/* Report Numbers */
#define SET_FAIL		0x00
#define SET_SUCCESS		0x01
#define GET_FAIL		0x00
#define GET_SUCCESS		0x01
#define SUCCESS			 1
#define FAILURE			-1

#define BUFFER_LENGTH					65
#define TIMEOUT							2000
#define CALIB_TIMEOUT					5000
#define DESCRIPTOR_SIZE_ENDPOINT		29
#define DESCRIPTOR_SIZE_IMU_ENDPOINT	23

/* HID STATUS */
#define SEE3CAM_STEREO_HID_SUCCESS			(0x01)
#define SEE3CAM_STEREO_HID_FAIL				(0x00)

/* EXPOSURE CONTROL */
#define SEE3CAM_STEREO_EXPOSURE_AUTO		(1)
#define SEE3CAM_STEREO_EXPOSURE_MIN			(10)
#define SEE3CAM_STEREO_EXPOSURE_MAX			(1000000)
#define SEE3CAM_STEREO_EXPOSURE_DEF			(8000)

/* IMU MODE */
#define IMU_ACC_GYRO_DISABLE				(0x00)
#define IMU_ACC_ENABLE						(0x01)
#define IMU_ACC_GYRO_ENABLE					(0x03)

/* ACC AXIS CONTROL */
#define IMU_ACC_X_Y_Z_ENABLE				(0x07)
#define IMU_ACC_X_ENABLE					(0x01)
#define IMU_ACC_Y_ENABLE					(0x02)
#define IMU_ACC_Z_ENABLE					(0x04)

/* ACC ODR CONTROL */
#define IMU_ODR_10_14_9HZ					(0x01)
#define IMU_ODR_50_59_9HZ					(0x02)
#define IMU_ODR_119HZ						(0x03)
#define IMU_ODR_238HZ						(0x04)
#define IMU_ODR_476HZ						(0x05)
#define IMU_ODR_952HZ						(0x06)

/* ACC SENSITIVITY CONTROL */
#define IMU_ACC_SENS_2G						(0x00)
#define IMU_ACC_SENS_4G						(0x02)
#define IMU_ACC_SENS_8G						(0x03)
#define IMU_ACC_SENS_16G					(0x01)

/* GYRO AXIS CONTROL */
#define IMU_GYRO_X_Y_Z_ENABLE				(0x07)
#define IMU_GYRO_X_ENABLE					(0x01)
#define IMU_GYRO_Y_ENABLE					(0x02)
#define IMU_GYRO_Z_ENABLE					(0x04)

/* GYRO SENSITIVITY CONTROL */
#define IMU_GYRO_SENS_245DPS				(0x00)
#define IMU_GYRO_SENS_500DPS				(0x01)
#define IMU_GYRO_SENS_2000DPS				(0x03)

/* IMU VALUE UPDATE MODE */
#define IMU_CONT_UPDT_EN					(0x01)
#define IMU_CONT_UPDT_DIS					(0x02)

/* IMU VALUES CONTROL */
#define IMU_AXES_VALUES_MIN					(1)
#define IMU_AXES_VALUES_MAX					(65535)

/* Range of Gyro */
#define LSM6DS0_G_FS_245                    (UINT8)(0x00) /* Full scale: 245 dps  */
#define LSM6DS0_G_FS_500                    (UINT8)(0x08) /* Full scale: 500 dps  */
#define LSM6DS0_G_FS_2000                   (UINT8)(0x18) /* Full scale: 2000 dps */

/* Range of Accelero */
#define LSM6DS0_XL_FS_2G                    (UINT8)(0x00) /* Full scale: +- 2g */
#define LSM6DS0_XL_FS_4G                    (UINT8)(0x10) /* Full scale: +- 4g */
#define LSM6DS0_XL_FS_8G                    (UINT8)(0x18) /* Full scale: +- 8g */
#define LSM6DS0_XL_FS_16G                   (UINT8)(0x08) /* Full scale: +- 16g*/
/* For Stereo - Tara End*/


/* Function Declarations */

BOOL InitExtensionUnit(char *busname);								//Initializes the Extension unit 

BOOL DeinitExtensionUnit(void);										//Deinits the Extension unit 

BOOL ReadFirmwareVersion(UINT8 *pMajorVersion, UINT8 *pMinorVersion1, UINT16 *pMinorVersion2, UINT16 *pMinorVersion3);													  //Reads the Firmware version of the device.

BOOL GetCameraUniqueID (char *UniqueID);							//Reads the unique ID of the camera

BOOL GetManualExposureValue_Stereo(INT32 *ManualExposureValue);		//Outputs the current Exposure Value of the camera

BOOL SetManualExposureValue_Stereo(INT32 ManualExposureValue);		//Sets the Exposure Value passed to the camera

BOOL GetIMUConfig(IMUCONFIG_TypeDef *lIMUConfig);					//Reads the current configuration of the IMU
	
BOOL SetIMUConfig(IMUCONFIG_TypeDef lIMUConfig);					//Sets the IMU configuration

BOOL ControlIMUCapture(IMUDATAINPUT_TypeDef *lIMUInput);			//Configures the IMU to read the IMU data in a specific format

BOOL GetIMUValueBuffer(IMUDATAOUTPUT_TypeDef *lIMUAxes);	//Reads the IMU values

BOOL GetIMUValueBuffer_write();									//Command to read IMU value

BOOL StereoCalibRead(unsigned char **IntrinsicBuffer, unsigned char **ExtrinsicBuffer, int *lIntFileLength, int *lExtFileLength);										 //Reads back the Intrinsic and Extrinsic values of the camera from the flash

BOOL GetStreamModeStereo(UINT32 *iStreamMode);			//Reads the mode in which the camera is set

BOOL SetStreamModeStereo(UINT32 iStreamMode);			//Sets the mode passed in the camera 


/* Function Declarations */

const char *bus_str(int);

unsigned int GetTickCount();

int find_hid_device(char *);

#endif
