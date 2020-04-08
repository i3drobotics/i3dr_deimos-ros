#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/time.h>

#include "uvc_cam/uvc_cam.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/CameraInfo.h"
#include "camera_info_manager/camera_info_manager.h"
#include "image_transport/image_transport.h"
#include "std_msgs/Float64.h"

#include "uvc_camera/deimos_ros.h"

#define MADGWICK_EN 1
#define IMU_ODOM_EN 0

using namespace sensor_msgs;

namespace uvc_camera
{

IMUDATAINPUT_TypeDef lIMUInput;

deimosCamera::deimosCamera(ros::NodeHandle _comm_nh, ros::NodeHandle _param_nh) : node(_comm_nh), pnode(_param_nh), it(_comm_nh), cam(0)
{
	/* default config values */
	width = 752;
	height = 480;
	fps = 60;
	skip_frames = 0;
	frames_to_skip = 0;
	device = "/dev/video0";
	frame = "deimos_depth_optical_frame";
	frameIMU = "deimos_base_link";
	frameImageLeft = frame;
	frameImageRight = frame;
	frameCameraInfoLeft = frame;
	frameCameraInfoRight = frame;
	rotate = false;
	exposure_value = 0;
	brightness_value = 0;
	// for IMU
	glIMU_Interval = 0.0f;
	isCameraStereo = false;

	angleX = 0.0f;
	angleY = 0.0f;
	angleZ = 0.0f;

	/* pull other configuration */
	pnode.getParam("device", device);
	pnode.getParam("fps", fps);
	pnode.getParam("skip_frames", skip_frames);
	pnode.getParam("width", width);
	pnode.getParam("height", height);
	pnode.getParam("frame_id", frame);
	pnode.getParam("imu_frame_id", frameIMU);

	frameImageLeft = frame;
	frameImageRight = frame;
	frameCameraInfoLeft = frame;
	frameCameraInfoRight = frame;

	// changing start
	pnode.getParam("exposureValue", exposure_value);

	/* advertise image streams and info streams */
	pub = it.advertise("image_raw", 1);
	pub_left = it.advertise("left//image_raw", 1);
	pub_right = it.advertise("right//image_raw", 1);
	pub_concat = it.advertise("concat", 1);

	exposure_pub = node.advertise<std_msgs::Float64>("get_exposure", 1, true);
	brightness_pub = node.advertise<std_msgs::Float64>("get_brightness", 1, true);

	std::string urlLeft;
	std::string urlRight;
	std::string leftCameraName = "cameraLeft";
	std::string rightCameraName = "cameraRight";

	std::string ns = ros::this_node::getNamespace();

	auto node_left = ros::NodeHandle(ns + "/left");
	auto node_right = ros::NodeHandle(ns + "/right");

	pnode.getParam("cameraLeft_info_url", urlLeft);
	pnode.getParam("cameraRight_info_url", urlRight);
	pnode.getParam("leftCameraName", leftCameraName);
	pnode.getParam("rightCameraName", leftCameraName);

	ROS_INFO("Camera Left Info URL: %s",urlLeft.c_str());
	ROS_INFO("Camera Left Info URL: %s",urlRight.c_str());

	info_mgr_left = new camera_info_manager::CameraInfoManager(node_left, leftCameraName, urlLeft);
	info_mgr_right = new camera_info_manager::CameraInfoManager(node_right, rightCameraName, urlRight);

	/* initialize the cameras */
	cam = new uvc_cam::Cam(device.c_str(), uvc_cam::Cam::MODE_Y16, width, height, fps);

	if (cam->IsStereo == true)
	{
		isCameraStereo = true;
		cam->showFirmwareVersion();
		unsigned char *in_buffer;
		unsigned char *ex_buffer;
		int intFileLength;
		int extFileLength;
		StereoCalibRead(&in_buffer, &ex_buffer, &intFileLength, &extFileLength);

		/* set up information manager */

		info_mgr_left->loadCameraInfo(urlLeft);
		info_mgr_right->loadCameraInfo(urlRight);

		info_pub_left = node.advertise<CameraInfo>("left//camera_info", 1);
		info_pub_right = node.advertise<CameraInfo>("right//camera_info", 1);

		returnValue = cam->set_control(V4L2_CID_BRIGHTNESS, 4); // brightness
		if (false == returnValue)
		{
			printf("setting brightness : FAIL\n");
		}

		if ((exposure_value > SEE3CAM_STEREO_EXPOSURE_MAX) || (exposure_value < SEE3CAM_STEREO_EXPOSURE_MIN))
		{
			if (checkFirmware(cam->MajorVersion, cam->MinorVersion1, cam->MinorVersion2, cam->MinorVersion3))
			{
				exposure_value = 1;
			}
			else
			{
				exposure_value = 15000;
			}
		}

		returnValue = SetManualExposureValue_Stereo(exposure_value); // exposure time 15.6ms

		if (false == returnValue)
		{
			printf("setting exposure : FAIL\n");
		}

		returnValue = cam->set_control(V4L2_CID_AUTOGAIN, false); // brightness
		if (false == returnValue)
		{
			printf("setting auto gain : FAIL\n");
		}

		returnValue = cam->set_control(V4L2_CID_EXPOSURE_AUTO, false); // auto exposure
		if (false == returnValue)
		{
			printf("setting auto exposure : FAIL\n");
		}

		returnValue = cam->set_control(V4L2_CID_GAIN, 0); // brightness
		if (false == returnValue)
		{
			printf("setting gain : FAIL\n");
		}

		std_msgs::Float64 exposure_msg;
		exposure_msg.data = (float)exposure_value;
		if (GetManualExposureValue_Stereo(&exposure_value) == true)
		{
			exposure_msg.data = (float)exposure_value;
		}
		else
		{
			printf("Error while getting exposure\n");
		}
		exposure_pub.publish(exposure_msg);

		std_msgs::Float64 brightness_msg;
		brightness_msg.data = (float)brightness_value;
		if (cam->get_control(V4L2_CID_BRIGHTNESS, &brightness_value) == true)
		{
			brightness_msg.data = brightness_value;
		}
		else
		{
			printf("Error while getting brightness\n");
		}
		brightness_pub.publish(brightness_msg);

		/* and turn on the streamer */
		ok = true;
		image_thread = boost::thread(boost::bind(&deimosCamera::feedImages, this));

		std::string time_topic;
		pnode.getParam("time_topic", time_topic);
		time_sub = node.subscribe("time_topic", 1, &deimosCamera::timeCb, this);

		exposure_sub = node.subscribe("set_exposure", 1, &deimosCamera::callBackExposure, this);
		brightness_sub = node.subscribe("set_brightness", 1, &deimosCamera::callBackBrightness, this);

		IMU_pub = node.advertise<sensor_msgs::Imu>("imu/data_raw", 1, true);
		IMU_inclination_pub = node.advertise<geometry_msgs::Point>("get_inclination", 1, true);
		IMU_thread = boost::thread(boost::bind(&deimosCamera::IMU_enable, this));

#if IMU_ODOM_EN
		IMU_odom_pub = node.advertise<nav_msgs::Odometry>("odom", 1);
#endif
	}
}

void deimosCamera::callBackExposure(std_msgs::Float64 call_exposure_msg)
{
	DisableIMU();
	exposure_value = (float)call_exposure_msg.data;
	returnValue = SetManualExposureValue_Stereo(exposure_value);
	if (true == returnValue)
	{
		printf("setting exposure : SUCCESS\n");
	}
	else
	{
		printf("setting exposure : FAIL\n");
	}

	if (GetManualExposureValue_Stereo(&exposure_value) == true)
	{
		call_exposure_msg.data = exposure_value;
	}
	else
	{
		printf("Error while getting exposure\n");
	}

	exposure_pub.publish(call_exposure_msg);

	SetIMUConfigDefaultEnable();
	if (GetIMUValueBuffer_write() == false)
	{
		cout << "GetIMUValueBuffer_write failed" << endl;
	}
}

void deimosCamera::callBackBrightness(std_msgs::Float64 call_brightness_msg)
{
	DisableIMU();
	brightness_value = (float)call_brightness_msg.data;
	returnValue = cam->set_control(V4L2_CID_BRIGHTNESS, brightness_value);
	if (true == returnValue)
	{
		printf("setting brightness : SUCCESS\n");
	}
	else
	{
		printf("setting brightness : FAIL\n");
	}

	if (cam->get_control(V4L2_CID_BRIGHTNESS, &brightness_value) == true)
	{
		call_brightness_msg.data = brightness_value;
	}
	else
	{
		printf("Error while getting brightness\n");
	}
	brightness_pub.publish(call_brightness_msg);
	SetIMUConfigDefaultEnable();
	if (GetIMUValueBuffer_write() == false)
	{
		cout << "GetIMUValueBuffer_write failed" << endl;
	}
}

void deimosCamera::sendInfoRight(ImagePtr &image, ros::Time time)
{
	CameraInfoPtr info(new CameraInfo(info_mgr_right->getCameraInfo()));

	/* Throw out any CamInfo that's not calibrated to this camera mode */
	if (info->K[0] != 0.0 &&
		(image->width != info->width || image->height != info->height))
	{
		info.reset(new CameraInfo());
	}

	/* If we don't have a calibration, set the image dimensions */
	if (info->K[0] == 0.0)
	{
		info->width = image->width;
		info->height = image->height;
	}

	info->header.stamp = time;

	/* Important: this is where we derive the tf from *not* the image header */

	info->header.frame_id = frameCameraInfoRight;

	info_pub_right.publish(info);
}
void deimosCamera::sendInfoLeft(ImagePtr &image, ros::Time time)
{
	CameraInfoPtr info(new CameraInfo(info_mgr_left->getCameraInfo()));

	/* Throw out any CamInfo that's not calibrated to this camera mode */
	if (info->K[0] != 0.0 &&
		(image->width != info->width || image->height != info->height))
	{
		info.reset(new CameraInfo());
	}

	/* If we don't have a calibration, set the image dimensions */
	if (info->K[0] == 0.0)
	{
		info->width = image->width;
		info->height = image->height;
	}

	info->header.stamp = time;
	/* Important: this is where we derive the tf from *not* the image header */

	info->header.frame_id = frameCameraInfoLeft;

	//      info -> binning_x = 2;
	//      info -> binning_y = 2;

	info_pub_left.publish(info);
}

void deimosCamera::timeCb(std_msgs::Time time)
{
	time_mutex_.lock();
	last_time = time.data;
	time_mutex_.unlock();
}

void deimosCamera::feedImages()
{
	unsigned int pair_id = 0;
	while (ok)
	{
		unsigned char *img_frame = nullptr;
		unsigned char *right_frame = nullptr;
		unsigned char *left_frame = nullptr;
		unsigned char *concat_frame = nullptr;
		uint32_t bytes_used;

		int idx = cam->grabStereo(&img_frame, bytes_used, &left_frame, &right_frame, &concat_frame);

		time_mutex_.lock();
		ros::Time capture_time = ros::Time::now();
		time_mutex_.unlock();

		/* Read in every frame the camera generates, but only send each
			 * (skip_frames + 1)th frame. It's set up this way just because
			 * this is based on Stereo...
			 */
		if (skip_frames == 0 || frames_to_skip == 0)
		{
			if (img_frame)
			{
				ImagePtr image(new Image);
				image->height = height;
				image->width = width;
				image->step = width;

				image->header.stamp = capture_time;
				image->header.seq = pair_id;
				image->data.resize(image->step * image->height);

				/* left and right frame  */
				image->encoding = image_encodings::MONO8;
				image->header.frame_id = frameImageRight;
				memcpy(&image->data[0], right_frame, image->data.size());
				pub_right.publish(image);
				sendInfoRight(image, capture_time);

				image->header.frame_id = frameImageLeft;
				memcpy(&image->data[0], left_frame, image->data.size());
				pub_left.publish(image);
				sendInfoLeft(image, capture_time);

				/* raw frame  */

				image->height = height;
				image->width = width * 2;
				image->step = width * 2;

				image->header.stamp = capture_time;
				image->header.seq = pair_id;
				image->header.frame_id = frame;
				image->data.resize(image->step * image->height);

				image->encoding = image_encodings::MONO8;
				memcpy(&image->data[0], img_frame, image->data.size());
				pub.publish(image);

				/* concatenate frame  */
				image->height = height;
				image->width = width * 2;
				image->step = width * 2;

				image->data.resize(image->step * image->height);
				image->encoding = image_encodings::MONO8;
				memcpy(&image->data[0], concat_frame, image->data.size());
				pub_concat.publish(image);

				//ROS_INFO_STREAM("capture time: " << capture_time);
				++pair_id;
			}

			frames_to_skip = skip_frames;
		}
		else
		{
			frames_to_skip--;
		}

		if (img_frame)
			cam->release(idx);
	}
}

deimosCamera::~deimosCamera()
{
	if (isCameraStereo == true)
	{
		ok = false;
		image_thread.join();
		DisableIMU();

		//Freeing the memory
		free(lIMUOutput);
		IMU_thread.join();
	}
	if (cam)
		delete cam;
}

BOOL deimosCamera::DisableIMU()
{
	lIMUInput.IMU_UPDATE_MODE = IMU_CONT_UPDT_DIS;
	lIMUInput.IMU_NUM_OF_VALUES = IMU_AXES_VALUES_MIN;

	//Resetting the IMU update to disable mode
	UINT8 uStatus = ControlIMUCapture(&lIMUInput);
	if (uStatus == FALSE)
	{
		cout << "ControlIMUCapture Failed\n";
		return FALSE;
	}
}

double deimosCamera::squared(double x)
{
	return x * x;
}

void deimosCamera::getInclination(double g_x, double g_y, double g_z, double a_x, double a_y, double a_z)
{
	int w = 0;
	double tmpf = 0.0;
	int signRzGyro;
	static bool firstSample = true;
	double wGyro = 10.0;
	double norm;

	getOrientation(g_x, g_y, g_z, a_x, a_y, a_z);

	// Normalise the accelerometer measurement
	norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;

	double RwAcc[3] = {a_x, a_y, a_z};
	double RwGyro[3] = {g_x, g_y, g_z};
	double Awz[2];
	double Gyro[3];

	if (firstSample)
	{
		//initialize with accelerometer readings
		for (w = 0; w <= 2; w++)
		{
			RwEst[w] = RwAcc[w];
		}
	}

	else
	{
		//evaluate Gyro vector
		if (fabs(RwEst[2]) < 0.1)
		{
			//Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
			//in this case skip the gyro data and just use previous estimate
			for (w = 0; w <= 2; w++)
			{
				Gyro[w] = RwEst[w];
			}
		}
		else
		{
			//get angles between projection of R on ZX/ZY plane and Z axis, based on last RwEst
			for (w = 0; w <= 1; w++)
			{
				tmpf = RwGyro[w];							  //get current gyro rate in deg/s
				tmpf *= glIMU_Interval / 1000.0f;			  //get angle change in deg
				Awz[w] = atan2(RwEst[w], RwEst[2]) * RAD2DEG; //get angle and convert to degrees
				Awz[w] += tmpf;								  //get updated angle according to gyro movement
			}

			//estimate sign of RzGyro by looking in what qudrant the angle Axz is,
			//RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
			signRzGyro = (cos(Awz[0] * DEG2RAD) >= 0) ? 1 : -1;

			//reverse calculation of Gyro from Awz angles, for formulas deductions see  http://starlino.com/imu_guide.html
			Gyro[0] = sin(Awz[0] * DEG2RAD);
			Gyro[0] /= sqrt(1 + squared(cos(Awz[0] * DEG2RAD)) * squared(tan(Awz[1] * DEG2RAD)));
			Gyro[1] = sin(Awz[1] * DEG2RAD);
			Gyro[1] /= sqrt(1 + squared(cos(Awz[1] * DEG2RAD)) * squared(tan(Awz[0] * DEG2RAD)));
			Gyro[2] = signRzGyro * sqrt(1 - squared(Gyro[0]) - squared(Gyro[1]));
		}

		//combine Accelerometer and gyro readings
		for (w = 0; w <= 2; w++)
			RwEst[w] = (RwAcc[w] + wGyro * Gyro[w]) / (1 + wGyro);

		//Normalizing the estimates
		norm = sqrt(RwEst[0] * RwEst[0] + RwEst[1] * RwEst[1] + RwEst[2] * RwEst[2]);
		RwEst[0] /= norm;
		RwEst[1] /= norm;
		RwEst[2] /= norm;
	}

	firstSample = false;

	//Computing the angles
	angleX = RwEst[0] * HALF_PI * RAD2DEG;
	angleY = RwEst[1] * HALF_PI * RAD2DEG;
	angleZ = RwEst[2] * HALF_PI * RAD2DEG;

	geometry_msgs::Point IMUValue;
	IMUValue.x = angleX;
	IMUValue.y = angleY;
	IMUValue.z = angleZ;
	IMU_inclination_pub.publish(IMUValue);
}
double deimosCamera::GetIMUIntervalTime(IMUCONFIG_TypeDef lIMUConfig)
{
	double lIMUIntervalTime = 10;
	if (lIMUConfig.IMU_MODE == IMU_ACC_GYRO_ENABLE)
	{
		switch (lIMUConfig.IMU_ODR_CONFIG)
		{
		case IMU_ODR_10_14_9HZ:
			lIMUIntervalTime = 1000.00 / 14.90;
			break;

		case IMU_ODR_50_59_9HZ:
			lIMUIntervalTime = 1000.00 / 59.50;
			break;

		case IMU_ODR_119HZ:
			lIMUIntervalTime = 1000.00 / 119.00;
			break;

		case IMU_ODR_238HZ:
			lIMUIntervalTime = 1000.00 / 238.00;
			break;

		case IMU_ODR_476HZ:
			lIMUIntervalTime = 1000.00 / 476.00;
			break;

		case IMU_ODR_952HZ:
			lIMUIntervalTime = 1000.00 / 952.00;
			break;
		}
	}
	else if (lIMUConfig.IMU_MODE == IMU_ACC_ENABLE)
	{
		switch (lIMUConfig.IMU_ODR_CONFIG)
		{
		case IMU_ODR_10_14_9HZ:
			lIMUIntervalTime = 1000.00 / 10.00;
			break;

		case IMU_ODR_50_59_9HZ:
			lIMUIntervalTime = 1000.00 / 50.00;
			break;

		case IMU_ODR_119HZ:
			lIMUIntervalTime = 1000.00 / 119.00;
			break;

		case IMU_ODR_238HZ:
			lIMUIntervalTime = 1000.00 / 238.00;
			break;

		case IMU_ODR_476HZ:
			lIMUIntervalTime = 1000.00 / 476.00;
			break;

		case IMU_ODR_952HZ:
			lIMUIntervalTime = 1000.00 / 952.00;
			break;
		}
	}
	return lIMUIntervalTime;
}

void Sleep(unsigned int TimeInMilli)
{
	BOOL timeout = TRUE;
	unsigned int start, end = 0;

	start = GetTickCount();
	while (timeout)
	{
		end = GetTickCount();
		if (end - start > TimeInMilli)
		{
			timeout = FALSE;
		}
	}
	return;
}

void deimosCamera::SetIMUConfigDefaultEnable()
{
	UINT8 uStatus = 0;
	//Configuring IMU rates
	lIMUConfig.IMU_MODE = IMU_ACC_GYRO_ENABLE;
	lIMUConfig.ACC_AXIS_CONFIG = IMU_ACC_X_Y_Z_ENABLE;
	lIMUConfig.IMU_ODR_CONFIG = IMU_ODR_119HZ;
	lIMUConfig.ACC_SENSITIVITY_CONFIG = IMU_ACC_SENS_2G;
	lIMUConfig.GYRO_AXIS_CONFIG = IMU_GYRO_X_Y_Z_ENABLE;
	lIMUConfig.GYRO_SENSITIVITY_CONFIG = IMU_GYRO_SENS_245DPS;

	//Setting the configuration using HID command
	uStatus = SetIMUConfig(lIMUConfig);
	if (!uStatus)
	{
		cout << "SetIMUConfig Failed\n";
		return;
	}
	else
	{
		//Reading the configuration to verify the values are set
		uStatus = GetIMUConfig(&lIMUConfig);
		if (!uStatus)
		{
			cout << "GetIMUConfig Failed\n";
			return;
		}
		//Finding the sampling interval time
		glIMU_Interval = GetIMUIntervalTime(lIMUConfig);
	}

	//Configuring IMU update mode
	lIMUInput.IMU_UPDATE_MODE = IMU_CONT_UPDT_EN;
	lIMUInput.IMU_NUM_OF_VALUES = IMU_AXES_VALUES_MIN;

	//Setting the IMU update mode using HID command
	uStatus = ControlIMUCapture(&lIMUInput);
	if (uStatus == FALSE)
	{
		cout << "ControlIMUCapture Failed\n";
		return;
	}
}

void deimosCamera::IMU_enable()
{
	SetIMUConfigDefaultEnable();
	//Allocating buffers for output structure
	lIMUOutput = (IMUDATAOUTPUT_TypeDef *)malloc(1 * sizeof(IMUDATAOUTPUT_TypeDef));
	//Memory validation
	if (lIMUOutput == nullptr)
	{
		cout << "Memory Allocation for output failed\n";
		return;
	}

	lIMUOutput->IMU_VALUE_ID = 0;

	if (GetIMUValueBuffer_write() == false)
	{
		cout << "GetIMUValueBuffer_write failed" << endl;
	}

	while (ros::ok())
	{
		//HID command
		if (!GetIMUValueBuffer(lIMUOutput))
		{
			cout << "GetIMUValueBuffer failed\n";
		}
		//Calculating angles based on the current raw values from IMU
		getInclination(lIMUOutput->gyroX, lIMUOutput->gyroY, lIMUOutput->gyroZ, lIMUOutput->accX, lIMUOutput->accY, lIMUOutput->accZ);
	}
}

int deimosCamera::econ_strcmp(const char *str1, const char *str2)
{
	int iter = 0;
	for (iter = 0; str1[iter] && str2[iter]; iter++)
	{
		if (str1[iter] != str2[iter])
		{
			if (str1[iter] > str2[iter])
			{
				return 1;
			}
			else
			{
				return -1;
			}
		}
	}

	if (str1[iter] > str2[iter])
	{
		return 1;
	}
	else
	{
		if (str1[iter] < str2[iter])
		{
			return -1;
		}
		else
		{
			return 0;
		}
	}
}

BOOL deimosCamera::checkFirmware(UINT8 MajorVersion, UINT8 MinorVersion1, UINT16 MinorVersion2, UINT16 MinorVersion3)
{
	if (MajorVersion > MajorVersion_t)
	{
		return 1;
	}
	else
	{
		if (MinorVersion1 > MinorVersion1_t)
		{
			return 1;
		}
		else
		{
			if (MinorVersion2 > MinorVersion2_t)
			{
				return 1;
			}
			else
			{
				if (MinorVersion3 > MinorVersion3_t)
				{
					return 1;
				}
				else
				{
					return 0;
				}
			}
		}
	}
}
void deimosCamera::getOrientation(double gx, double gy, double gz, double ax, double ay, double az)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	sensor_msgs::Imu IMUValue;

	IMUValue.orientation.x = 0.0;
	IMUValue.orientation.y = 0.0;
	IMUValue.orientation.z = 0.0;
	IMUValue.orientation.w = 0.0;

	double linear_accel_cov = 0.01;
	double angular_velocity_cov = 0.01;

	IMUValue.orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	IMUValue.linear_acceleration_covariance = {linear_accel_cov, 0.0, 0.0, 0.0, linear_accel_cov, 0.0, 0.0, 0.0, linear_accel_cov};
	IMUValue.angular_velocity_covariance = {angular_velocity_cov, 0.0, 0.0, 0.0, angular_velocity_cov, 0.0, 0.0, 0.0, angular_velocity_cov};

	// To convert acceleration from 'mg' to 'm/s2'
	IMUValue.linear_acceleration.x = ax * 9.80665 / 1000;
	IMUValue.linear_acceleration.y = ay * 9.80665 / 1000;
	IMUValue.linear_acceleration.z = az * 9.80665 / 1000;

	// To convert acceleration from 'dps' to 'radians/second'

	gx *= 0.0174533;
	gy *= 0.0174533;
	gz *= 0.0174533;

	IMUValue.angular_velocity.x = gx;
	IMUValue.angular_velocity.y = gy;
	IMUValue.angular_velocity.z = gz;

	// Orientation calculation implemented according to Madgwick
	// Refer : http://x-io.co.uk/res/doc/madgwick_internal_report.pdf
	// Accuracy - Unknown
#if MADGWICK_EN
	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	IMUValue.orientation.w = q0;
	IMUValue.orientation.x = q1;
	IMUValue.orientation.y = q2;
	IMUValue.orientation.z = q3;
#endif

#if IMU_ODOM_EN
	nav_msgs::Odometry IMU_odom;
	geometry_msgs::Quaternion odom_quat;
	IMU_odom.header.stamp = ros::Time::now();
	IMU_odom.header.frame_id = "odom";
	//set the orientation
	odom_quat.w = q0;
	odom_quat.x = q1;
	odom_quat.y = q2;
	odom_quat.z = q3;
	//set the position
	IMU_odom.pose.pose.position.x = 0.0;
	IMU_odom.pose.pose.position.y = 0.0;
	IMU_odom.pose.pose.position.z = 0.0;
	IMU_odom.pose.pose.orientation = odom_quat;
	//set the covariance
	//float64[36] covariance;
	//IMU_odom.pose.covariance = covariance;
	IMU_odom_pub.publish(IMU_odom);
#endif

	IMUValue.header.frame_id = frameIMU;
	IMUValue.header.stamp = ros::Time::now();
	IMU_pub.publish(IMUValue);
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float deimosCamera::invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
}; // namespace uvc_camera
