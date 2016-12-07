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

#include "uvc_camera/tara_ros.h"

using namespace sensor_msgs;

namespace uvc_camera {

	BOOL	glIMUAbortThread;
	IMUDATAINPUT_TypeDef lIMUInput;


	taraCamera::taraCamera(ros::NodeHandle _comm_nh, ros::NodeHandle _param_nh) :
		node(_comm_nh), pnode(_param_nh), it(_comm_nh),
		info_mgr(_comm_nh, "camera"), info_mgr_left(_comm_nh, "cameraLeft"), info_mgr_right(_comm_nh, "cameraRight"), cam(0) {

			/* default config values */
			width = 640;
			height = 480;
			fps = 10;
			skip_frames = 0;
			frames_to_skip = 0;
			device = "/dev/video0";
			frame = "camera";
			frameLeft = "cameraLeft";
			frameRight = "cameraRight";
			rotate = false;
			exposure_value = 1;
			brightness_value = 4;
			// for IMU
			glIMUAbortThread = FALSE;
			glIMU_Interval = 0.0f;

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

			// changing start
			pnode.getParam ("exposureValue", exposure_value);

			/* advertise image streams and info streams */
			pub = it.advertise("image_raw", 1);
			pub_left = it.advertise("left//image_raw", 1);
			pub_right = it.advertise("right//image_raw", 1);
			pub_concat = it.advertise("concat", 1);

			exposure_pub = node.advertise<std_msgs::Float64>("get_exposure", 10000, true);
			brightness_pub = node.advertise<std_msgs::Float64>("get_brightness", 1, true);

			std::string url;
			std::string urlLeft;
			std::string urlRight;
			pnode.getParam("camera_info_url", url);
			pnode.getParam("cameraLeft_info_url", urlLeft);
			pnode.getParam("cameraRight_info_url", urlRight);

			/* initialize the cameras */
			cam = new uvc_cam::Cam(device.c_str(), uvc_cam::Cam::MODE_Y16, width, height, fps);

			cam -> showFirmwareVersion();

			unsigned char *in_buffer;
			unsigned char *ex_buffer;
			int intFileLength;
			int extFileLength;
			StereoCalibRead(&in_buffer, &ex_buffer, &intFileLength, &extFileLength);
			LoadCameraMatrix();   

			/* set up information manager */

			info_mgr.loadCameraInfo(url);
			info_mgr_left.loadCameraInfo(urlLeft);
			info_mgr_right.loadCameraInfo(urlRight);

			info_pub = node.advertise<CameraInfo>("camera_info", 1);
			info_pub_left = node.advertise<CameraInfo>("left//camera_info", 1);
			info_pub_right = node.advertise<CameraInfo>("right//camera_info", 1);

			returnValue = cam->set_control(V4L2_CID_BRIGHTNESS , 4); // brightness
			if (true == returnValue)
			{
				printf ("setting brightness : SUCCESS\n");
			}
			else
			{
				printf ("setting brightness : FAIL\n");
			}

			returnValue = SetManualExposureValue_Stereo( exposure_value); // exposure time 15.6ms
			if (true == returnValue)
			{
				printf ("setting exposure : SUCCESS\n");
			}
			else
			{
				printf ("setting exposure : FAIL\n");
			}
			std_msgs::Float64 exposure_msg;
			exposure_msg.data=(float)exposure_value;
			if ( GetManualExposureValue_Stereo( &exposure_value) == true )
			{
				exposure_msg.data = (float) exposure_value;
			}
			else
			{
				printf ("Error while getting exposure\n");
			}
			exposure_pub.publish( exposure_msg );

			std_msgs::Float64 brightness_msg;
			brightness_msg.data=(float)brightness_value;
			if ( cam -> get_control ( V4L2_CID_BRIGHTNESS, &brightness_value ) == true)
			{
				brightness_msg.data = brightness_value;
			}
			else
			{
				printf ("Error while getting brightness\n");
			}
			brightness_pub.publish( brightness_msg );

			/* and turn on the streamer */
			ok = true;
			image_thread = boost::thread(boost::bind(&taraCamera::feedImages, this));

			std::string time_topic;
			pnode.getParam("time_topic", time_topic);
			time_sub = node.subscribe("time_topic", 1, &taraCamera::timeCb, this );

			exposure_sub = node.subscribe ("set_exposure", 1, &taraCamera::callBackExposure, this);
			brightness_sub = node.subscribe ("set_brightness", 1, &taraCamera::callBackBrightness, this);

			IMU_pub = node.advertise<geometry_msgs::Point>("get_IMU", 1, true);
			IMU_thread = boost::thread(boost::bind(&taraCamera::IMU_enable, this));
		}

	void taraCamera::callBackExposure (std_msgs::Float64 call_exposure_msg)
	{
		DisableIMU();
		exposure_value=(float)call_exposure_msg.data;
		returnValue = SetManualExposureValue_Stereo( exposure_value); 
		if (true == returnValue)
		{
			printf ("setting exposure : SUCCESS\n");
		}
		else
		{
			printf ("setting exposure : FAIL\n");
		}

		if ( GetManualExposureValue_Stereo( &exposure_value) == true )
		{
			call_exposure_msg.data = exposure_value;
		}
		else
		{
			printf ("Error while getting exposure\n");
		}

		exposure_pub.publish( call_exposure_msg );

		SetIMUConfigDefault ();
		if (GetIMUValueBuffer_write() == true )
		{
			cout <<"GetIMUValueBuffer_write success" << endl;
		}
		else
		{
			cout <<"GetIMUValueBuffer_write failed" << endl;
		}
	}
	
	void taraCamera::callBackBrightness (std_msgs::Float64 call_brightness_msg)
	{
		DisableIMU();
		brightness_value=(float)call_brightness_msg.data;
		returnValue = cam -> set_control( V4L2_CID_BRIGHTNESS ,brightness_value); 
		if (true == returnValue)
		{
			printf ("setting brightness : SUCCESS\n");
		}
		else
		{
			printf ("setting brightness : FAIL\n");
		}

		if ( cam -> get_control ( V4L2_CID_BRIGHTNESS, &brightness_value ) == true)
		{
			call_brightness_msg.data = brightness_value;
		}
		else
		{
			printf ("Error while getting brightness\n");
		}
		brightness_pub.publish( call_brightness_msg );
		SetIMUConfigDefault ();
		if (GetIMUValueBuffer_write() == true )
		{
			cout <<"GetIMUValueBuffer_write success" << endl;
		}
		else
		{
			cout <<"GetIMUValueBuffer_write failed" << endl;
		}
	}

	void taraCamera::sendInfo(ImagePtr &image, ros::Time time) {
		CameraInfoPtr info(new CameraInfo(info_mgr.getCameraInfo()));

		/* Throw out any CamInfo that's not calibrated to this camera mode */
		if (info->K[0] != 0.0 &&
				(image->width != info->width
				 || image->height != info->height)) {
			info.reset(new CameraInfo());
		}

		/* If we don't have a calibration, set the image dimensions */
		if (info->K[0] == 0.0) {
			info->width = image->width;
			info->height = image->height;
		}

		info->header.stamp = time;
		info->header.frame_id = frame;
		/*      
			info -> binning_x = 2;
			info -> binning_y = 2;
		 */
		info_pub.publish(info);
	}
	void taraCamera::sendInfoRight(ImagePtr &image, ros::Time time) {
		CameraInfoPtr info(new CameraInfo(info_mgr_right.getCameraInfo()));

		/* Throw out any CamInfo that's not calibrated to this camera mode */
		if (info->K[0] != 0.0 &&
				(image->width != info->width
				 || image->height != info->height)) {
			info.reset(new CameraInfo());
		}

		/* If we don't have a calibration, set the image dimensions */
		if (info->K[0] == 0.0) {
			info->width = image->width;
			info->height = image->height;
		}

		info->header.stamp = time;
		info->header.frame_id = frameRight;

		info_pub_right.publish(info);
	}
	void taraCamera::sendInfoLeft(ImagePtr &image, ros::Time time) {
		CameraInfoPtr info(new CameraInfo(info_mgr_left.getCameraInfo()));

		/* Throw out any CamInfo that's not calibrated to this camera mode */
		if (info->K[0] != 0.0 &&
				(image->width != info->width
				 || image->height != info->height)) {
			info.reset(new CameraInfo());
		}

		/* If we don't have a calibration, set the image dimensions */
		if (info->K[0] == 0.0) {
			info->width = image->width;
			info->height = image->height;
		}

		info->header.stamp = time;
		info->header.frame_id = frameLeft;

		//      info -> binning_x = 2;
		//      info -> binning_y = 2;

		info_pub_left.publish(info);
	}

	void taraCamera::timeCb(std_msgs::Time time)
	{
		time_mutex_.lock();
		last_time = time.data;
		time_mutex_.unlock();
	}

	void taraCamera::feedImages() {
		unsigned int pair_id = 0;
		while (ok) {
			unsigned char *img_frame = NULL;
			unsigned char *right_frame = NULL;
			unsigned char *left_frame = NULL;
			unsigned char *concat_frame = NULL;
			uint32_t bytes_used;

			int idx = cam->grabStereo(&img_frame, bytes_used, &left_frame, &right_frame, &concat_frame);

			time_mutex_.lock();
			ros::Time capture_time = ros::Time::now();        
			time_mutex_.unlock();


			/* Read in every frame the camera generates, but only send each
			 * (skip_frames + 1)th frame. It's set up this way just because
			 * this is based on Stereo...
			 */
			if (skip_frames == 0 || frames_to_skip == 0) {
				if (img_frame) {
					ImagePtr image(new Image);
					image->height = height;
					image->width = width;
					image->step = width;

					image->header.stamp = capture_time;
					image->header.seq = pair_id;
					image->data.resize(image->step * image->height);

					/* left and right frame  */             
					image->encoding = image_encodings::MONO8;                        
					image->header.frame_id = frameRight;
					memcpy(&image->data[0], right_frame, image->data.size());
					pub_right.publish(image);
					sendInfoRight(image, capture_time);           

					image->header.frame_id = frameLeft;
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
					sendInfo(image, capture_time);
					/* concatenate frame  */             
					image->height = height;
					image->width = width * 2;
					image->step = width * 2;

					image->data.resize(image->step * image->height);		
					image->encoding = image_encodings::MONO8;                        
					memcpy(&image->data[0], concat_frame, image->data.size());
					pub_concat.publish(image);
					//		         sendInfo(image, capture_time);

					//ROS_INFO_STREAM("capture time: " << capture_time);
					++pair_id;
				}

				frames_to_skip = skip_frames;
			} else {
				frames_to_skip--;
			}

			if (img_frame) cam->release(idx);
		}
	}

	taraCamera::~taraCamera() {
		ok = false;
		image_thread.join();
		DisableIMU();

		glIMUAbortThread = FALSE;

		//Freeing the memory
		free(lIMUOutput);

		IMU_thread.join();
		if (cam) delete cam;
	}

	BOOL taraCamera::DisableIMU()
	{
		glIMUAbortThread = TRUE;
		lIMUInput.IMU_UPDATE_MODE = IMU_CONT_UPDT_DIS;
		lIMUInput.IMU_NUM_OF_VALUES = IMU_AXES_VALUES_MIN;

		//Resetting the IMU update to disable mode
		UINT8 uStatus = ControlIMUCapture(&lIMUInput);
		if(uStatus == FALSE)			
		{
			cout << "ControlIMUCapture Failed\n";
			return FALSE;
		}
	}

	BOOL taraCamera::LoadCameraMatrix()
	{

		printf ("in %s\n", __func__);
		unsigned char *IntrinsicBuffer, *ExtrinsicBuffer;
		int LengthIntrinsic, LengthExtrinsic;

		//Read the data from the flash
		if(StereoCalibRead(&IntrinsicBuffer, &ExtrinsicBuffer, &LengthIntrinsic, &LengthExtrinsic))
		{
			cout << "\nLoadCameraMatrix : Read Intrinsic and Extrinsic Files\n";
		}
		else
		{
			cout << "\nLoadCameraMatrix : Failed Reading Intrinsic and Extrinsic Files\n";
			return FALSE;
		}

		FILE *IntFile=NULL, *ExtFile=NULL;

		std::string intrinsic_file = "/.ros/camera_info/intrinsics.yaml";
		std::string extrinsic_file =  "/.ros/camera_info/extrinsics.yaml";

		std :: string intrinsic_file_path = getenv ("HOME") + intrinsic_file;
		std :: string extrinsic_file_path = getenv ("HOME") + extrinsic_file;

		IntFile = fopen( intrinsic_file_path.c_str(), "wb");
		ExtFile = fopen( extrinsic_file_path.c_str(), "wb");

		if(IntFile == NULL || ExtFile == NULL)
		{
			cout << "LoadCameraMatrix : Failed Opening Intrinsic and Extrinsic Files\n";
			perror("failed ");
			if(IntFile != NULL)
				fclose(IntFile);
			if(ExtFile != NULL)
				fclose(ExtFile);

			return FALSE;
		}

		if(LengthIntrinsic < 0 || LengthExtrinsic < 0)
		{
			cout << "LoadCameraMatrix : Invalid Intrinsic and Extrinsic File Length\n";
			fclose(IntFile);
			fclose(ExtFile);

			return FALSE;
		}

		fwrite(IntrinsicBuffer, 1, LengthIntrinsic, IntFile);
		fwrite(ExtrinsicBuffer, 1, LengthExtrinsic, ExtFile);

		fclose(IntFile);
		fclose(ExtFile);

		free ( IntrinsicBuffer );
		free ( ExtrinsicBuffer );

		YAML::Node intrinsic_matrix = YAML::LoadFile(intrinsic_file_path.c_str());
		YAML::Node extrinsic_matrix = YAML::LoadFile(extrinsic_file_path.c_str());

		std::string cameraLeft_name =  "/.ros/camera_info/cameraLeft.yaml";
		std::string cameraLeft_path = getenv("HOME") + cameraLeft_name;

		std::string cameraRight_name =  "/.ros/camera_info/cameraRight.yaml";
		std::string cameraRight_path = getenv("HOME") + cameraRight_name;

		std::ofstream foutLeft(cameraLeft_path.c_str());
		if (foutLeft == NULL)
		{
			printf ("Left camera matrix not found\n");
		}

		std::ofstream foutRight(cameraRight_path.c_str());	
		if (foutRight == NULL)
		{
			printf ("Right camera matrix not found\n");
		}

		foutLeft << "image_width: 640\nimage_height: 480\ncamera_name: cameraLeft\ncamera_matrix:\n   rows: 3\n   cols: 3\n   data: [";
		foutRight << "image_width: 640\nimage_height: 480\ncamera_name: cameraRight\ncamera_matrix:\n   rows: 3\n   cols: 3\n   data: [";   		

		printf ("printing value\n");
		int m1_found = 0;
		int d1_found = 0;
		int m2_found = 0;
		int d2_found = 0;

		while ( !d2_found || !d1_found)
		{
			for(YAML::const_iterator it = intrinsic_matrix.begin(); it!= intrinsic_matrix.end(); ++it)
			{
				const char *buffer = ((it -> first.as<std::string>()).c_str());

				if ( !strcmp (buffer, "M1") && m1_found == 0)
				{
					for (int iterator = 0; iterator < 9; iterator ++)
					{
						foutLeft << intrinsic_matrix["M1"]["data"][iterator];
						if (iterator < 9 - 1)
						{
							foutLeft << ", ";
						}
					}
					foutLeft << "]\ndistortion_model: plumb_bob\ndistortion_coefficients:\n   rows: 1\n   cols: 5\n   data: [";
					m1_found++;
				}
				if ( !strcmp (buffer, "M2") && m2_found == 0)
				{
					for (int iterator = 0; iterator < 9; iterator ++)
					{
						foutRight << intrinsic_matrix["M2"]["data"][iterator];
						if (iterator < 9 - 1)
						{
							foutRight << ", ";
						}
					}
					foutRight << "]\ndistortion_model: plumb_bob\ndistortion_coefficients:\n   rows: 1\n   cols: 5\n   data: [";
					m2_found++;
				}
				if ( !strcmp (buffer, "D1") && m1_found == 1 && d1_found == 0)
				{
					for (int iterator = 0; iterator < 5; iterator ++)
					{
						foutLeft << intrinsic_matrix["D1"]["data"][iterator];
						if (iterator < 5 - 1)
						{
							foutLeft << ", ";
						}
					}
					foutLeft << "]\n";
					d1_found++;
				}
				if ( !strcmp (buffer, "D2") && m2_found == 1  && d2_found == 0)
				{
					for (int iterator = 0; iterator < 5; iterator ++)
					{
						foutRight << intrinsic_matrix["D2"]["data"][iterator];
						if (iterator < 5 - 1)
						{
							foutRight << ", ";
						}
					}
					d2_found++;
					foutRight << "]\n";
				}
			}
		}

		foutLeft << "rectification_matrix:\n   rows: 3\n   cols: 3\n   data: [";
		foutRight << "rectification_matrix:\n   rows: 3\n   cols: 3\n   data: [";

		for(YAML::const_iterator it = extrinsic_matrix.begin(); it!= extrinsic_matrix.end(); ++it)
		{
			const char *buffer = ((it -> first.as<std::string>()).c_str());
			if ( !strcmp (buffer, "R1"))
			{
				for (int iterator = 0; iterator < 9; iterator ++)
				{
					foutLeft << extrinsic_matrix["R1"]["data"][iterator];
					if (iterator < 9 - 1)
					{
						foutLeft << ", ";
					}
				}

				foutLeft << "]\nprojection_matrix:\n   rows: 3\n   cols: 4\n   data: [";
			}
			if ( !strcmp (buffer, "R2"))
			{
				for (int iterator = 0; iterator < 9; iterator ++)
				{
					foutRight << extrinsic_matrix["R2"]["data"][iterator];
					if (iterator < 9 - 1)
					{
						foutRight << ", ";
					}
				}
				foutRight << "]\nprojection_matrix:\n   rows: 3\n   cols: 4\n   data: [";
			}
			if ( !strcmp (buffer, "P1"))
			{
				for (int iterator = 0; iterator < 12; iterator ++)
				{
					foutLeft << extrinsic_matrix["P1"]["data"][iterator];
					if (iterator < 12 - 1)
					{
						foutLeft << ", ";
					}
				}
				foutLeft << "]\n";
			}
			if ( !strcmp (buffer, "P2"))
			{
				for (int iterator = 0; iterator < 12; iterator ++)
				{
					foutRight << extrinsic_matrix["P2"]["data"][iterator];
					if (iterator < 12 - 1)
					{
						foutRight << ", ";
					}
				}
				foutRight << "]\n";
			}
		}
		std :: remove ( intrinsic_file_path.c_str() );
		std :: remove ( extrinsic_file_path.c_str() );
		foutLeft.close();
		foutRight.close();
		return TRUE;

	}
	double taraCamera::squared(double x)
	{
		return x * x;
	}

	void taraCamera::getInclination(double g_x, double g_y, double g_z, double a_x, double a_y, double a_z)
	{
		int w = 0;
		double tmpf = 0.0;
		int signRzGyro;
		static bool firstSample = true;
		double wGyro = 10.0;
		double norm;

		// Normalise the accelerometer measurement
		norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
		a_x /= norm;
		a_y /= norm;
		a_z /= norm;

		double RwAcc[3] = {a_x, a_y, a_z};
		double RwGyro[3] = { g_x, g_y, g_z };
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
				for (w = 0;w <= 2;w++) 
				{
					Gyro[w] = RwEst[w];
				}
			}
			else {
				//get angles between projection of R on ZX/ZY plane and Z axis, based on last RwEst
				for (w = 0;w <= 1;w++) 
				{
					tmpf = RwGyro[w];                        //get current gyro rate in deg/s
					tmpf *= glIMU_Interval / 1000.0f;                     //get angle change in deg
					Awz[w] = atan2(RwEst[w], RwEst[2]) * RAD2DEG;   //get angle and convert to degrees 
					Awz[w] += tmpf;             //get updated angle according to gyro movement
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
		IMU_pub.publish(IMUValue);

	}
	double taraCamera::GetIMUIntervalTime(IMUCONFIG_TypeDef	lIMUConfig)
	{
		double lIMUIntervalTime = 10;
		if(lIMUConfig.IMU_MODE == IMU_ACC_GYRO_ENABLE)
		{
			switch(lIMUConfig.IMU_ODR_CONFIG)
			{
				case IMU_ODR_10_14_9HZ:
					lIMUIntervalTime = 1000.00 / 14.90 ;
					break;

				case IMU_ODR_50_59_9HZ:
					lIMUIntervalTime = 1000.00 / 59.50 ;
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
		else if(lIMUConfig.IMU_MODE == IMU_ACC_ENABLE)
		{
			switch(lIMUConfig.IMU_ODR_CONFIG)
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
					lIMUIntervalTime = 1000.00  / 952.00;
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
		while(timeout)
		{
			end = GetTickCount();
			if(end - start > TimeInMilli)
			{
				timeout = FALSE;
			}
		}
		return;
	}

	void taraCamera::SetIMUConfigDefault()
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
		if(!uStatus)			
		{
			cout << "SetIMUConfig Failed\n";
			return ;
		}
		else
		{
			cout << "SetIMUConfig success\n";
			//Reading the configuration to verify the values are set 
			uStatus = GetIMUConfig(&lIMUConfig);
			if(!uStatus)			
			{
				cout << "GetIMUConfig Failed\n";			
				return ;
			}
			else
			{
				cout << "GetIMUConfig success\n";
			}
			//Finding the sampling interval time
			glIMU_Interval = GetIMUIntervalTime(lIMUConfig);
		}

		//Configuring IMU update mode
		lIMUInput.IMU_UPDATE_MODE = IMU_CONT_UPDT_EN;
		lIMUInput.IMU_NUM_OF_VALUES = IMU_AXES_VALUES_MIN;

		//Setting the IMU update mode using HID command
		uStatus = ControlIMUCapture(&lIMUInput);
		if(uStatus == FALSE)			
		{
			cout << "ControlIMUCapture Failed\n";
			return ;
		}
	}

	void taraCamera::IMU_enable()
	{
		cout << endl  << "		IMU Sample Application " << endl  << endl;
		cout << " Application to illustrate the IMU unit LSM6DS0 integrated with Tara Camera" << endl;
		cout << " Demonstrating the rotations of camera around x-axis and y-axis " << endl;
		cout << " IMU values are limited from -90 to +90 degrees for illustration " << endl << endl;

		SetIMUConfigDefault();

		//Allocating buffers for output structure			
		lIMUOutput = (IMUDATAOUTPUT_TypeDef*)malloc(1 * sizeof(IMUDATAOUTPUT_TypeDef));

		//Memory validation
		if(lIMUOutput == NULL)
		{
			cout << "Memory Allocation for output failed\n";
			return ;
		}

		lIMUOutput->IMU_VALUE_ID = 0;

		if (GetIMUValueBuffer_write() == true )
		{
			cout <<"GetIMUValueBuffer_write success" << endl;
		}
		else
		{
			cout <<"GetIMUValueBuffer_write failed" << endl;
		}

		while (ros::ok() )
		{
			//HID command
			if( !GetIMUValueBuffer ( lIMUOutput ) )
			{
				cout << "GetIMUValueBuffer failed\n";
			}
			//Calculating angles based on the current raw values from IMU			
			getInclination(lIMUOutput->gyroX, lIMUOutput->gyroY, lIMUOutput->gyroZ, lIMUOutput->accX, lIMUOutput->accY, lIMUOutput->accZ);
		}
	}

	int taraCamera::econ_strcmp (const char *str1,const char *str2)
	{
		int iter = 0;
		for (iter = 0; str1 [iter] && str2 [iter] ; iter ++ )
		{
			if (str1[iter] != str2[iter] )
			{
				if ( str1 [iter] > str2 [iter] )
				{
					return 1;
				}
				else
				{
					return -1;
				}
			}
		}

		if ( str1 [iter] > str2 [iter] )
		{
			return 1;
		}
		else
		{
			if ( str1 [iter] < str2 [iter] )
			{
				return -1;
			}
			else
			{
				return 0;
			}
		}
	}
};
