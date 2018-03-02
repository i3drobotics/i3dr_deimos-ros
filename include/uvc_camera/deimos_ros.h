#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "uvc_cam/uvc_cam.h"
#include <boost/thread.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include "std_msgs/Time.h"
#include "std_msgs/Float64.h"
#include <libv4l2.h>
#include "camera_calibration_parsers/parse_yml.h"
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "geometry_msgs/Point.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Imu.h"
#include <memory>

using namespace std;

#define		M_PI				3.14159265358979323846
#define		HALF_PI				(M_PI / 2)
#define		DEG2RAD				(M_PI / 180.f)
#define		RAD2DEG				(180.f / M_PI)

//	1.2.131.652 is the last firmware version of Tara that doesn't support auto exposure.
#define 		MajorVersion_t		1
#define 		MinorVersion1_t	2
#define 		MinorVersion2_t	131
#define 		MinorVersion3_t	652

#define sampleFreq     119.0f                                   // sample frequency in Hz
#define gyroMeasError  0.1                                      // gyroscope measurement error in rad/s
#define betaDef        sqrt(3.0f / 4.0f) * gyroMeasError        // compute beta

namespace uvc_camera {

	void Sleep(unsigned int TimeInMilli);

	class deimosCamera {
		public:

			IMUCONFIG_TypeDef lIMUConfig;
			IMUDATAINPUT_TypeDef lIMUInput;
			IMUDATAOUTPUT_TypeDef *lIMUOutput;
			bool isCameraStereo;

			deimosCamera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh);
			void onInit();
			void sendInfoLeft(sensor_msgs::ImagePtr &image, ros::Time time);
			void sendInfoRight(sensor_msgs::ImagePtr &image, ros::Time time);
			void feedImages();
			~deimosCamera();
			void timeCb(std_msgs::Time time);
			//IMU
			void getInclination(double w_x, double w_y, double w_z, double a_x, double a_y, double a_z);
			// getOrientation return the orientation in quaternion format
			void getOrientation(double w_x, double w_y, double w_z, double a_x, double a_y, double a_z);
			int returnValue;

		private:
			ros::NodeHandle node, pnode;
			image_transport::ImageTransport it;
			bool ok;

			int width, height, fps, skip_frames, frames_to_skip;
			std::string device, frame;
			std::string frameImageLeft;
			std::string frameImageRight;
			std::string frameCameraInfoLeft;
			std::string frameCameraInfoRight;
			std::string frameIMU;
			int  exposure_value;
			int  brightness_value;
			bool rotate;

			camera_info_manager::CameraInfoManager* info_mgr_left;
			camera_info_manager::CameraInfoManager* info_mgr_right;

			image_transport::Publisher pub, pub_left, pub_right, pub_concat;
			ros::Publisher info_pub;
			ros::Publisher info_pub_left;
			ros::Publisher info_pub_right;    
			ros::Publisher exposure_pub;
			ros::Publisher brightness_pub;
			ros::Publisher IMU_inclination_pub;
			ros::Publisher IMU_pub;

			ros::Subscriber time_sub;
			ros::Subscriber exposure_sub;
			ros::Subscriber brightness_sub;

			ros::Time last_time;
			boost::mutex time_mutex_;

			uvc_cam::Cam *cam;
			boost::thread image_thread;
			boost::thread IMU_thread;
			volatile float beta;	// 2 * proportional gain (Kp)
			volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

			double angleX, angleY, angleZ; // Rotational angle for cube [NEW]
			double RwEst[3];

			double squared(double x);
			double glIMU_Interval;

			void callBackExposure(std_msgs::Float64 call_exposure_value);
			void callBackBrightness(std_msgs::Float64 call_brightness_value);
			void SetIMUConfigDefaultEnable();
			void IMU_enable();    
			int econ_strcmp (const char * str1, const char *str2);
			/*  Returns the interval time for sampling the values of the IMU. */
			double GetIMUIntervalTime(IMUCONFIG_TypeDef	lIMUConfig);
			BOOL DisableIMU();
			BOOL checkFirmware (UINT8 MajorVersion, UINT8 MinorVersion1, UINT16 MinorVersion2, UINT16 MinorVersion3);		//Returns 1 if firmware supports auto exposure, else 0;
			float invSqrt(float x);

	};

};

