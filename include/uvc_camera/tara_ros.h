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
#include <string.h>
#include <tf/transform_broadcaster.h>

using namespace std;

#define		M_PI				3.14159265358979323846
#define		HALF_PI				(M_PI / 2)
#define		DEG2RAD				(M_PI / 180.f)
#define		RAD2DEG				(180.f / M_PI)
#define 		TARA_STREAM_APP	0
#define 		IMU_APP				1

namespace uvc_camera {

	//Keyboard hit detection
	void Sleep(unsigned int TimeInMilli);

	class taraCamera {
		public:

			IMUCONFIG_TypeDef lIMUConfig;
			IMUDATAINPUT_TypeDef lIMUInput;
			IMUDATAOUTPUT_TypeDef *lIMUOutput;

			taraCamera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh);
			void onInit();
			void sendInfo(sensor_msgs::ImagePtr &image, ros::Time time);
			void sendInfoLeft(sensor_msgs::ImagePtr &image, ros::Time time);
			void sendInfoRight(sensor_msgs::ImagePtr &image, ros::Time time);
			void feedImages();
			~taraCamera();
			void timeCb(std_msgs::Time time);
			BOOL LoadCameraMatrix();
			
			//IMU
			void getInclination(double w_x, double w_y, double w_z, double a_x, double a_y, double a_z);
			//     double GetIMUIntervalTime(IMUCONFIG_TypeDef	lIMUConfig);
			int returnValue;

		private:
			ros::NodeHandle node, pnode;
			image_transport::ImageTransport it;
			bool ok;

			int width, height, fps, skip_frames, frames_to_skip;
			std::string device, frame;
			std::string frameLeft;
			std::string frameRight;
			int  exposure_value;
			int  brightness_value;
			bool rotate;

			camera_info_manager::CameraInfoManager info_mgr;
			camera_info_manager::CameraInfoManager info_mgr_left;
			camera_info_manager::CameraInfoManager info_mgr_right;

			image_transport::Publisher pub, pub_left, pub_right, pub_concat;
			ros::Publisher info_pub;
			ros::Publisher info_pub_left;
			ros::Publisher info_pub_right;    
			ros::Publisher exposure_pub;
			ros::Publisher brightness_pub;
			ros::Publisher IMU_pub;

			ros::Subscriber time_sub;
			ros::Subscriber exposure_sub;
			ros::Subscriber brightness_sub;

			ros::Time last_time;
			boost::mutex time_mutex_;

			uvc_cam::Cam *cam;
			boost::thread image_thread;
			boost::thread IMU_thread;

			double angleX, angleY, angleZ; // Rotational angle for cube [NEW]
			double RwEst[3];
			//	double glIMU_Interval;

			double squared(double x);
			double glIMU_Interval;

			void callBackExposure(std_msgs::Float64 call_exposure_value);
			void callBackBrightness(std_msgs::Float64 call_brightness_value);
			void SetIMUConfigDefault();
			void IMU_enable();    
			int econ_strcmp (const char * str1, const char *str2);
			/*  Returns the interval time for sampling the values of the IMU. */
			double GetIMUIntervalTime(IMUCONFIG_TypeDef	lIMUConfig);
			BOOL DisableIMU();

	};

};

