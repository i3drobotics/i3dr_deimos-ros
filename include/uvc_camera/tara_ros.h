#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "uvc_cam/uvc_cam.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include "std_msgs/Time.h"
#include "std_msgs/Float64.h"
#include <libv4l2.h>
#include "camera_calibration_parsers/parse_yml.h"
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <string.h>
using namespace std;

namespace uvc_camera {

class taraCamera {
  public:
    taraCamera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh);
    void onInit();
    void sendInfo(sensor_msgs::ImagePtr &image, ros::Time time);
    void sendInfoLeft(sensor_msgs::ImagePtr &image, ros::Time time);
    void sendInfoRight(sensor_msgs::ImagePtr &image, ros::Time time);
    void feedImages();
    ~taraCamera();
    void timeCb(std_msgs::Time time);
    void callBackExposure(std_msgs::Float64 call_exposure_value);
    void callBackBrightness(std_msgs::Float64 call_brightness_value);
   BOOL LoadCameraMatrix();
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
    ros::Subscriber time_sub;
    ros::Subscriber exposure_sub;
     ros::Subscriber brightness_sub;

    ros::Time last_time;
    boost::mutex time_mutex_;

    uvc_cam::Cam *cam;
    boost::thread image_thread;
};

};

