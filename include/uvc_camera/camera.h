#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "uvc_cam/uvc_cam.h"
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include "std_msgs/Time.h"

namespace uvc_camera {

class Camera {
  public:
    Camera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh);
    void onInit();
    void sendInfo(sensor_msgs::ImagePtr &image, ros::Time time);
    void feedImages();
    ~Camera();

    void timeCb(std_msgs::Time time);

  private:
    ros::NodeHandle node, pnode;
    image_transport::ImageTransport it;
    bool ok;

    int width, height, fps, skip_frames, frames_to_skip;
    std::string device, frame;
    bool rotate;

    camera_info_manager::CameraInfoManager info_mgr;

    image_transport::Publisher pub;
    ros::Publisher info_pub;
    ros::Publisher exposure_pub;

    ros::Subscriber time_sub;

    ros::Time last_time;
    boost::mutex time_mutex_;

    uvc_cam::Cam *cam;
    boost::thread image_thread;
};

};

