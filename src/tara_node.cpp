#include <ros/ros.h>
#include <nodelet/loader.h>

#include "uvc_camera/tara_ros.h"

int main (int argc, char **argv) {
  ros::init(argc, argv, "tara_ros");

  uvc_camera::taraCamera camera(ros::NodeHandle(), ros::NodeHandle("~"));

cout << "\n\t\tin tara_node.cpp " << endl;
//camera.IMU_function();
cout << "\n\t\tin tara_node.cpp " << endl;

  ros::spin();
  return 0;
}

