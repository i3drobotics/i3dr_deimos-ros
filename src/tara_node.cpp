#include <ros/ros.h>
#include <nodelet/loader.h>

#include "uvc_camera/tara_ros.h"

int main (int argc, char **argv)
{
	ros::init(argc, argv, "tara_ros");
	  	
	uvc_camera::taraCamera camera(ros::NodeHandle(), ros::NodeHandle("~"));
	if ( camera.isCameraStereo == false )
	{
		ros::shutdown();
	}
	else
	{  
		ros::spin();
	}
	return 0;
}

