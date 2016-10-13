see3cam
=======

ROS driver for the e-consystems See3CAM_Stereo (Tara) camera based on the 
[uvc_camera](https://github.com/ktossell/camera_umd/tree/master/uvc_camera) package.

Creates a streo image node pair (left/image_raw and right/image_raw) from any conected Tara devices. This is for compatibility with any other ROS modules which support stero cameras.
Added support for setting,getting exposure and brightness.

The following nodes will be created upon launching this driver.
	/stereo/concat
	/stereo/image_raw
	/stereo/left/image_raw
	/stereo/left/camera_info
	/stereo/right/image_raw
	/stereo/right/camera_info


The Camera preview can be seen using any basic ROS camera application. rqt_image_view can be used for simplicity.
To Install and use rqt_image_view 
	$ sudo apt-get install ros-jade-rqt-image-view
	$ rqt_image_view


Some Tested Examples
====================

1. To check the exposure of the camera at run time :
	$ rostopic echo /stereo/exposure
	
2. To check the brightness of the camera at run time :
	$ rostopic echo /stereo/brightness
	
3. To change the exposure of the camera at run time :
	$ rostopic pub -1 /stereo/set_exposure std_msgs/Float64 "data: <value>"
		e.g. :
			$ rostopic pub -1 /stereo/set_exposure std_msgs/Float64 "data: 20000"

4. To change the brightness of the camera at run time :
	$ rostopic pub -1 /stereo/set_brightness std_msgs/Float64 "data: <value>"
		e.g. :
			$ rostopic pub -1 /stereo/set_brightness std_msgs/Float64 "data: 6" 
