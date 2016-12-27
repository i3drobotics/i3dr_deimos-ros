see3cam
=======

ROS driver for the e-consystems See3CAM_Stereo (Tara) camera based on the 
[uvc_camera](https://github.com/ktossell/camera_umd/tree/master/uvc_camera) package.

Creates a stereo image node pair (`left/image_raw` and `right/image_raw`) from any connected Tara devices. This is for compatibility with any other ROS modules which support stereo cameras.
Added support for setting and getting exposure and brightness.
Added support for getting IMU data.

The following nodes will be created upon launching this driver.
```
    /stereo/concat
    /stereo/image_raw
    /stereo/left/image_raw
    /stereo/left/camera_info
    /stereo/right/image_raw
    /stereo/right/camera_info
    /stereo/get_brightness
    /stereo/set_brightness
    /stereo/get_exposure
    /stereo/set_exposure
    /stereo/get_IMU
```

The Camera preview can be seen using any basic ROS camera application. `rqt_image_view` can be used for simplicity.
To Install and use `rqt_image_view` 
```bash
sudo apt-get install ros-jade-rqt-image-view
rqt_image_view
```

Some Tested Examples
====================

* To check the exposure of the camera at run time:

```bash
rostopic echo /stereo/get_exposure
```
    
* To check the brightness of the camera at run time:

```bash
rostopic echo /stereo/get_brightness
```
    
* To change the exposure of the camera at run time:

```bash
rostopic pub -1 /stereo/set_exposure std_msgs/Float64 "data: <value>"
```

e.g. :

```bash
rostopic pub -1 /stereo/set_exposure std_msgs/Float64 "data: 20000"
```

* To change the brightness of the camera at run time:

```bash
rostopic pub -1 /stereo/set_brightness std_msgs/Float64 "data: <value>"
```

e.g. :

```bash
rostopic pub -1 /stereo/set_brightness std_msgs/Float64 "data: 6"
```

* To read the X,Y,Z co-ordinates of the camera using the built-in IMU:

```bash
rostopic echo /stereo/get_IMU 
```

Known Issues
============

* The directory which stores the camera config files (yaml) has to be created manually for the first time after driver installation (catkin_make).

```bash
mkdir ~/.ros/camera_info -p 
```
