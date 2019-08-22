# Deimos (Beta)

WARNING: This is the beta version with development and unstable changes. Only download this if you know what you are doing.

ROS package for i3D Robotics' Deimos stereo camera, derived from the e-consystems See3CAM_Stereo (Tara) camera driver. This driver is in turn based on the [uvc_camera](https://github.com/ktossell/camera_umd/tree/master/uvc_camera) package. We have made significant improvements to the driver which make it a lot more usable in ROS.

## Installation

You need a few dependencies, including `v4l2`:

```bash
sudo apt-get install libv4l-dev v4l-utils qv4l2 v4l2ucp
```

Copy udev rules for usb permission:

```bash
sudo cp udev/99-uvc.rules /etc/udev/rules.d/
```

This package relies on the **i3dr_stereo_camera** package for generating of 3D and urdfs. Please make sure you have the **i3dr_stereo_camera** package in your workspace.
[link](https://github.com/i3drobotics/i3dr_stereo_camera-ros.git)

And some ROS bits which you may not already have:

```bash
sudo apt install ros-kinetic-joint-state-publisher ros-kinetic-stereo-image-proc ros-kinetic-robot-state-publisher ros-kinetic-xacro
```

Or you can run `rosdep install deimos` which should pull things from the package file.

Add the package to your workspace as usual, build with catkin, source your `devel/setup.bash` file and then run:

roslaunch deimos deimos.launch

This package has been tested informally on Ubuntu 16.04, but should run on most distros. It has also been tested on the Jetson TX1 and TX2.

## Introduction

Creates a stereo image node pair (`left/image_raw` and `right/image_raw`) from any connected Deimos/See3Cam device. This is for compatibility with any other ROS modules which support stereo cameras. This driver has been modified to respect ROS image pipeline conventions and to ease use of the IMU. As per convention, both left and right images are published in the left camera optical frame. IMU data are published in the `imu_link` frame on the `imu_data` topic.

The following nodes will be created upon launching this driver.

``` bash
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
    /stereo/imu_data
```

The Camera preview can be seen using any basic ROS camera application. `rqt_image_view` can be used for simplicity.
To Install and use `rqt_image_view` 

```bash
sudo apt-get install ros-jade-rqt-image-view
rqt_image_view
```

## Including Deimos in your project

Take a look at the `standalone.urdf` file for how to add a Deimos camera to your own project. There is an Xacro macro which you can call to insert an instance of the camera, you can then add a joint via the left mounting point to wherever you've physically attached it.

You can use the `./launch/deimos.launch` launch file as a base to go from. Typically you'll want to namespace the camera (nominally `/stereo`) and run your stereo matching node inside this namespace.

## Frames

This package has been updated to conform to the ROS image pipeline frame conventions.

Image conventions in ROS can be a bit confusing, as computer vision usually defines the Z axis to point out of the image, wheras in ROS Z is normally upwards. For this reason cameras define a separate `optical` frame which is rotated. Note that also both images are published in the left optical frame. This is because typically when using a stereo camera, the right image is effectively ignored. The point cloud is derived and intensity mapped using data from the left camera.

You can see in the URDF file for Deimos - there are two links for each camera. The `cameraLeft` and `cameraRight` links are the nominal locations of the image sensors, but the `_optical` links are rotated in X and Z. This means that when you view the point cloud in RVIZ it should appear the right way up.

## TF tree

The root frame is `deimos_left_mount` which is the location of the centre M4 mounting hole on the base of the unit, under the left camera. The reference point is on the face of the unit.

The full tree for a standalone Deimos system is shown below:

![Deimos TF tree](doc/deimos_tf_tree.PNG)

## Calibration

You can calibrate the camera using ROS' camera calibration tool. In one terminal window, run the basic launch file:

```bash
roslaunch deimos deimos.launch
```

And in another:

```bash
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 right:=/stereo/right/image_raw left:=/stereo/left/image_raw left_camera:=/stereo/left right_camera:=/stereo/right
```

You should see something like:

```bash
('Waiting for service', '/stereo/left/set_camera_info', '...')
OK
('Waiting for service', '/stereo/right/set_camera_info', '...')
OK
```

And the calibration window will appear. You should edit the parameters above to correspond to your calibration board.

See [link](http://wiki.ros.org/camera_calibration) for details.

You can provide a name for the cameras by adding to your `deimos_node`, for example:

```xml
    <param name="leftCameraName" type="string" value="left" />
    <param name="rightCameraName" type="string" value="right" />
```

## Some Tested Examples

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

* To read the inclination of the camera using the built-in IMU:

```bash
rostopic echo /stereo/get_inclination
```

* To read the angular velocity and linear acceleration of the camera using built-in IMU:

```bash
rostopic echo /stereo/get_IMU
```

## Known Issues

* The directory which stores the camera config files (yaml) has to be created manually for the first time after driver installation (catkin_make).

```bash
mkdir ~/.ros/camera_info -p
```
