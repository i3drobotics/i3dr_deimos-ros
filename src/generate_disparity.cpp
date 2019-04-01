#include <ros/ros.h>
#include <ros/console.h>
#include <ros/param.h>

#include <boost/bind.hpp>

#include "std_msgs/String.h"
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/ximgproc.hpp>

#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

#include <dynamic_reconfigure/server.h>
#include <deimos/i3DR_DisparityConfig.h>

#include <boost/filesystem.hpp>

#include <string>

using namespace cv;

int CV_StereoBM = 0;
int CV_StereoSGBM = 1;
int JR_StereoSGBM = 2;
ros::Publisher _disparity_pub;
int _stereo_algorithm = CV_StereoBM;
int _min_disparity = 9;
int _disparity_range = 64;
int _correlation_window_size = 15;
int _uniqueness_ratio = 15;
int _texture_threshold = 10;
int _speckle_size = 100;
int _speckle_range = 4;

typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image>
    policy_t;

//Calculate disparity using left and right images
Mat stereo_match(Mat left_image, Mat right_image, int algorithm, int min_disparity, int disparity_range, int correlation_window_size, int uniqueness_ratio, int texture_threshold, int speckleSize, int speckelRange)
{

    cv::Mat disp, disparity_rl, disparity_filter;
    cv::Size image_size = cv::Size(left_image.size().width, left_image.size().height);
    // Setup for 16-bit disparity
    cv::Mat(image_size, CV_16S).copyTo(disp);
    cv::Mat(image_size, CV_16S).copyTo(disparity_rl);
    cv::Mat(image_size, CV_16S).copyTo(disparity_filter);

    if (disparity_range < 1 || disparity_range % 16 != 0)
    {
        ROS_ERROR("disparity_range must be a positive integer divisible by 16");
        return disp;
    }

    //disparity_range = disparity_range > 0 ? disparity_range : ((left_image.size().width / 8) + 15) & -16;

    if (algorithm == CV_StereoBM)
    {
        cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(64, 9);

        bm->setPreFilterCap(31);
        bm->setPreFilterSize(15);
        bm->setPreFilterType(1);
        bm->setBlockSize(correlation_window_size > 0 ? correlation_window_size : 9);
        bm->setMinDisparity(min_disparity);
        bm->setNumDisparities(disparity_range);
        bm->setTextureThreshold(texture_threshold);
        bm->setUniquenessRatio(uniqueness_ratio);
        bm->setSpeckleWindowSize(speckleSize);
        bm->setSpeckleRange(speckelRange);
        bm->setDisp12MaxDiff(-1);

        bm->compute(left_image, right_image, disp);

        auto right_matcher = cv::ximgproc::createRightMatcher(bm);
        right_matcher->compute(right_image, left_image, disparity_rl);

        double wls_lambda = 8000;
        double wls_sigma = 1.5;
        cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(bm);
        wls_filter->setLambda(wls_lambda);
        wls_filter->setSigmaColor(wls_sigma);
        wls_filter->filter(disp, left_image, disparity_filter, disparity_rl);
    }
    else if (algorithm == CV_StereoSGBM)
    {
        cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(64, 9);

        sgbm->setPreFilterCap(31);
        //sgbm->setP1(1);
        //sgbm->setP2(1);
        sgbm->setBlockSize(correlation_window_size > 0 ? correlation_window_size : 9);
        sgbm->setMinDisparity(min_disparity);
        sgbm->setNumDisparities(disparity_range);
        sgbm->setUniquenessRatio(uniqueness_ratio);
        sgbm->setSpeckleWindowSize(speckleSize);
        sgbm->setSpeckleRange(speckelRange);
        sgbm->setDisp12MaxDiff(-1);

        sgbm->compute(left_image, right_image, disp);

        auto right_matcher = cv::ximgproc::createRightMatcher(sgbm);
        right_matcher->compute(right_image, left_image, disparity_rl);

        double wls_lambda = 8000;
        double wls_sigma = 1.5;
        cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(sgbm);
        wls_filter->setLambda(wls_lambda);
        wls_filter->setSigmaColor(wls_sigma);
        wls_filter->filter(disp, left_image, disparity_filter, disparity_rl);
    }
    disp.convertTo(disp, CV_32F);
    disparity_filter.convertTo(disparity_filter, CV_32F);
    return disp;
}

void publish_disparity(cv::Mat disparity, int min_disparity, int disparity_range)
{
    stereo_msgs::DisparityImage disp_msg;
    disp_msg.min_disparity = min_disparity;
    disp_msg.max_disparity = disparity_range;

    // should be safe
    disp_msg.valid_window.x_offset = 0;
    disp_msg.valid_window.y_offset = 0;
    disp_msg.valid_window.width = 0;
    disp_msg.valid_window.height = 0;
    disp_msg.T = 0;
    disp_msg.f = 0;
    disp_msg.delta_d = 0;
    disp_msg.header.stamp = ros::Time::now();
    disp_msg.header.frame_id = ros::this_node::getName();

    sensor_msgs::Image &dimage = disp_msg.image;
    dimage.width = disparity.size().width;
    dimage.height = disparity.size().height;
    dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    dimage.step = dimage.width * sizeof(float);
    dimage.data.resize(dimage.step * dimage.height);
    cv::Mat_<float> dmat(dimage.height, dimage.width, reinterpret_cast<float *>(&dimage.data[0]), dimage.step);

    disparity.convertTo(dmat, dmat.type());

    _disparity_pub.publish(disp_msg);
}

void parameterCallback(deimos::i3DR_DisparityConfig &config, uint32_t level) {
    config.prefilter_size |= 0x1; // must be odd
    config.correlation_window_size |= 0x1; //must be odd
    config.disparity_range = (config.disparity_range / 16) * 16; // must be multiple of 16

    _stereo_algorithm = config.stereo_algorithm;
    _correlation_window_size = config.correlation_window_size;
    _min_disparity = config.min_disparity;
    _disparity_range = config.disparity_range;
    _uniqueness_ratio = config.uniqueness_ratio;
    _texture_threshold = config.texture_threshold;
    _speckle_size = config.speckle_size;
    _speckle_range = config.speckle_range;
}

void stereoImageCallback(const sensor_msgs::ImageConstPtr &msg_left, const sensor_msgs::ImageConstPtr &msg_right)
{
    try
    {
        cv_bridge::CvImagePtr input_image_left, input_image_right;
        input_image_left = cv_bridge::toCvCopy(msg_left, "mono8");
        input_image_right = cv_bridge::toCvCopy(msg_right, "mono8");

        Mat disp = stereo_match(input_image_left->image, input_image_right->image, 0, _min_disparity, _disparity_range, _correlation_window_size, _uniqueness_ratio, _texture_threshold, _speckle_size, _speckle_range);
        publish_disparity(disp,_min_disparity,_disparity_range);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg_right->encoding.c_str());
        ROS_ERROR("or Could not convert from '%s' to 'mono8'.", msg_left->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "generate_disparity");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    int stereo_algorithm, min_disparity, disparity_range, correlation_window_size, uniqueness_ratio, texture_threshold, speckle_size, speckle_range;

    //Get parameters
    if (p_nh.getParam("stereo_algorithm", stereo_algorithm))
    {
        _stereo_algorithm = stereo_algorithm;
        ROS_INFO("stereo_algorithm: %d", _stereo_algorithm);
    }
    if (p_nh.getParam("min_disparity", min_disparity))
    {
        _min_disparity = min_disparity;
        ROS_INFO("min_disparity: %d", _min_disparity);
    }
    if (p_nh.getParam("disparity_range", disparity_range))
    {
        _disparity_range = disparity_range;
        ROS_INFO("disparity_range: %d", _disparity_range);
    }
    if (p_nh.getParam("correlation_window_size", correlation_window_size))
    {
        _correlation_window_size = correlation_window_size;
        ROS_INFO("correlation_window_size: %d", _correlation_window_size);
    }
    if (p_nh.getParam("uniqueness_ratio", uniqueness_ratio))
    {
        _uniqueness_ratio = uniqueness_ratio;
        ROS_INFO("uniqueness_ratio: %d", _uniqueness_ratio);
    }
    if (p_nh.getParam("texture_threshold", texture_threshold))
    {
        _texture_threshold = texture_threshold;
        ROS_INFO("texture_threshold: %d", _texture_threshold);
    }
    if (p_nh.getParam("speckle_size", speckle_size))
    {
        _speckle_size = speckle_size;
        ROS_INFO("speckle_size: %d", _speckle_size);
    }
    if (p_nh.getParam("speckle_range", speckle_range))
    {
        _speckle_range = speckle_range;
        ROS_INFO("speckle_range: %d", _speckle_range);
    }

    std::string ns = ros::this_node::getNamespace();

    // Dynamic parameters
    dynamic_reconfigure::Server<deimos::i3DR_DisparityConfig> server;
    dynamic_reconfigure::Server<deimos::i3DR_DisparityConfig>::CallbackType f;
    f = boost::bind(&parameterCallback, _1, _2);
    server.setCallback(f);

    // Publishers creation
    _disparity_pub = nh.advertise<stereo_msgs::DisparityImage>(ns+"/i3dr_disparity",1,true);

    // Subscribers creation.
    message_filters::Subscriber<sensor_msgs::Image> sub_l(nh, ns+"/left/image_rect", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_r(nh, ns+"/right/image_rect", 1);

    // Message filter creation.
    message_filters::Synchronizer<policy_t> sync(policy_t(10), sub_l, sub_r);
    sync.registerCallback(boost::bind(&stereoImageCallback, _1, _2));

    ros::spin();
}