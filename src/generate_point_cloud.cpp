#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <stereo_msgs/DisparityImage.h>

ros::Publisher point_cloud_pub;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PointCloudNormal;

//Get ROS disparity msg from stereo camera and send to processing
void disparityCallback(const stereo_msgs::DisparityImageConstPtr& msg)
{

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "generate_point_cloud");
  ros::NodeHandle nh;
  ros::NodeHandle pnode("~");

  std::string disparityNode,pointCloudNode;
  pnode.getParam("disparity_node",disparityNode);
  pnode.getParam("point_cloud_node",pointCloudNode);

  //subscribe to disparity node
  ros::Subscriber disparity_sub = nh.subscribe(disparityNode, 1, disparityCallback);

  point_cloud_pub = nh.advertise<PointCloudNormal>(pointCloudNode, 1);

  ros::spin();
}