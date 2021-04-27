#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#include <cmath>
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdint.h>
#include "Local.h"
#include <stdio.h>
#include "pinhole_projection/camera_pose_and_info.h"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <time.h>
#include <iostream>
#include <iterator>
#include <opencv2/opencv.hpp>
#include "Tracking.h"
#include <vector>
#include <geometry_msgs/Point.h>
#include <parabolicMask.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <pcl_ros/transforms.h>

using namespace cv;
using namespace std;
using namespace pinhole_projection;
using namespace sensor_msgs;
#define PI 3.1415926

	
class Processing
{
private:
	const float roiRatio = 0.7;
	const int thresholdingQ = 990;//990   750
  cv::Mat  imgOrigin, imgIPM32,imgIPM35, imgIPMInv32, imgROIGray, imgmask, imgGaussian, imgThreshold, tsfIPM, tsfIPMInv, gaussianKernelX, gaussianKernelY, gaussianKernel, XX, YY, imgBackIPM32, imgBackIPMInv32, imgFinal,imgMix;
  cv::Rect roiLane;
	ros::Publisher final_pub, IPM_pub, marking_pub, association_pub, marker_pub, gaussian_pub, graph_pub, histogram_pub, mask_pub, cloud_pub, draw_pub;
	ros::Subscriber sub_image, sub_dl;
	std::string path;
	sensor_msgs::ImagePtr outputIPM, outputmarking, outputassociation, outputgaussian, outputhistogram, outputgraph, outputmask;
	visualization_msgs::MarkerArray features;
  std::vector<cv::Point2d> lanemark, dlmark;
	int c;
  ros::Time msgtime;
	double rotate, camera_height, camera_pitch;
	bool fileFirst, sy_flag, show_img, mode_flag;
	Tracking lt = Tracking();
  pinhole_projection::CameraPoseAndInfo* camera_info;
  ros::NodeHandle m_n;
  
public:
  string camera_name;
  cv::Mat outputmap;
  pcl::PointCloud<pcl::PointXYZ>::Ptr sur_cloud;
	Processing(ros::NodeHandle n, string i) {
		Processing::onGaussianChange(0, NULL);
		Processing::onROIChange(0, NULL);
    //n.getParam("camera_name", camera_name);
    n.getParam("debug", show_img);
    m_n = n;
    n.getParam("camera_name_"+i, camera_name);
    sub_image = n.subscribe(camera_name+"/h265/rgb8/compressed", 1, &Processing::callback1, this);
    sub_dl = n.subscribe("erfnet_tensorrt/"+camera_name+"/Points", 1, &Processing::callback2, this);
    //path = ros::package::getPath("laneitri");
    camera_info = new pinhole_projection::CameraPoseAndInfo(n, camera_name);
    mode_flag = 0;
    camera_height = 1.8;
    fileFirst = 1, sy_flag = 0;
    rotate = 0;
    sur_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
    marker_pub = n.advertise<visualization_msgs::MarkerArray>(camera_name+"/visualization_marker", 10);
    final_pub = n.advertise<sensor_msgs::Image> (camera_name+"/final", 10);                                                                  
    mask_pub = n.advertise<sensor_msgs::Image> (camera_name+"/mask", 10);
    IPM_pub = n.advertise<sensor_msgs::Image> (camera_name+"/IPM", 10);
    marking_pub = n.advertise<sensor_msgs::Image> (camera_name+"/marking", 10);
    association_pub = n.advertise<sensor_msgs::Image> (camera_name+"/association", 10);
    gaussian_pub = n.advertise<sensor_msgs::Image> (camera_name+"/gaussian", 10);
    draw_pub = n.advertise<sensor_msgs::Image> (camera_name+"/draw", 10);
    histogram_pub = n.advertise<sensor_msgs::Image> (camera_name+"/histogram", 10);
    graph_pub = n.advertise<sensor_msgs::Image> (camera_name+"/graph", 1);
    cloud_pub = n.advertise<sensor_msgs::PointCloud2>(camera_name+"/cloud", 10);
    if (show_img) {debug();}
    
	}
  ~Processing(){};
  void debug();
  void onGaussianChange(int _x, void *_ptr);
  void onROIChange(int _x, void *ptr);
  void callback1(const sensor_msgs::CompressedImageConstPtr& msg);
  void callback2(const std_msgs::Int16MultiArrayConstPtr& msg);
  void match(std::vector<cv::Point2d> input, ros::Time msg_time, pcl::PointCloud<pcl::PointXYZ>::Ptr lane_cloud);
  cv::Mat detectLanePixels(cv::Mat imgInput, double rotate);
  cv::Mat RotateMatrix(cv::Mat input, int degree);
  pcl::PointCloud<pcl::PointXYZ>::Ptr get_features();
  void clear_features();
  ros::Time get_msg_time();
  Eigen::Vector3d IPM(cv::Point2d);
  void draw_points_on_image(pcl::PointCloud<pcl::PointXYZ>::Ptr lane_cloud,
                            Eigen::Matrix4d vehicle_pose);
};