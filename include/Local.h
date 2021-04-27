#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/Imu.h"
#include "string.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2_msgs/TFMessage.h"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <eigen_conversions/eigen_msg.h>
#include "pinhole_projection/camera_pose_and_info.h"

#include <random>
#include <json/value.h>
#include <jsoncpp/json/json.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <opencv2/opencv.hpp>
#include "ros/time.h"
#include <cv_bridge/cv_bridge.h>
#include <string.h>
#include <iostream>
#include "tf_conversions/tf_eigen.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen3/Eigen/Dense>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <laneitri/VelodyneTelemetry.h>
class Local{
private: 
    
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    tf::TransformBroadcaster br;
    Eigen::Matrix4d icp_transformed, ego_vehicle, gps_matrix;
    Eigen::Matrix3d camera_intrin;
    tf::Transform transformed, transform, base2velo, base2baja, came2velo, base2came;//define axie shift; 
    geometry_msgs::TransformStamped temp_trans;
    bool initial_flag, gps_flag;
    ros::Publisher map_pub, right_pub, left_pub, sub_map_pub, ori_pub;
    ros::Subscriber gps_sub, sub_lidar;
    //std::vector<cv::Point2d> readmap; 
    nav_msgs::Odometry current_gps;
    double camera_pitch, camera_height;
    geometry_msgs::Pose origin;
    Eigen::Vector3d orien;
    laneitri::VelodyneTelemetry previ;
    Eigen::Vector3d forward, last_position;
    std::string name;
    pinhole_projection::CameraPoseAndInfo* camera_info;
    ros::Time tf_time;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mapp, lidar_points;
public:
    Eigen::Matrix4d initial_guess;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sub_map;
    Local(ros::NodeHandle n){//}, std::string camera_name, pinhole_projection::CameraPoseAndInfo* camera_inf) {
      initial_flag = 0, gps_flag = 0;
      //name = camera_name;
      map_pub = n.advertise<sensor_msgs::PointCloud2>("semantic_map", 100);
      sub_map_pub = n.advertise<sensor_msgs::PointCloud2>("sub_map", 100);
      //camera_info = camera_inf;
      right_pub = n.advertise<sensor_msgs::PointCloud2>("right", 1000);
      left_pub = n.advertise<sensor_msgs::PointCloud2> ("left", 1000);
      ori_pub = n.advertise<sensor_msgs::PointCloud2> (name+"ori", 1000);
      right_pub = n.advertise<sensor_msgs::PointCloud2>("right", 1000);
      //sub_lidar = n.subscribe("/", 100, &Local::lane_callback, this);
      sub_lidar = n.subscribe("/velodyne_points", 100, &Local::lidar_callback, this);
      gps_sub = n.subscribe("/velodyne_telemetry", 100, &Local::gps_callback, this);
      current_gps.pose.pose.position.x = 0;
      current_gps.pose.pose.position.y = 0;
      current_gps.pose.pose.position.z = 0;
      current_gps.pose.pose.orientation.x = 0;
      current_gps.pose.pose.orientation.y = 0;
      current_gps.pose.pose.orientation.z = 0;
      current_gps.pose.pose.orientation.w = 1;
      icp_transformed = Eigen::Matrix4d::Identity(4, 4);
      ego_vehicle = Eigen::Matrix4d::Identity(4, 4);
      initial_guess = Eigen::Matrix4d::Identity(4, 4);
      //get_tf(n);
      sub_map = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
      lidar_points = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
      mapp = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
      read_map();
    }
    ~Local() {}
    void tf_broadcast(Eigen::Matrix4d input, std::string frame_name, ros::Time msgtime);
    //void get_tf(ros::NodeHandle n);
    
    bool Setting_Initial_Guess(ros::Time msgtime);
    bool Setting_Initial_Guess_from_NDT(ros::Time msgtime, ros::NodeHandle n);
    void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void gps_callback(const laneitri::VelodyneTelemetry::Ptr msg);
    
    void ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr in1, ros::Time msgtime, ros::NodeHandle n);
    void read_map();
  
};