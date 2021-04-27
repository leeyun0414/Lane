//#pragma warning(disable: 4819)

#include "Local.h"
#define _USE_MATH_DEFINES
    
using namespace sensor_msgs;
using namespace geometry_msgs;

double pointdistance(pcl::PointXYZ a, pcl::PointXYZ b) {
  double x = (a.x - b.x) * (a.x - b.x);
  double y = (a.y - b.y) * (a.y - b.y);
  double z = (a.z - b.z) * (a.z - b.z);
  return sqrt(x + y + z);
}
tf::Quaternion spherical_to_Q(double phi, double theta){
  tf::Quaternion qy(0.*sin(.5*theta),-1.0*sin(.5*theta),0.*sin(.5*theta),cos(.5*theta));
  tf::Quaternion qz(0.*sin(.5*phi),0.*sin(.5*phi),1.*sin(.5*phi),cos(.5*phi));
  return qz*qy;
}
geometry_msgs::Point geodetic_to_ecef(double lat, double lon, double alt) {
  double R = 6378.137;
  double f = 1/298.257223563;
  double c = 1. - f;
  double Lat = lat * M_PI / 180;
  double Lon = lon * M_PI / 180;
  double c_lat_g = cos(Lat);
  double s_lat_g = sin(Lat);
  double c_lon = cos(Lon);
  double s_lon = sin(Lon);
  double c_lat_p = c_lat_g / sqrt(c_lat_g*c_lat_g + c*c * s_lat_g*s_lat_g);
  double s_lat_p = c * s_lat_g / sqrt(c_lat_g*c_lat_g + c*c * s_lat_g*s_lat_g);
  double x = R * c_lat_p * c_lon;
  double y = R * c_lat_p * s_lon;
  double z = R * c * s_lat_p;
  geometry_msgs::Point temp;
  temp.x = x;
  temp.y = y;
  temp.z = z;
  
  return temp;
}
geometry_msgs::Pose make_LTP(double lat, double lon){
  geometry_msgs::Point P = geodetic_to_ecef(lat, lon, 0);
  tf::Quaternion Q = spherical_to_Q(lon/180.*M_PI, lat/180.*M_PI);
  double theta, phi;
  theta = M_PI/2.;
  phi = M_PI/2.;
  tf::Quaternion a(0.*sin(.5*theta),1.0*sin(.5*theta),0.*sin(.5*theta),cos(.5*theta));
  tf::Quaternion b(0.*sin(.5*phi),0.*sin(.5*phi),1.*sin(.5*phi),cos(.5*phi));
  Q = Q * a * b;
  geometry_msgs::Pose temp;
  temp.position = P;
  temp.orientation.x = Q.x();
  temp.orientation.y = Q.y();
  temp.orientation.z = Q.z();
  temp.orientation.w = Q.w();
  return temp;

}
void Local::gps_callback(const laneitri::VelodyneTelemetry::Ptr msg) {
  laneitri::VelodyneTelemetry input;
  if (!gps_flag) {
    previ = *msg;
    origin = make_LTP(msg->gprmc.lat, msg->gprmc.lon);

    gps_flag = 1;
    return;
  }
  if (previ.gprmc.lat == msg->gprmc.lat && previ.gprmc.lon == msg->gprmc.lon)
    return;
  geometry_msgs::Pose gps_pose = make_LTP(msg->gprmc.lat, msg->gprmc.lon);
  //gps_pose = geodetic_dis(origin.gprmc.lat, origin.gprmc.lon, input.gprmc.lat, input.gprmc.lon);
  geometry_msgs::Point temp;
  temp.x = gps_pose.position.x - origin.position.x;
  temp.y = gps_pose.position.y - origin.position.y;
  temp.z = gps_pose.position.z - origin.position.z;
  Eigen::Vector3d tans;
  Eigen::Quaterniond rot;
  tf::pointMsgToEigen(temp, tans);
  tf::quaternionMsgToEigen(origin.orientation, rot);
  tans =  rot.inverse() * tans ;
  current_gps.pose.pose.position.x = tans[0] * 1000.;
  current_gps.pose.pose.position.y = tans[1] * 1000.;
  current_gps.pose.pose.position.z = tans[2] * 1000.;
  double theta = (90. - msg->gprmc.track) * M_PI / 180.;
  tf::Quaternion yaw(0.*sin(.5*theta),0.*sin(.5*theta),1.*sin(.5*theta),cos(.5*theta));
  theta = 0.;
  tf::Quaternion pitch(1.*sin(.5*theta),0.*sin(.5*theta),0.*sin(.5*theta),cos(.5*theta));
  tf::Quaternion roll(0.*sin(.5*theta),1.*sin(.5*theta),0.*sin(.5*theta),cos(.5*theta));
  tf::Quaternion Q_rpy = yaw * pitch * roll;
  geometry_msgs::Quaternion Q_Rpy, qq;
  Q_Rpy.x = Q_rpy.x();Q_Rpy.y = Q_rpy.y();Q_Rpy.w = Q_rpy.w();Q_Rpy.z = Q_rpy.z();
  Eigen::Quaterniond Q_RPY;
  tf::quaternionMsgToEigen(Q_Rpy, Q_RPY);
  Eigen::Quaterniond Q_LTP;
  tf::quaternionMsgToEigen(gps_pose.orientation, Q_LTP);
  Eigen::Quaterniond Q = rot.inverse() * Q_LTP * Q_RPY;
  tf::quaternionEigenToMsg(Q, qq);
  current_gps.pose.pose.orientation = qq;
  std::cout << current_gps.pose.pose;
  previ = *msg;
}

void Local::lidar_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
	pcl::fromROSMsg (*msg, *lidar_points);
}


Eigen::Matrix4d tftoeigen(tf::Transform input) {
  Eigen::Matrix4d output;
	Eigen::Matrix3d rotation_matrix;
  tf::Matrix3x3 rotation_tf_matrix;
	tf::Quaternion Quaternion_temp;
	geometry_msgs::Quaternion c2l;
	c2l.x = input.getRotation()[0];
	c2l.y = input.getRotation()[1];
	c2l.z = input.getRotation()[2];
	c2l.w = input.getRotation()[3];
	quaternionMsgToTF(c2l, Quaternion_temp);
	rotation_tf_matrix.setRotation(Quaternion_temp); //myQuaternion
	tf::matrixTFToEigen(rotation_tf_matrix, rotation_matrix);
	output(0,0) = rotation_matrix(0,0);
	output(0,1) = rotation_matrix(0,1);
	output(0,2) = rotation_matrix(0,2);
	output(1,0) = rotation_matrix(1,0);
	output(1,1) = rotation_matrix(1,1);
	output(1,2) = rotation_matrix(1,2);
	output(2,0) = rotation_matrix(2,0);
	output(2,1) = rotation_matrix(2,1);
	output(2,2) = rotation_matrix(2,2);
	output(0,3) = input.getOrigin()[0];
	output(1,3) = input.getOrigin()[1];
	output(2,3) = input.getOrigin()[2];
  return output;
}

void Local::ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr in1,
                ros::Time msgtime,
                ros::NodeHandle n
                ){
  msgtime = tf_time;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut_new(new pcl::PointCloud<pcl::PointXYZ>);
  bool gps_in = Setting_Initial_Guess_from_NDT(msgtime, n);

  double gps_cov = 25.0;//25
  sub_map->points.clear();
  sub_map->width = 0;
  sub_map->height = 0;
  
  for(size_t i=0; i<mapp->points.size(); i++) {//make sub map 
    if (std::abs(mapp->points[i].x - initial_guess(0,3)) < gps_cov &&
        std::abs(mapp->points[i].y - initial_guess(1,3)) < gps_cov) {
      pcl::PointXYZ temp2;
      temp2.x = mapp->points[i].x;
      temp2.y = mapp->points[i].y;
      temp2.z = mapp->points[i].z;
      sub_map->points.push_back(temp2);
      //in1_base->width++;
      //in1_base->points.push_back(temp);	
    }
  }
  
  if (!gps_in) {
    //return;
    initial_guess = ego_vehicle;
  }
  ego_vehicle = initial_guess;
  Eigen::Matrix4f ego_vehicle_f = ego_vehicle.cast <float> ();
  tf_broadcast(ego_vehicle, "base_link_from_lane_initial", msgtime);
  if (!in1->points.size()) {
    tf_broadcast(ego_vehicle, "base_link_from_lane", msgtime);
  }
  else {
    bool icp_is_ok = false;
    while(!icp_is_ok) {
      
      /*
      icp.setInputSource(in1);
      icp.setInputTarget(sub_map);	
      icp.setMaxCorrespondenceDistance(100);
      icp.setTransformationEpsilon(1e-10);
      icp.setEuclideanFitnessEpsilon(1e-10);
      icp.setMaximumIterations (1000);
      icp.setUseReciprocalCorrespondences(true);
      icp.align (*cloudOut_new, ego_vehicle_f);
      std::cout << icp.getFitnessScore() << std::endl;
      ego_vehicle_f = icp.getFinalTransformation();
      ego_vehicle = ego_vehicle_f.cast<double>();
      //ego_vehicle = initial_guess;
      double suc_num = 0, fal_num = 0;
      for(size_t i=0; i<in1->points.size(); i++) {//try icp result
        Eigen::Vector4d lidarPoint(in1->points[i].x, in1->points[i].y, in1->points[i].z, 1);
        lidarPoint = ego_vehicle * lidarPoint;
        pcl::PointXYZ temp;
        temp.x = lidarPoint[0];
        temp.y = lidarPoint[1];
        temp.z = lidarPoint[2];
        double icp_thres = 1;
        bool suc_flag = 0;
        for (size_t i = 0; i < sub_map->points.size(); i++) {
          if (suc_flag)
            break;
          if (std::abs(sub_map->points[i].x - temp.x) < icp_thres &&
              std::abs(sub_map->points[i].y - temp.y) < icp_thres)
            suc_flag = 1;
        }
        if (suc_flag)
          suc_num++;
        else
        {
           fal_num++;
        } 
      }
      std::cerr<<"suc_num:" << suc_num << std::endl << "fal_num" << fal_num << std::endl;
      if (fal_num < suc_num * 0.5)
        icp_is_ok = true;
      else if(suc_num > fal_num){
        const double mean = 0.0;
        const double stddev = 3.0;
        std::default_random_engine generator;
        std::normal_distribution<double> dist(mean, stddev);
        ego_vehicle_f(0, 3) += dist(generator);
        ego_vehicle_f(1, 3) += dist(generator);
        std::cerr<< "do_ICP_AGAIN"<<std::endl;
        icp_is_ok = true;
      }
      else {
        icp_is_ok = true;//too bad to drop
      }
    }*/
    tf_broadcast(ego_vehicle, "base_link_from_lane", msgtime);
  }
  // Convert the pcl/PointCloud to sensor_msgs/PointCloud2
  sensor_msgs::PointCloud2 map_publish;
  sensor_msgs::PointCloud2 sub_publish;
  sensor_msgs::PointCloud2 ori_publish;
	
  pcl::toROSMsg(*mapp, map_publish);
  map_publish.header.frame_id = "/map";
  map_publish.header.stamp = msgtime;
  map_pub.publish(map_publish);
  pcl::toROSMsg(*in1, ori_publish);
  ori_publish.header.stamp = msgtime;
  ori_publish.header.frame_id = "/base_link_from_lane";
  ori_pub.publish(ori_publish);
  pcl::toROSMsg(*sub_map, sub_publish);
  sub_publish.header.frame_id = "/map";
  sub_publish.header.stamp = msgtime;
  sub_map_pub.publish(sub_publish);
}
bool Local::Setting_Initial_Guess(ros::Time msgtime)
{   	
	static nav_msgs::Odometry cu_pose;
  if (cu_pose.pose.pose.position.x == current_gps.pose.pose.position.x
      && cu_pose.pose.pose.position.y == current_gps.pose.pose.position.y
      && cu_pose.pose.pose.position.z == current_gps.pose.pose.position.z)
    return 0;
  cu_pose = current_gps;
  last_position << initial_guess(0,3), initial_guess(1,3), 0;
	initial_guess = Eigen::Matrix4d::Identity();
	    
  tf::Matrix3x3 rotation_tf_matrix;
	tf::Quaternion Quaternion_temp, lidar_base_temp;
	geometry_msgs::Quaternion c2l;
	c2l.x = base2velo.getRotation()[0];
	c2l.y = base2velo.getRotation()[1];
	c2l.z = base2velo.getRotation()[2];
	c2l.w = base2velo.getRotation()[3];
	//cu_pose.pose.pose.orientation *= lidar_to_camera;
	
	quaternionMsgToTF(cu_pose.pose.pose.orientation, Quaternion_temp);
	quaternionMsgToTF(c2l, lidar_base_temp);
	//myQuaternion1.setRPY(0, 0, i);
	Quaternion_temp = Quaternion_temp * lidar_base_temp;
	rotation_tf_matrix.setRotation(Quaternion_temp); //myQuaternion
  Eigen::Matrix3d rotation_matrix;
	tf::matrixTFToEigen(rotation_tf_matrix, rotation_matrix);
	initial_guess(0,0) = rotation_matrix(0,0);
	initial_guess(0,1) = rotation_matrix(0,1);
	initial_guess(0,2) = rotation_matrix(0,2);
	initial_guess(1,0) = rotation_matrix(1,0);
	initial_guess(1,1) = rotation_matrix(1,1);
	initial_guess(1,2) = rotation_matrix(1,2);
	initial_guess(2,0) = rotation_matrix(2,0);
	initial_guess(2,1) = rotation_matrix(2,1);
	initial_guess(2,2) = rotation_matrix(2,2);
	initial_guess(0,3) = cu_pose.pose.pose.position.x + base2velo.getOrigin()[0] + 5;//shift origin
	initial_guess(1,3) = cu_pose.pose.pose.position.y + base2velo.getOrigin()[1] - 14;
	initial_guess(2,3) = 0;//cu_pose.pose.pose.position.z + base2velo.getOrigin()[2] - 1.5;// - 11;		
  forward << initial_guess(0,3) - last_position[0], initial_guess(1,3)-last_position[1], 0;
  return 1;
}
bool Local::Setting_Initial_Guess_from_NDT(ros::Time msgtime, ros::NodeHandle n) { 
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  tf::StampedTransform temp_stamp;
  while(n.ok()){
    try{
    //	tf2_geometry_msgs::tf2_geometry_msgs a;
      temp_trans = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
      tf::transformStampedMsgToTF(temp_trans, temp_stamp);
      transformed = temp_stamp;
      tf_time = temp_stamp.stamp_;
      break;
    }
    catch (tf2::TransformException &ex) {
      int s;
      //ROS_WARN("%s",ex.what());
    }
	}  	
  const double mean = 0.0;
  const double stddev = 5.0;
  std::default_random_engine generator;
  std::normal_distribution<double> dist(mean, stddev);

	const double ro_mean = 0.0;
  const double ro_stddev = 0.00;
  std::default_random_engine ro_generator;
  std::normal_distribution<double> ro_dist(ro_mean, ro_stddev);  

 tf::Matrix3x3 rotation_tf_matrix;
	tf::Quaternion Quaternion_temp;
	geometry_msgs::Quaternion c2l;
	c2l.x = transformed.getRotation()[0] + ro_dist(ro_generator); 
	c2l.y = transformed.getRotation()[1] + ro_dist(ro_generator);
	c2l.z = transformed.getRotation()[2] + ro_dist(ro_generator);
	c2l.w = transformed.getRotation()[3] + ro_dist(ro_generator);
	
	quaternionMsgToTF(c2l, Quaternion_temp);
	rotation_tf_matrix.setRotation(Quaternion_temp); //myQuaternion
  Eigen::Matrix3d rotation_matrix;
	tf::matrixTFToEigen(rotation_tf_matrix, rotation_matrix);
	initial_guess(0,0) = rotation_matrix(0,0);
	initial_guess(0,1) = rotation_matrix(0,1);
	initial_guess(0,2) = rotation_matrix(0,2);
	initial_guess(1,0) = rotation_matrix(1,0);
	initial_guess(1,1) = rotation_matrix(1,1);
	initial_guess(1,2) = rotation_matrix(1,2);
	initial_guess(2,0) = rotation_matrix(2,0);
	initial_guess(2,1) = rotation_matrix(2,1);
	initial_guess(2,2) = rotation_matrix(2,2);
	initial_guess(0,3) = transformed.getOrigin()[0] + dist(generator);//shift origin
	initial_guess(1,3) = transformed.getOrigin()[1] + dist(generator);//
	initial_guess(2,3) = 1.8;//transformed.getOrigin()[2];
  return 1;
}
void Local::tf_broadcast(Eigen::Matrix4d input, std::string frame_name, ros::Time msgtime){
	tf::Matrix3x3 mat_local;
  mat_local.setValue(static_cast<double>(input(0,0)), static_cast<double>(input(0,1)), static_cast<double>(input(0,2)),
                       static_cast<double>(input(1,0)), static_cast<double>(input(1,1)), static_cast<double>(input(2,2)),
                       static_cast<double>(input(2,0)), static_cast<double>(input(2,1)), static_cast<double>(input(2,2)));
	double roll, pitch, yaw;
  mat_local.getRPY(roll, pitch, yaw, 1);
  transform.setOrigin(tf::Vector3(input(0, 3), input(1, 3), input(2, 3)));
  tf::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, msgtime, "map", frame_name));
}
/*void Local::get_tf(ros::NodeHandle n){
	tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf::StampedTransform temp_stamp;
	while(n.ok()){
		try{
		//	tf2_geometry_msgs::tf2_geometry_msgs a;
			temp_trans = tfBuffer.lookupTransform("base_link", "sensorhead_1", ros::Time(0));
			tf::transformStampedMsgToTF(temp_trans, temp_stamp);
			base2baja = temp_stamp;

			temp_trans = tfBuffer.lookupTransform("base_link", "velodyne", ros::Time(0));
			tf::transformStampedMsgToTF(temp_trans, temp_stamp);
			base2velo = temp_stamp;

			temp_trans = tfBuffer.lookupTransform("lucid_cameras/gige_3_near", "sensorhead_1", ros::Time(0));
			tf::transformStampedMsgToTF(temp_trans, temp_stamp);
			c32baja = temp_stamp;

			temp_trans = tfBuffer.lookupTransform("lucid_cameras/gige_0", "sensorhead_1", ros::Time(0));
			tf::transformStampedMsgToTF(temp_trans, temp_stamp);
			c02baja = temp_stamp;
			base2c3 = base2baja * c32baja.inverse();
			base2c0 = base2baja * c02baja.inverse();
			c3_camera_height = base2c3.getOrigin()[2];
			c0_camera_height = base2c0.getOrigin()[2];
			std::cout <<"tf OK!!!";
			break;
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
		}
	}
}*/
void Local::read_map(){
  Json::Value root;
  std::ifstream file("/mnt/data/linux/bag_files/DCV_BAG/map/data/itri/roadlines.json", std::ifstream::binary);
  file >> root;

  //std::cout<<root["roadlines"].size(); //This will print the entire json object.
  for (int i = 0; i < root["roadlines"].size();  i++) {//lines
    for (int j = 0; j < root["roadlines"][i]["points"].size();j++) {//points
      if (root["roadlines"][i]["points"][j]["type"].asDouble() < 10 || root["roadlines"][i]["points"][j]["type"].asDouble() < 0) {
        pcl::PointXYZ temp;
        temp.x = root["roadlines"][i]["points"][j]["x"].asDouble();
        temp.y = root["roadlines"][i]["points"][j]["y"].asDouble();
        temp.z = 0;//root["roadlines"][i]["points"][j]["z"].asDouble();
        mapp->points.push_back(temp);
      }
    }
  }
  mapp->width = (int)mapp->points.size();
  mapp->height = 1;

}