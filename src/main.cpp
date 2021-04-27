#include "Processing.h"

int main (int argc, char** argv) {
	ros::init(argc, argv, "lanetracking_node");
	ROS_INFO("my_node running...");
  ros::NodeHandle n;
  int ThreadNumber;
  n.getParam("camera_num", ThreadNumber);
  std::vector<Processing> cameras_vec;
  std::uint64_t stamp_vec[5];
  /*for (int i = 0; i < ThreadNumber; i++) {
    Processing temp(n, std::to_string(i));
    cameras_vec.push_back(temp);
  }*/
  //sub_right = n.subscribe(camera_name+"/h265/rgb8/compressed", 1, &Processing::callback1;
  Processing images_0(n, "0");
  Processing images_1(n, "1");
  /*Processing images_2(n, "2");
  Processing images_3(n, "3");
  Processing images_4(n, "4");
  cameras_vec.push_back(images_4);
  cameras_vec.push_back(images_3);
  cameras_vec.push_back(images_2);*/
  cameras_vec.push_back(images_1);
  cameras_vec.push_back(images_0);
  Local localizer = Local(n);
  ros::AsyncSpinner spinner(ThreadNumber + 1);
  spinner.start();
  pcl::PointCloud<pcl::PointXYZ>::Ptr sum(new pcl::PointCloud<pcl::PointXYZ>);
  clock_t a;
  while(ros::ok()) {
    for (int i = 0; i < ThreadNumber; i++) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
      temp = cameras_vec[i].sur_cloud;
      //cout << "temp:" << temp->points.size() <<   " ";
      //if (stamp_vec[i] == temp->header.stamp)
      //  continue;
      //else {
        //stamp_vec[i] = temp->header.stamp;
        if (temp->points.size()) {
          *sum = *sum + *temp; 
        }
      //}
    }
    //a = clock();
    localizer.ICP(sum, ros::Time::now(), n);
    //std::cout << "ICP" <<(float)(clock() -a)/CLOCKS_PER_SEC<< std::endl;
    /*for (int i = 0; i < ThreadNumber; i++) {
      
      cameras_vec[i].draw_points_on_image(localizer.sub_map, localizer.initial_guess);
    }*/
    sum->clear();
  }
  ros::waitForShutdown();
  return 0;
}
