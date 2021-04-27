

#include "Processing.h"

class Test{
  private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr sur_cloud;
  public:
    Test(int camera_num, ros::NodeHandle n){
      sur_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
      Processing images_0(n, "0");
      Processing images_1(n, "1");
      //Processing images_2(n, "2");
      //Processing images_3(n, "3");
      //Processing images_4(n, "4");
    }
    void mix_cloud(pcl::PointCloud<pcl::PointXYZ> input, pcl::PointCloud<pcl::PointXYZ> output);
};