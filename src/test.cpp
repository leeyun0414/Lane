#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/CompressedImage.h>

using namespace sensor_msgs;
using namespace message_filters;


class Fuck
{
  private:
	message_filters::Subscriber<CompressedImage> T_sub;
	message_filters::Subscriber<CompressedImage> T_DL_sub;  
  ros::NodeHandle n;
  typedef TimeSynchronizer<CompressedImage, CompressedImage> ImageSynchronizer;
  boost::shared_ptr<ImageSynchronizer> sync;
  public:
  Fuck(ros::NodeHandle _n) {
    n = _n;
		T_sub.subscribe(n, "/zed/left/image_raw_color/compressed", 50);
		T_DL_sub.subscribe(n, "/zed/left/image_raw_color/compressed", 50);  
 
		sync.reset(new ImageSynchronizer(T_sub, T_DL_sub, 50));
    sync->registerCallback(boost::bind(&Fuck::callback, this, _1, _2));
  
  }
  void callback(const CompressedImageConstPtr& image, const CompressedImageConstPtr& cam_info)
  {
  std::cout << image->header.stamp <<std::endl;
  std::cout << cam_info->header.stamp <<std::endl;
  // Solve all of perception here...
  }
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  Fuck aaaaa(nh);
  /*
  message_filters::Subscriber<CompressedImage> image_sub(nh, "/zed/left/image_raw_color/compressed", 50);
  message_filters::Subscriber<CompressedImage> info_sub(nh, "/zed/right/image_raw_color/compressed", 50);
  TimeSynchronizer<CompressedImage, CompressedImage> sync(image_sub, info_sub, 50);
  sync.registerCallback(boost::bind(&callback, _1, _2));
*/
  ros::spin();

  return 0;
}
