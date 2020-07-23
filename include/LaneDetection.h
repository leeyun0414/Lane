#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <set>
#include <utility>
#include "ros/ros.h"
#define pdd std::pair<double, double>
extern int fla;

struct LANE_MARKING {
  cv::Point2f str_p;
  cv::Point2f cnt_p;
  cv::Point2f end_p;
  cv::Point2f inn_p;
  int size;
};
struct MARKING_SEED {
  std::vector<int> index;
  int flag;  // 1: valid results, 0: candidnates, -1: invalid supermarking
  float cnt_dir;
  float str_dir;
  float end_dir;
  float length;
  cv::Point2f cnt_p;
  cv::Point2f str_p;
  cv::Point2f end_p;
};

struct VALID_LINE {
  double a, b, c;
  pdd getUpPoint() const {
    return std::make_pair(a * (280 * 280) + b * (280) + c, 280);
  }
  pdd getMidPoint() const {
    return std::make_pair(a * (500 * 500) + b * (500) + c, 500);
  }
  pdd getDownPoint() const {
    return std::make_pair(a * (720 * 720) + b * (720) + c, 720);
  }
  pdd get360Point() const {
    return std::make_pair(a * (360 * 360) + b * (360) + c, 360);
  }
  bool operator<(VALID_LINE L) const {
    if (getDownPoint() == L.getDownPoint())
      return getUpPoint() < L.getUpPoint();
    return getDownPoint() < L.getDownPoint();
  }
  //int straight_curve;
  int times;
};

class LaneDetection {
 public:
  LaneDetection() {}
  ~LaneDetection() {}
  void initialize_variable(cv::Mat img_src);
  void initialize_Img(cv::Mat img_input);
  
  //bool initialize_variable(std::string &img_name);
  //bool initialize_Img(cv::Mat img_input);
  void lane_marking_detection(cv::Mat img_input, cv::Mat img_show,
                              bool verbose = false);
  int dist_ftn1(int i, int sj, double slope);

  void seed_generation(cv::Mat img_input, cv::Mat img_show,
                       bool verbose = false);
  void seed_specification(MARKING_SEED &marking_seed_curr, int mode);
  float dist_ftn2(int idx1, int idx2);
  float slope_ftn(cv::Point2f i, cv::Point2f j);
  float length_ftn(cv::Point2f str_p, cv::Point2f end_p);
  int validating_final_seeds(cv::Mat img_show, cv::Mat &img_result,ros::Time time,
                              bool verbose);
  void poly2(std::vector<cv::Point2f> points, int n, std::vector<float> &coeff);

 private:
  cv::Size img_size;
  cv::Mat img_gray;
  unsigned int img_height;
  unsigned int img_width;
  unsigned int img_roi_height;
  unsigned int img_depth;

  // Lane marking variable
  std::vector<int> max_lw;
  std::vector<int> min_lw;
  std::vector<int> max_lw_d;
  std::vector<LANE_MARKING> lm;
  std::vector<MARKING_SEED> marking_seed;
};
