#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <set>
#include <utility>
#include "ros/ros.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
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
struct NODE_CRF {
	int vert_idx1;
	int vert_idx2;
	int label;	//
	int idx;	// group #
	double unary;
	//int edge;//??
};
struct EDGE_CRF {
	std::vector<int> node_idx;	// nodes index
	int grp_idx;		// group # that it belongs to
	int label;
	float pairwise;
};
struct NODE_GRP {
	std::vector<int> idx;	// node #
};
struct VALID_LINE {
  double a, b, c, d;
  pdd getUpPoint() const {
    return std::make_pair(a * (280 * 280 * 280) + b * (280 * 280) + c * 280 + d, 280);
  }
  pdd getMidPoint() const {
    return std::make_pair(a * (600 * 600 * 600) + b * (600 * 600) + c * 600 + d, 600);
  }
  pdd getDownPoint() const {
    return std::make_pair(a * (536 * 536 * 536) + b * (536 * 536) + c * 536 + d, 536);
  }
  pdd get360Point() const {
    return std::make_pair(a * (1000 * 1000 * 1000) + b * (1000 * 1000) + c * 1000 + d, 1000);
  }
  bool operator<(VALID_LINE L) const {
    if (getDownPoint() == L.getDownPoint())
      return getUpPoint() < L.getUpPoint();
    return getDownPoint() < L.getDownPoint();
  }
  int head;
  double cov;
  int times;
};

class LaneDetection {
 public:
  LaneDetection() {}
  ~LaneDetection() {}
  void initialize_variable(cv::Mat img_src, float roiRatio);
  void initialize_Img(cv::Mat img_input);
  
  //bool initialize_variable(std::string &img_name);
  //bool initialize_Img(cv::Mat img_input);
  void lane_marking_detection(cv::Mat img_input, cv::Mat img_show,
                              bool verbose = false);
  int dist_ftn1(int i, int sj, double slope);
  int dist_ftn3(int i, int j, int s_i, int s_j);
  void seed_generation(cv::Mat img_input, cv::Mat img_show, bool verbose = false);
  void seed_specification(MARKING_SEED &marking_seed_curr, int mode);
  std::vector<cv::Point2d> graph_generation(cv::Mat inputImage, cv::Mat outputImage, int numThreshold);
  float dist_ftn2(int idx1, int idx2);
  float slope_ftn(cv::Point2f i, cv::Point2f j);
  float length_ftn(cv::Point2f str_p, cv::Point2f end_p);
  std::vector<VALID_LINE> validating_final_seeds(double rotate, ros::Time times, cv::Mat homo, ros::Publisher pub);
  void poly2(std::vector<cv::Point2f> points, int n, std::vector<float> &coeff);
  void poly3(std::vector<cv::Point2f> points, int n, std::vector<float>& coeff);
  void poly4(std::vector<cv::Point2f> points, int n, std::vector<float>& coeff);
  void node_grouping(cv::Mat& mat_in, int size, int type, int n, int label);
  float unary_ftn(int i, int j);
  float pairwise_ftn(std::vector<cv::Point2f>& pts);
  cv::Mat CannyDetector(cv::Mat src);
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
  std::vector<NODE_CRF> nodes;
	std::vector<EDGE_CRF> edges;
  // Lane marking definition
  const int MAX_LANE_MARKING = 3500;
  const int MAX_LW_N = 100;//100
  const int MAX_LW_F = 30;//30
  const int MAX_LW_D = 5;//50
  const int MIN_LW_N = 10;//10
  const int MIN_LW_F = 1;//1
  const int gvalue = 100;//100

  // Lane Marking Grouping
  const int MAX_LANE_SEED = 200;
  const int SEED_MARKING_DIST_THRES = 20;//1000;//200
  const int VALID_SEED_MARKING_NUMBER_THRES =15;// 5;//6
  const double LOW_LEVEL_ASS_THRES = 1.96;//1;//1.9
  const int RANSAC_ITERATIONS = 500;//500
  const int RANSAC_MODEL_SIZE = 5;//3
  const int RANSAC_ERROR_THRESHOLD = 90;//90
  const double RANSAC_INLINERS = 0.75;//;;0.75;

  /*bool fileFirst = 1, VP_success = 0;
  std::set<VALID_LINE> v;
  cv::Point2f VP(680, 340);

  int VP_count = 0;

  std::vector<float> coeff3(4);*/
};
