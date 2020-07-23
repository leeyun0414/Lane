#ifndef PARABOLICMASK_H_
#define PARABOLICMASK_H_
#include <opencv2/opencv.hpp>

namespace aps {
class parabolicMask {
 public:
  parabolicMask();

  parabolicMask(double w, double h, double s);

  virtual ~parabolicMask();

  const cv::Mat& mkMask();

 private:
  double m_a, m_b, m_s;
  int m_width, m_height;
  cv::Mat m_mask;
};
}  // namespace aps
#endif

