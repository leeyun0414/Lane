#include "LaneModels.hpp"

namespace vision {
namespace lane_model {
Parabola fit(std::vector<cv::Point2f>& points, const int n) {
  Eigen::MatrixXd aMat, bMat, xMat;
  aMat.resize(n, 3);
  bMat.resize(n, 1);
  xMat.resize(3, 1);
  for (int rr = 0; rr != n; ++rr) {
    double x = points[rr].y;
    double y = points[rr].x;
    aMat(rr, 0) = 1;
    aMat(rr, 1) = x;
    aMat(rr, 2) = x * x;
    bMat(rr, 0) = y;
  }
  xMat = aMat.colPivHouseholderQr().solve(bMat);

  return Parabola(xMat(2, 0), xMat(1, 0), xMat(0, 0));
}
}  // namespace lane_model
}  // namespace vision
