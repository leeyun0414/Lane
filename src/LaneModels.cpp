#include "LaneModels.hpp"
#include <cmath>
namespace vision {
namespace lane_model {
Parabola fit(std::vector<cv::Point2f>& points, const int n, int degree) {
  Eigen::MatrixXd aMat, bMat, xMat;
  aMat.resize(n, 4);
  bMat.resize(n, 1);
  xMat.resize(4, 1);
  for (int rr = 0; rr != n; ++rr) {
    double x = points[rr].y;
    double y = points[rr].x;
    for (int d = 0; d < degree; d++) {
      aMat(rr, d) = 1 * pow(x, d);
      if (degree < 4)
        aMat(rr, degree) = 0;
    }

    //aMat(rr, 0) = 1;
    //aMat(rr, 1) = x;
    //aMat(rr, 2) = x * x;
    //aMat(rr, 3) = x * x * x;
    
    bMat(rr, 0) = y;
  }
  xMat = aMat.colPivHouseholderQr().solve(bMat);

  return Parabola(xMat(3, 0),xMat(2, 0), xMat(1, 0), xMat(0, 0));
}
}  // namespace lane_model
}  // namespace vision
