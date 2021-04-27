#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>
#include "opencv2/imgproc.hpp"

namespace vision {
namespace lane_model {

struct Parabola {
  double a, b, c, d;
  
  Parabola(double A = 0, double B = 0, double C = 0, double D = 0) : a(A), b(B), c(C) , d(D){}

  double value(double x) const { return (((a * x) + b) * x + c) * x + d; }

  double operator()(double x) const { return value(x); }

  bool IsValid() const { return !(a == 0 && b == 0 && c == 0 && d == 0); }

  std::string ToString() const {
    return "f(y) = " + std::to_string(a) + " y^3 + " + std::to_string(b) +
           " y^2 + " + std::to_string(c) + " y + " + std::to_string(d);
  }
};

Parabola fit(std::vector<cv::Point2f>& points, const int n, int degree);

}  // namespace lane_model
}  // namespace vision
