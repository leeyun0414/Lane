/**
 * Given:
 *   data – a set of observed data points
 *   model – a model that can be fitted to data points
 *   n – minimum number of data points required to fit the model
 *   k – maximum number of iterations allowed in the algorithm
 *   t – threshold value to determine when a data point fits a model
 *   d – number of close data points required to assert that a model fits well
 *       to data
 *
 * Return:
 *   bestfit – model parameters which best fit the data (or nul if no good model
 *             is found)
 *
 * iterations = 0
 * bestfit = nul
 * besterr = something really large
 *
 * while iterations < k {
 *   maybeinliers = n randomly selected values from data
 *   maybemodel = model parameters fitted to maybeinliers
 *   alsoinliers = empty set
 *
 *   for every point in data not in maybeinliers {
 *     if point fits maybemodel with an error smaller than t
 *       add point to alsoinliers
 *   }
 *   if the number of elements in alsoinliers is > d {
 *     % this implies that we may have found a good model
 *     % now test how good it is
 *     bettermodel = model parameters fitted to all points in maybeinliers and
 *                   alsoinliers
 *     thiserr = a measure of how well model fits these points
 *     if thiserr < besterr {
 *       bestfit = bettermodel
 *       besterr = thiserr
 *     }
 *   }
 * increment iterations
 * }
 * return bestfit
 */

#pragma once
#include "LaneModels.hpp"
#include "random_unique.hpp"

vision::lane_model::Parabola RANSAC_Parabola(
    int iterations, int init_samples, int n, double error_threshold,
    std::vector<cv::Point2f> inputData, int degree) {
  auto best_fit = vision::lane_model::Parabola();
  double best_error = std::numeric_limits<double>::max();
  int consensus_set;

  for (int i = 0; i < iterations; ++i) {
    random_unique(inputData.begin(), inputData.end(), init_samples);
    auto model = vision::lane_model::fit(inputData, init_samples, degree);
    auto straight_model = model;
    double model_error;
    if (degree == 4) {
      straight_model.a = 0;
      straight_model.b = 0;
      model_error = model.a * model.a * model.a * model.b * model.b * 5000;
    }
    else {
      straight_model.a = 0;
      model_error = model.a * model.a * 5000;
    }
    consensus_set = 0;
    //auto model_error = model.a * model.a * 5000;  // bias towards straight lines *10000
    auto straight_model_error = 0;
    for (auto p : inputData) {
      const auto err = std::abs(model(p.y) - p.x);
      if (err < error_threshold) {
        consensus_set += 1;
        model_error += err;
        straight_model_error += std::abs(straight_model(p.y) - p.x);
      }
    }

    if (consensus_set >= n) {
      if (straight_model_error < model_error) {
        model_error = straight_model_error;
        model = straight_model;
      }

      if (model_error < best_error) {
        best_fit = model;
        best_error = model_error;
      }
    }
  }
  return best_fit;
}
