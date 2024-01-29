#pragma once

// #define OBSTACLE_GRID_DEBUG_DUMP_IMAGE

#include <vector>

#ifdef OBSTACLE_GRID_DEBUG_DUMP_IMAGE
#include <opencv2/opencv.hpp>
#endif

#include "../sbp_obstacle_interface.h"
#include "multi_circle_footprint_model.h"

namespace msquare {

namespace hybrid_a_star_2 {

class ObstacleGridLegacy {
public:
  bool inited_;

  double res_;
  double inv_res_;
  double origin_x_, origin_y_;

  // grid_data_(x,y) = minimun distance from grid center to closest obstacle
  std::vector<float> grid_data_;

  int rows_;
  int cols_;
  // gx, gy = math.floor((x-origin_x_)/res), math.floor((y-origin_y_)/res)
  //      (x,y) : coordinate in meters from local coordinate
  //      (gx,gy) : grid coordinate

  double debug_sum_abs_diff = 0.0;
  double debug_max_positive_diff = 0.0;
  double debug_max_minus_diff = 0.0;
  int debug_calcdis_function_called_num = 0;
  int debug_obs_point_num = 0;

public:
  ObstacleGridLegacy() {
    inited_ = false;
    res_ = 0;
    inv_res_ = 0.0;
    origin_x_ = 0;
    origin_y_ = 0;
    rows_ = 0;
    cols_ = 0;
  }

  ~ObstacleGridLegacy() {}

  bool isInited() const { return inited_; }

  bool init(int rows, int cols, double res, double origin_x, double origin_y) {
    if (rows <= 0 || cols <= 0 || res <= 0) {
      return false;
    }
    grid_data_.resize(rows * cols);
    memset(grid_data_.data(), 0, sizeof(float) * rows * cols);

    origin_x_ = origin_x;
    origin_y_ = origin_y;
    res_ = res;
    inv_res_ = 1.0 / res_;
    rows_ = rows;
    cols_ = cols;

    inited_ = true;
    return true;
  }

  bool
  constructFromSbpObstacle(const std::vector<SbpObstaclePoint> &discrete_obs) {
    std::vector<planning_math::Vec2d> points;
    for (std::size_t i = 0; i < discrete_obs.size(); i++) {
      const std::vector<planning_math::Vec2d> &obs_points =
          discrete_obs[i].getPoints();
      points.insert(points.end(), obs_points.begin(), obs_points.end());
    }
    debug_obs_point_num = (int)points.size();

    for (int gy = 0; gy < rows_; gy++) {
      double y = gy * res_ + origin_y_;
      for (int gx = 0; gx < cols_; gx++) {
        double x = gx * res_ + origin_x_;
        double min_square_distance = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < points.size(); i++) {
          double dx = points[i].x() - x;
          double dy = points[i].y() - y;
          min_square_distance =
              std::min(min_square_distance, dx * dx + dy * dy);
        }
        grid_data_[gy * cols_ + gx] = std::sqrt(min_square_distance);
      }
    }
    return true;
  }

  double calcMinDistance(
      const std::shared_ptr<MultiCircleFootprintModel> &footprint_model) {
    double min_distance = std::numeric_limits<double>::infinity();
    for (int ci = 0; ci < (int)footprint_model->circles_local_.size(); ci++) {
      int gx = std::round(
          (footprint_model->circles_local_[ci].center_.x() - origin_x_) *
          inv_res_);
      int gy = std::round(
          (footprint_model->circles_local_[ci].center_.y() - origin_y_) *
          inv_res_);
      double radius = footprint_model->circles_local_[ci].radius_;
      if (gx < 0 || gx >= cols_ || gy < 0 || gy >= rows_) {
        min_distance = -radius; // out of boundary
        break;
      }

      double distance = grid_data_[gy * cols_ + gx] - radius;
      min_distance = std::min(min_distance, distance);
    }
    return min_distance;
  }

  double calcMinDistanceAndCompareWithGroundtruthDebug(
      const std::shared_ptr<MultiCircleFootprintModel> &footprint_model,
      const std::vector<SbpObstaclePtr> &obs_ptrs) {

    std::vector<planning_math::Vec2d> points;
    for (std::size_t i = 0; i < obs_ptrs.size(); i++) {
      std::vector<planning_math::Vec2d> obs_points =
          obs_ptrs[i]->getDiscretePoints(0.1);
      points.insert(points.end(), obs_points.begin(), obs_points.end());
    }

    double min_distance_gt = std::numeric_limits<double>::infinity();

    for (int ci = 0; ci < (int)footprint_model->circles_local_.size(); ci++) {
      double x = footprint_model->circles_local_[ci].center_.x();
      double y = footprint_model->circles_local_[ci].center_.y();
      double radius = footprint_model->circles_local_[ci].radius_;

      for (std::size_t i = 0; i < points.size(); i++) {
        double dx = points[i].x() - x;
        double dy = points[i].y() - y;
        min_distance_gt =
            std::min(min_distance_gt, sqrt(dx * dx + dy * dy) - radius);
      }
    }
    double min_distance = calcMinDistance(footprint_model);

    double diff = min_distance - min_distance_gt;

    if (diff > 0.0) {
      debug_max_positive_diff = std::max(debug_max_positive_diff, diff);
    } else {
      debug_max_minus_diff = std::min(debug_max_minus_diff, diff);
    }
    debug_sum_abs_diff += std::abs(diff);
    debug_calcdis_function_called_num++;

    return min_distance;
  }

#ifdef OBSTACLE_GRID_DEBUG_DUMP_IMAGE
  bool dumpGridImage(const std::string &filename) {
    cv::Mat debug_image_grayscale(rows_, cols_, CV_8UC1);
    cv::Mat debug_image_colored(rows_, cols_, CV_8UC3);

    double max_val_visual = 2.0;
    double min_val_visual = 0;
    for (int gy = 0; gy < rows_; gy++) {
      for (int gx = 0; gx < cols_; gx++) {
        double value_normalized =
            (grid_data_[(rows_ - 1 - gy) * cols_ + gx] - min_val_visual) /
            (max_val_visual - min_val_visual);
        value_normalized = std::min(value_normalized, 1.0);
        value_normalized = std::max(value_normalized, 0.0);
        debug_image_grayscale.at<uchar>(gy, gx) =
            255 - (uchar)(value_normalized * 255);
      }
    }
    cv::applyColorMap(debug_image_grayscale, debug_image_colored,
                      cv::COLORMAP_JET);
    cv::resize(debug_image_colored, debug_image_colored, cv::Size(1600, 1600));
    return cv::imwrite(filename, debug_image_colored);
  }

  bool dumpcalcMinDistanceImage(
      const std::string &filename,
      const std::shared_ptr<MultiCircleFootprintModel> &footprint_model,
      std::string debug_txt) {
    cv::Mat debug_image_grayscale(rows_, cols_, CV_8UC1);
    cv::Mat debug_image_colored(rows_, cols_, CV_8UC3);

    double max_val_visual = 2.0;
    double min_val_visual = 0;
    for (int gy = 0; gy < rows_; gy++) {
      for (int gx = 0; gx < cols_; gx++) {
        double value_normalized =
            (grid_data_[(rows_ - 1 - gy) * cols_ + gx] - min_val_visual) /
            (max_val_visual - min_val_visual);
        value_normalized = std::min(value_normalized, 1.0);
        value_normalized = std::max(value_normalized, 0.0);
        debug_image_grayscale.at<uchar>(gy, gx) =
            255 - (uchar)(value_normalized * 255);
      }
    }

    cv::applyColorMap(debug_image_grayscale, debug_image_colored,
                      cv::COLORMAP_JET);

    for (int ci = 0; ci < footprint_model->circle_num_; ci++) {
      int gx = std::round(
          (footprint_model->circles_local_[ci].center_.x() - origin_x_) / res_);
      int gy = std::round(
          (footprint_model->circles_local_[ci].center_.y() - origin_y_) / res_);
      double radius = footprint_model->circles_local_[ci].radius_;
      cv::circle(debug_image_colored, cv::Point2i(gx, rows_ - 1 - gy),
                 radius / res_, cv::Scalar(0, 255, 255), -1);
    }
    cv::resize(debug_image_colored, debug_image_colored, cv::Size(1600, 1600));

    cv::putText(debug_image_colored, debug_txt, cv::Point2d(100, 100), 0, 2.0,
                cv::Scalar(255, 255, 255));
    return cv::imwrite(filename, debug_image_colored);
  }
#endif
};

} // namespace hybrid_a_star_2

} // namespace msquare
