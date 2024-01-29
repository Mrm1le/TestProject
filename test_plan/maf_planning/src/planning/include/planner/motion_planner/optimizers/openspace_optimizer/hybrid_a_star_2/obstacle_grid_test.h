#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include "../sbp_obstacle_interface.h"
#include "multi_circle_footprint_model.h"
#include <fstream>

namespace msquare {

namespace hybrid_a_star_2 {

class ObstacleGridTest {
public:
  bool inited_;

  double res_;
  double inv_res_;
  double origin_x_, origin_y_;

  // grid_data_gt(x,y) = minimun distance from grid center to closest obstacle
  cv::Mat grid_data_gt_;

  // This buffer is only used because cv::fillConvexPoly does not fill dynamic
  // "color" (distance-to-center)
  //  so a buffer of voroni-center index is required.  (will be removed in the
  //  future)
  cv::Mat voroni_index_gt_;

  std::vector<planning_math::Vec2d> test_buffer_obstacle_points_;

  int rows_;
  int cols_;
  // gx, gy = math.floor((x-origin_x_)/res), math.floor((y-origin_y_)/res)
  //      (x,y) : coordinate in meters from local coordinate
  //      (gx,gy) : grid coordinate
public:
  double debug_sum_abs_diff = 0.0;
  double debug_max_positive_diff = 0.0;
  double debug_max_minus_diff = 0.0;
  int debug_calcdis_function_called_num = 0;
  int debug_obs_point_num = 0;

  double test_result_max_grid_positive_diff_ = 0.0;
  double test_result_max_grid_negative_diff_ = 0.0;
  double test_result_sum_grid_mean_abs_diff_ = 0.0;
  std::vector<cv::Point2i> test_result_big_grid_diff_coordinate_;

public:
  ObstacleGridTest() {
    inited_ = false;
    res_ = 0;
    inv_res_ = 0.0;
    origin_x_ = 0;
    origin_y_ = 0;
    rows_ = 0;
    cols_ = 0;
  }

  ~ObstacleGridTest() {}

  bool isInited() const { return inited_; }

  bool init(int rows, int cols, double res, double origin_x, double origin_y) {
    if (rows <= 0 || cols <= 0 || res <= 0) {
      return false;
    }
    grid_data_gt_.create(rows, cols, CV_32F);
    grid_data_gt_.setTo(0);

    voroni_index_gt_.create(rows, cols, CV_32S);

    origin_x_ = origin_x;
    origin_y_ = origin_y;
    res_ = res;
    inv_res_ = 1.0 / res_;
    rows_ = rows;
    cols_ = cols;

    inited_ = true;
    return true;
  }

  std::vector<cv::Point2f> convertObstaclePointsToGrid(
      const std::vector<planning_math::Vec2d> &obs_points) {
    std::vector<cv::Point2f> out_points_in_grid(obs_points.size());
    for (std::size_t i = 0; i < obs_points.size(); i++) {
      out_points_in_grid[i].x = (obs_points[i].x() - origin_x_) * inv_res_;
      out_points_in_grid[i].y = (obs_points[i].y() - origin_y_) * inv_res_;
    }
    return out_points_in_grid;
  }

  std::vector<planning_math::Vec2d> convertSbpObstacleToObstaclePoints(
      const std::vector<SbpObstaclePoint> &descrete_obs) {
    std::vector<planning_math::Vec2d> points;
    for (std::size_t i = 0; i < descrete_obs.size(); i++) {
      const std::vector<planning_math::Vec2d> &obs_points =
          descrete_obs[i].getPoints();
      points.insert(points.end(), obs_points.begin(), obs_points.end());
    }
    return points;
  }

  bool constructGridGTWithoutVoroniFromSbpObstaclePoints(
      const std::vector<SbpObstaclePoint> &descrete_obs) {
    test_buffer_obstacle_points_ =
        convertSbpObstacleToObstaclePoints(descrete_obs);
    return constructGridGTWithoutVoroniFromObstaclePoints(
        test_buffer_obstacle_points_);
  }
  // this function generates grid data gt
  //  each grid data gt contains min-distance to nearest obstacle
  //  sbp obstacle are converted to points before use
  bool constructGridGTWithoutVoroniFromObstaclePoints(
      const std::vector<planning_math::Vec2d> &points) {
    float *grid_data_ptr = (float *)grid_data_gt_.data;
    for (int gy = 0; gy < rows_; gy++) {
      double y = gy * res_ + origin_y_;
      for (int gx = 0; gx < cols_; gx++) {
        double x = gx * res_ + origin_x_;
        double min_square_distance = 10000.0 * 10000.0;
        for (std::size_t i = 0; i < points.size(); i++) {
          double dx = points[i].x() - x;
          double dy = points[i].y() - y;
          min_square_distance =
              std::min(min_square_distance, dx * dx + dy * dy);
        }
        grid_data_ptr[gy * cols_ + gx] = std::sqrt(min_square_distance);
      }
    }
    return true;
  }

  // this function generates full grid_data && voroni_index GT
  //  from given voroni_center points
  //  voroni center points should be the output of getVoroniFaceList so that the
  //  repeated center points are removed call this function with raw
  //  vecor-of-obstacle-points-in-grid does not garantee accurate voroni index
  //  GT
  bool constructGridAndVoroniGTFromVoroniCenterPoints(
      const std::vector<cv::Point2f> &voroni_center_points_in_grid) {
    for (int gy = 0; gy < rows_; gy++) {
      double y = gy * res_ + origin_y_;
      for (int gx = 0; gx < cols_; gx++) {
        double x = gx * res_ + origin_x_;
        double min_square_distance = 10000.0 * 10000.0;
        int min_sqr_idx = 0;
        for (std::size_t i = 0; i < voroni_center_points_in_grid.size(); i++) {
          double dx = voroni_center_points_in_grid[i].x - gx;
          double dy = voroni_center_points_in_grid[i].y - gy;

          double sqr_distance = dx * dx + dy * dy;
          if (sqr_distance < min_square_distance) {
            min_square_distance = sqr_distance;
            min_sqr_idx = i;
          }
        }

        grid_data_gt_.at<float>(gy, gx) = std::sqrt(min_square_distance) * res_;
        voroni_index_gt_.at<int>(gy, gx) = min_sqr_idx;
      }
    }
    return true;
  }

  double calcMinDistance(
      const std::shared_ptr<MultiCircleFootprintModel> &footprint_model) {
    double min_distance = std::numeric_limits<double>::infinity();
    float *grid_data_ptr = (float *)grid_data_gt_.data;
    for (const auto &circle : footprint_model->circles()) {
      int gx = std::round((circle.center_x - origin_x_) * inv_res_);
      int gy = std::round((circle.center_y - origin_y_) * inv_res_);
      double radius = circle.radius;
      if (gx < 0 || gx >= cols_ || gy < 0 || gy >= rows_) {
        min_distance = -radius; // out of boundary
        break;
      }

      double distance = grid_data_ptr[gy * cols_ + gx] - radius;
      min_distance = std::min(min_distance, distance);
    }
    return min_distance;
  }

  double calcMinDistanceGT(
      const std::shared_ptr<MultiCircleFootprintModel> &footprint_model,
      const std::vector<SbpObstaclePoint> &descrete_obs) {

    std::vector<planning_math::Vec2d> points =
        convertSbpObstacleToObstaclePoints(descrete_obs);
    double min_distance_gt = std::numeric_limits<double>::infinity();

    for (const auto &circle : footprint_model->circles()) {
      for (std::size_t i = 0; i < points.size(); i++) {
        double dx = points[i].x() - circle.center_x;
        double dy = points[i].y() - circle.center_y;
        min_distance_gt =
            std::min(min_distance_gt, sqrt(dx * dx + dy * dy) - circle.radius);
      }
    }

    return min_distance_gt;
  }

  void testDistanceWithGTOnce(double min_distance, double min_distance_gt) {
    double diff = min_distance - min_distance_gt;
    if (diff > 0.0) {
      debug_max_positive_diff = std::max(debug_max_positive_diff, diff);
    } else {
      debug_max_minus_diff = std::min(debug_max_minus_diff, diff);
    }
    debug_sum_abs_diff += std::abs(diff);
    debug_calcdis_function_called_num++;
  }

  // test grid diff
  // expected result should be max_grid_positive_diff ~ float 0
  //                         && max_grid_negative_diff ~ float 0
  //  will output coordinates of big diff coordinates
  template <typename Dtype>
  void testGridDiff(const Dtype *other_obs_grid_ptr, double scale = 1.0) {
    double max_positive_diff = 0.0;
    double max_negative_diff = 0.0;
    double sum_abs_diff = 0.0;
    std::vector<cv::Point2i> diff_coordinate;

    for (int gy = 0; gy < rows_; gy++) {
      for (int gx = 0; gx < cols_; gx++) {
        float obs_grid_gt_data = grid_data_gt_.at<float>(gy, gx);
        float other_grid_data =
            double(other_obs_grid_ptr[gy * cols_ + gx]) * scale;

        double diff = other_grid_data - obs_grid_gt_data;
        if (diff > 0.0) {
          max_positive_diff = std::max(max_positive_diff, diff);
        } else {
          max_negative_diff = std::min(max_negative_diff, diff);
        }
        sum_abs_diff += std::abs(diff);

        if (diff > 0.01) {
          diff_coordinate.push_back(cv::Point2i(gx, gy));
        }
      }
    }

    test_result_max_grid_positive_diff_ = max_positive_diff;
    test_result_max_grid_negative_diff_ = max_negative_diff;
    test_result_sum_grid_mean_abs_diff_ = sum_abs_diff / (rows_ * cols_);
    test_result_big_grid_diff_coordinate_ = diff_coordinate;
  }

  bool getTestGridDiffResult() {
    return !(test_result_max_grid_positive_diff_ > 0.01 ||
             test_result_max_grid_negative_diff_ < -0.01 ||
             test_result_big_grid_diff_coordinate_.size() > 0);
  }

  bool dumpGridImage(const std::string &filename) {
    cv::Mat debug_image_grayscale(rows_, cols_, CV_8UC1);
    cv::Mat debug_image_colored(rows_, cols_, CV_8UC3);

    double max_val_visual = 2.0;
    double min_val_visual = 0;

    float *grid_data_ptr = (float *)grid_data_gt_.data;
    for (int gy = 0; gy < rows_; gy++) {
      for (int gx = 0; gx < cols_; gx++) {
        double value_normalized =
            (grid_data_ptr[(rows_ - 1 - gy) * cols_ + gx] - min_val_visual) /
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
    float *grid_data_ptr = (float *)grid_data_gt_.data;
    for (int gy = 0; gy < rows_; gy++) {
      for (int gx = 0; gx < cols_; gx++) {
        double value_normalized =
            (grid_data_ptr[(rows_ - 1 - gy) * cols_ + gx] - min_val_visual) /
            (max_val_visual - min_val_visual);
        value_normalized = std::min(value_normalized, 1.0);
        value_normalized = std::max(value_normalized, 0.0);
        debug_image_grayscale.at<uchar>(gy, gx) =
            255 - (uchar)(value_normalized * 255);
      }
    }

    cv::applyColorMap(debug_image_grayscale, debug_image_colored,
                      cv::COLORMAP_JET);

    for (const auto &circle : footprint_model->circles()) {
      int gx = std::round((circle.center_x - origin_x_) / res_);
      int gy = std::round((circle.center_y - origin_y_) / res_);
      double radius = circle.radius;
      cv::circle(debug_image_colored, cv::Point2i(gx, rows_ - 1 - gy),
                 radius / res_, cv::Scalar(0, 255, 255), -1);
    }
    cv::resize(debug_image_colored, debug_image_colored, cv::Size(1600, 1600));

    cv::putText(debug_image_colored, debug_txt, cv::Point2d(100, 100), 0, 2.0,
                cv::Scalar(255, 255, 255));
    return cv::imwrite(filename, debug_image_colored);
  }

  bool dumpTestDataToFile(std::string file_name) {
    std::ofstream ofs(file_name);
    if (!ofs.is_open()) {
      return false;
    }
    char str[512];
    sprintf(str, "%d %d %f %f %f", rows_, cols_, res_, origin_x_, origin_y_);
    ofs << str << std::endl;
    ofs << test_buffer_obstacle_points_.size() << std::endl;
    for (std::size_t i = 0; i < test_buffer_obstacle_points_.size(); i++) {
      sprintf(str, "%f %f", test_buffer_obstacle_points_[i].x(),
              test_buffer_obstacle_points_[i].y());
      ofs << str << std::endl;
    }
    ofs.close();
    return true;
  }

  bool loadTestDataFromFile(std::string file_name) {
    std::ifstream ifs(file_name);
    if (!ifs.is_open()) {
      return false;
    }

    int rows, cols;
    double res, origin_x, origin_y;
    ifs >> rows >> cols >> res >> origin_x >> origin_y;
    init(rows, cols, res, origin_x, origin_y);

    int point_num = 0;

    ifs >> point_num;

    test_buffer_obstacle_points_.resize(point_num);
    for (std::size_t i = 0; i < test_buffer_obstacle_points_.size(); i++) {
      double x, y;
      ifs >> x >> y;
      test_buffer_obstacle_points_[i] = planning_math::Vec2d(x, y);
    }
    ifs.close();
    return true;
  }
};

} // namespace hybrid_a_star_2

} // namespace msquare
