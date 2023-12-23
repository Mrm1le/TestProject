#include <cmath>
#include <string>

#include <opencv2/highgui/highgui.hpp>

#include "planner/motion_planner/optimizers/openspace_optimizer/hybrid_a_star_2/heuristic_cost.hpp"

namespace msquare {

namespace hybrid_a_star_2 {

bool HeuristicCost::dump(std::string file_name) {
  std::ofstream ofs(file_name, std::ios::out | std::ios::binary);
  if (!ofs.is_open()) {
    return false;
  }
  ofs.write(reinterpret_cast<char *>(&x_min_), sizeof(x_min_));
  ofs.write(reinterpret_cast<char *>(&y_min_), sizeof(y_min_));
  ofs.write(reinterpret_cast<char *>(&theta_min_), sizeof(theta_min_));
  ofs.write(reinterpret_cast<char *>(&x_step_), sizeof(x_step_));
  ofs.write(reinterpret_cast<char *>(&y_step_), sizeof(y_step_));
  ofs.write(reinterpret_cast<char *>(&theta_step_), sizeof(theta_step_));
  ofs.write(reinterpret_cast<char *>(&x_count_), sizeof(x_count_));
  ofs.write(reinterpret_cast<char *>(&y_count_), sizeof(y_count_));
  ofs.write(reinterpret_cast<char *>(&theta_count_), sizeof(theta_count_));
  ofs.write(reinterpret_cast<char *>(&cost_scale_), sizeof(cost_scale_));
  ofs.write(reinterpret_cast<char *>(&y_flip_), sizeof(y_flip_));
  ofs.write(reinterpret_cast<char *>(cost_.data()),
            sizeof(std::uint8_t) * cost_.size());
  return true;
}

void HeuristicCost::dumpImage() {
  for (int it = 0; it < theta_count_; it++) {
    int y_count_real = y_flip_ ? ((y_count_ - 1) * 2 + 1) : y_count_;
    float y_min_real = y_flip_ ? (-float(y_count_ - 1) * y_step_) : y_min_;
    cv::Mat_<std::uint8_t> result(y_count_real, x_count_);
    float theta = theta_min_ + float(it) * theta_step_;
    for (int iy = 0; iy < result.rows; iy++) {
      for (int ix = 0; ix < result.cols; ix++) {
        float x = x_min_ + float(ix) * x_step_;
        float y = y_min_real + float(iy) * y_step_;
        float cost = get(true, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, x, y, theta);
        result(iy, ix) = std::round(cost * inv_cost_scale_);
      }
    }
    char buf[256];
    sprintf(buf, "%010d.png", it);
    cv::imwrite(std::string(buf), result);
  }
}

} // namespace hybrid_a_star_2

} // namespace msquare
