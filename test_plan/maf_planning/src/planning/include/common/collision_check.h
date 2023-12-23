#define MSQUARE_DECISION_PLANNING_COMMON_COLLISION_CHECK_H_

#include "mtime_core/mtime.h"
#include "planning/common/circle_object_buffer.h"
#include <Eigen/Dense>

using namespace mtime;

namespace msd_planning {
class CollisionCheck {
public:
  CollisionCheck();
  ~CollisionCheck();

  void calculate_data(void);
  void update_velocity(double velocity_mps);
  void update_rotation_matrix();
  void update_linear_acceleration(double linear_acceleration_x,
                                  double linear_acceleration_y,
                                  double linear_acceleration_z);

  bool get_collide_to_limiter_when_reverse(void);

private:
  double min(CircleObjectBuffer<double> *buffer) {
    double min_value = std::numeric_limits<double>::max();
    for (int i = 0; i < buffer->size(); i++) {
      double value = buffer->at(i);
      if (value < min_value) {
        min_value = value;
      }
    }
    return min_value;
  }

  double max(CircleObjectBuffer<double> *buffer) {
    double max_value = -std::numeric_limits<double>::max();
    for (int i = 0; i < buffer->size(); i++) {
      double value = buffer->at(i);
      if (value > max_value) {
        max_value = value;
      }
    }
    return max_value;
  }

  double average(CircleObjectBuffer<double> *buffer) {
    double sum = 0.0;
    for (int i = 0; i < buffer->size(); i++) {
      sum += buffer->at(i);
    }
    return sum / buffer->size();
  }

  double last_update_velocity_time_ = MTIME()->timestamp().sec();
  double last_update_linear_acceleration_time_ = MTIME()->timestamp().sec();
  double last_collide_to_limiter_when_reverse_time_ =
      MTIME()->timestamp().sec();
  double sample_time_ = 1.0;
  int sample_freq_imu_ = 100;
  int sample_freq_velocity_ = 10;
  int buffer_length_imu_ = static_cast<int>(sample_freq_imu_ * sample_time_);
  int buffer_length_velocity_ =
      static_cast<int>(sample_freq_velocity_ * sample_time_);

  double reverse_apa_speed_limit_ = -1.0;

  double average_velocity_ = 0.0;
  double linear_acceleration_x_positive_ = 0.0;
  double linear_acceleration_z_delta_ = 0.0;

  bool collide_to_limiter_when_reverse_ = false;

  CircleObjectBuffer<double> *velocity_buffer_;
  CircleObjectBuffer<double> *linear_acceleration_x_buffer_;
  CircleObjectBuffer<double> *linear_acceleration_z_buffer_;
  Eigen::Matrix3d r_s2b_;
};
} // namespace msd_planning