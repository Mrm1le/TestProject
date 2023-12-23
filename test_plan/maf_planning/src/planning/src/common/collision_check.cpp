#include "common/collision_check.h"
#include "common/config/vehicle_param.h"
#include "planning/common/common.h"

namespace msd_planning {

CollisionCheck::CollisionCheck() {
  sample_time_ = 1.0;
  sample_freq_imu_ = 100;
  sample_freq_velocity_ = 10;

  reverse_apa_speed_limit_ = -1.0;

  buffer_length_imu_ = static_cast<int>(sample_freq_imu_ * sample_time_);
  buffer_length_velocity_ =
      static_cast<int>(sample_freq_velocity_ * sample_time_);
  velocity_buffer_ = new CircleObjectBuffer<double>(buffer_length_velocity_);
  linear_acceleration_x_buffer_ =
      new CircleObjectBuffer<double>(buffer_length_imu_);
  linear_acceleration_z_buffer_ =
      new CircleObjectBuffer<double>(buffer_length_imu_);

  for (int i = 0; i < buffer_length_imu_; i++) {
    (void)linear_acceleration_x_buffer_->push(0.0);
    (void)linear_acceleration_z_buffer_->push(0.0);
  }
  for (int i = 0; i < buffer_length_velocity_; i++) {
    (void)velocity_buffer_->push(0.0);
  }
}

CollisionCheck::~CollisionCheck() {
  delete velocity_buffer_;
  delete linear_acceleration_x_buffer_;
  delete linear_acceleration_z_buffer_;
}

bool CollisionCheck::get_collide_to_limiter_when_reverse(void) {
  double &linear_acceleration_x_positive_thres_ =
      msquare::VehicleParam::Instance()
          ->limiter_detect_linear_acceleration_x_positive;
  double &linear_acceleration_z_delta_thres_ =
      msquare::VehicleParam::Instance()
          ->limiter_detect_linear_acceleration_z_delta;
  double &collision_duration_time_ =
      msquare::VehicleParam::Instance()->limiter_detect_collision_duration_time;
  double current_time = MTIME()->timestamp().sec();
  calculate_data();
  // MSD_LOG(WARN,"linear_acceleration: %f, %f",
  // linear_acceleration_x_positive_, linear_acceleration_z_delta_);
  if (average_velocity_ < 0.0 && average_velocity_ > reverse_apa_speed_limit_ &&
      linear_acceleration_x_positive_ > linear_acceleration_x_positive_thres_ &&
      linear_acceleration_z_delta_ > linear_acceleration_z_delta_thres_) {
    if (!collide_to_limiter_when_reverse_) {
      collide_to_limiter_when_reverse_ = true;
      last_collide_to_limiter_when_reverse_time_ = current_time;
    }
  } else {
    if (collide_to_limiter_when_reverse_) {
      if (current_time - last_collide_to_limiter_when_reverse_time_ >
          collision_duration_time_) {
        collide_to_limiter_when_reverse_ = false;
      }
    }
  }
  return collide_to_limiter_when_reverse_;
}

void CollisionCheck::calculate_data(void) {
  average_velocity_ = average(velocity_buffer_);
  linear_acceleration_x_positive_ = max(linear_acceleration_x_buffer_);
  linear_acceleration_z_delta_ =
      max(linear_acceleration_z_buffer_) - min(linear_acceleration_z_buffer_);

  MSD_DBG_TAG_LOOK_AT(collision_check, average_velocity_);
  MSD_DBG_TAG_LOOK_AT(collision_check, linear_acceleration_x_positive_);
  MSD_DBG_TAG_LOOK_AT(collision_check, linear_acceleration_z_delta_);
}

void CollisionCheck::update_velocity(double velocity) {
  (void)velocity_buffer_->push(velocity);
}

void CollisionCheck::update_rotation_matrix() {
  auto &r_s2b_vec = msquare::VehicleParam::Instance()->r_s2b;
  r_s2b_ = Eigen::Matrix3d::Identity();
  if (r_s2b_vec.size() != 3) {
    return;
  }
  Eigen::Vector3d r_s2b(r_s2b_vec[0], r_s2b_vec[1], r_s2b_vec[2]);
  if (!r_s2b.isZero()) {
    double len = r_s2b.norm();
    r_s2b_ = Eigen::AngleAxisd(len, r_s2b / len).toRotationMatrix();
  }
}

void CollisionCheck::update_linear_acceleration(double linear_acceleration_x,
                                                double linear_acceleration_y,
                                                double linear_acceleration_z) {
  Eigen::Vector3d acc_imu(linear_acceleration_x, linear_acceleration_y,
                          linear_acceleration_z);
  Eigen::Vector3d acc_car = r_s2b_ * acc_imu;
  double g = 9.8;
  (void)linear_acceleration_x_buffer_->push(acc_car[0] * g);
  (void)linear_acceleration_z_buffer_->push(acc_car[2] * g);
}
} // namespace msd_planning
