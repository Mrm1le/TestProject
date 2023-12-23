#include "common/static_check.h"
#include "planning/common/common.h"

namespace msd_planning {

StaticCheck::StaticCheck() {
  sample_time_ = 0.5;
  sample_freq_ = 50;

  buffer_length_ = static_cast<int>(sample_freq_ * sample_time_);
  front_left_buffer_ = new CircleObjectBuffer<uint16_t>(buffer_length_);
  front_right_buffer_ = new CircleObjectBuffer<uint16_t>(buffer_length_);
  rear_left_buffer_ = new CircleObjectBuffer<uint16_t>(buffer_length_);
  rear_right_buffer_ = new CircleObjectBuffer<uint16_t>(buffer_length_);

  for (int i = 0; i < buffer_length_; i++) {
    (void)front_left_buffer_->push(0);
    (void)front_right_buffer_->push(0);
    (void)rear_left_buffer_->push(0);
    (void)rear_right_buffer_->push(0);
  }
}

StaticCheck::~StaticCheck() {
  delete front_left_buffer_;
  delete front_right_buffer_;
  delete rear_left_buffer_;
  delete rear_right_buffer_;
}

void StaticCheck::UpdateStaticStatus(void) {
  static_status_ =
      (front_left_buffer_->front() == front_left_buffer_->back()) &&
      (front_right_buffer_->front() == front_right_buffer_->back()) &&
      (rear_left_buffer_->front() == rear_left_buffer_->back()) &&
      (rear_right_buffer_->front() == rear_right_buffer_->back());
  return;
}

void StaticCheck::UpdateWheelPositionData(uint16_t front_left,
                                          uint16_t front_right,
                                          uint16_t rear_left,
                                          uint16_t rear_right) {
  (void)front_left_buffer_->push(front_left);
  (void)front_right_buffer_->push(front_right);
  (void)rear_left_buffer_->push(rear_left);
  (void)rear_right_buffer_->push(rear_right);
  UpdateStaticStatus();
  return;
}
} // namespace msd_planning
