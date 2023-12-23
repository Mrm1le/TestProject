#define MSQUARE_DECISION_PLANNING_COMMON_STATIC_CHECK_H_

#include "planning/common/circle_object_buffer.h"

namespace msd_planning {
class StaticCheck {
public:
  StaticCheck();
  ~StaticCheck();

  void UpdateWheelPositionData(uint16_t front_left, uint16_t front_right,
                               uint16_t rear_left, uint16_t rear_right);
  bool get_static_status(void) { return static_status_; };

private:
  void UpdateStaticStatus(void);
  double sample_time_ = 0.5;
  int sample_freq_ = 10;
  int buffer_length_ = static_cast<int>(sample_freq_ * sample_time_);

  bool static_status_ = false;

  CircleObjectBuffer<uint16_t> *front_left_buffer_;
  CircleObjectBuffer<uint16_t> *front_right_buffer_;
  CircleObjectBuffer<uint16_t> *rear_left_buffer_;
  CircleObjectBuffer<uint16_t> *rear_right_buffer_;
};
} // namespace msd_planning