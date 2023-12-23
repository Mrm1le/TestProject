#ifndef MSQUARE_DECISION_PLANNING_TIMER_H_
#define MSQUARE_DECISION_PLANNING_TIMER_H_

namespace msquare {

namespace parking {

class Timer {
public:
  explicit Timer(double duration) : duration_(duration) { reset(); }
  void reset() { start_clock_ = MTIME()->timestamp().sec(); }
  void set_timer_duration(double duration) { duration_ = duration; }
  bool is_timeout() {
    if (MTIME()->timestamp().sec() - start_clock_ > duration_) {
      return true;
    } else {
      return false;
    }
  }
  double get_count() { return MTIME()->timestamp().sec() - start_clock_; }

private:
  double start_clock_;
  double duration_;
};

} // namespace parking

} // namespace msquare

#endif