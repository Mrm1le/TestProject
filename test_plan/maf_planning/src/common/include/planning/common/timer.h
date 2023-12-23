#pragma once

#include <string>

#ifdef BUILD_IN_TEST_BAG_RECURRENT
#include "msd_log_macro_impl.h"
#include "mtime_impl.h"
#else // BUILD_IN_TEST_BAG_RECURRENT
#include "logging.h"
#include "mtime_core/mtime.h"
#endif // BUILD_IN_TEST_BAG_RECURRENT

namespace msd_planning {

namespace utils {

namespace detail {

using TimePoint = decltype(MTIME()->timestamp());

} // namespace detail

template <bool Enable> class Timer {
public:
  Timer(std::string name) : name_(name) {}
  void Tic() { start_ = MTIME()->timestamp(); }
  void Toc() {
    double time = MTIME()->timestamp().ms() - start_.ms();
    time -= MTIME()->timestamp().ms() - MTIME()->timestamp().ms();
    MSD_LOG(ERROR, "%s: %f ms\n", name_.c_str(), time);
  }

private:
  std::string name_;
  detail::TimePoint start_;
};

template <> class Timer<false> {
public:
  Timer(std::string name) {}
  void Tic() {}
  void Toc() {}
};

template <bool Enable> class AverageTimer {
public:
  AverageTimer(std::string name) : name_(name), count_(0), total_(0.0) {}
  void Tic() { start_ = MTIME()->timestamp(); }
  void Toc() {
    total_ += MTIME()->timestamp().ms() - start_.ms();
    total_ -= MTIME()->timestamp().ms() - MTIME()->timestamp().ms();
    count_++;
    MSD_LOG(ERROR, "%s: %f ms\n", name_.c_str(), total_ / count_);
  }

private:
  std::string name_;
  int count_;
  double total_;
  detail::TimePoint start_;
};

template <> class AverageTimer<false> {
public:
  AverageTimer(std::string name) {}
  void Tic() {}
  void Toc() {}
};

template <bool Enable> class AveragePeriodTimer {
public:
  AveragePeriodTimer(std::string name, int period)
      : name_(name), count_(0), total_(0.0), period_(period) {}
  void Tic() { start_ = MTIME()->timestamp(); }
  void Toc() {
    total_ += MTIME()->timestamp().ms() - start_.ms();
    total_ -= MTIME()->timestamp().ms() - MTIME()->timestamp().ms();
    count_++;
    if (count_ == period_) {
      MSD_LOG(ERROR, "%s: %f ms\n", name_.c_str(), total_ / count_);
      count_ = 0;
      total_ = 0.0;
    }
  }

private:
  std::string name_;
  int count_;
  double total_;
  int period_;
  detail::TimePoint start_;
};

template <> class AveragePeriodTimer<false> {
public:
  AveragePeriodTimer(std::string name, int period) {}
  void Tic() {}
  void Toc() {}
};

template <bool Enable, int Iter> class TotalTimer {
public:
  TotalTimer(std::string name)
      : name_(name), count_(0), total_(0.0), iter_(0), tic_(false) {}
  ~TotalTimer() {
    MSD_LOG(ERROR, "%s: %f ms, %d times\n", name_.c_str(), total_ * Iter,
            count_ * Iter);
  }
  void Tic() {
    iter_++;
    if (iter_ == Iter) {
      start_ = MTIME()->timestamp();
      tic_ = true;
      iter_ = 0;
    } else {
      tic_ = false;
    }
  }
  void Toc() {
    if (tic_) {
      total_ += MTIME()->timestamp().ms() - start_.ms();
      total_ -= MTIME()->timestamp().ms() - MTIME()->timestamp().ms();
      count_++;
      tic_ = false;
    }
  }

private:
  std::string name_;
  int count_;
  double total_;
  detail::TimePoint start_;
  int iter_;
  bool tic_;
};

template <int Iter> class TotalTimer<false, Iter> {
public:
  TotalTimer(std::string name) {}
  void Tic() {}
  void Toc() {}
};

} // namespace utils

} // namespace msd_planning
