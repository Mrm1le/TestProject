#include "planner/motion_planner/common/window_smoother.h"
#include "planning/common/common.h"

namespace msquare {

WindowSmoother::WindowSmoother(const int &half_window_width) {
  half_window_width_ = half_window_width;
}

void WindowSmoother::smooth(std::vector<std::pair<double, double>> *data) {
  int window_width = half_window_width_ * 2 + 1;
  if ((int)data->size() < window_width) {
    MSD_LOG(INFO, "Date size is smaller than window width!!");
    return;
  }
  // smooth first
  double curr_sum = 0.0;
  int index_s = 0, index_e = 0;
  for (; index_e < (int)data->size() && index_e - index_s < window_width;
       ++index_e) {
    curr_sum = curr_sum + data->at(index_e).first;
  }
  index_e--;
  std::deque<double> deq;
  deq.clear();
  for (int i = 0; i < (int)data->size(); ++i) {
    deq.push_back(curr_sum / static_cast<double>(window_width));
    if (i - index_s == half_window_width_ &&
        index_e - i == half_window_width_ && index_e + 1 < (int)data->size()) {
      curr_sum = curr_sum - data->at(index_s).first;
      data->at(index_s).first = deq.front();
      deq.pop_front();
      index_s++;
      index_e++;
      curr_sum = curr_sum + data->at(index_e).first;
    }
  }
  for (; index_s < (int)data->size(); ++index_s) {
    data->at(index_s).first = deq.front();
    deq.pop_front();
  }
  // smooth second
  curr_sum = 0.0;
  index_s = 0, index_e = 0;
  for (; index_e < (int)data->size() && index_e - index_s < window_width;
       ++index_e) {
    curr_sum = curr_sum + data->at(index_e).second;
  }
  index_e--;
  deq.clear();
  for (int i = 0; i < (int)data->size(); ++i) {
    deq.push_back(curr_sum / static_cast<double>(window_width));
    if (i - index_s == half_window_width_ &&
        index_e - i == half_window_width_ && index_e + 1 < (int)data->size()) {
      curr_sum = curr_sum - data->at(index_s).second;
      data->at(index_s).second = deq.front();
      deq.pop_front();
      index_s++;
      index_e++;
      curr_sum = curr_sum + data->at(index_e).second;
    }
  }
  for (; index_s < (int)data->size(); ++index_s) {
    data->at(index_s).second = deq.front();
    deq.pop_front();
  }
}

} // namespace msquare
