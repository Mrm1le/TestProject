#ifndef WINDOW_SMOOTHER_H_
#define WINDOW_SMOOTHER_H_

#include <deque>
#include <utility>
#include <vector>

namespace msquare {

class WindowSmoother {
public:
  // constructor
  WindowSmoother(const int &half_window_width);

  // smppth data with moving rectangle window
  void smooth(std::vector<std::pair<double, double>> *data);

private:
  int half_window_width_;
};

} // namespace msquare

#endif
