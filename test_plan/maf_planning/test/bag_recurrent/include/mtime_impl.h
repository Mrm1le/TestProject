#pragma once

#include <chrono>
#include <cstdint>

namespace detail {

class MTimePoint {
public:
  MTimePoint() : ns_(0) {}
  MTimePoint(std::uint64_t ns) : ns_(ns) {}
  double sec() const { return double(ns_) * 1e-9; }
  double ms() const { return double(ns_) * 1e-6; }
  uint64_t us() const { return ns_ / 1000; }
  uint64_t ns() const { return ns_; }

private:
  std::uint64_t ns_;
};

class MTimeInstance {
public:
  MTimeInstance() { start_ = std::chrono::high_resolution_clock::now(); }
  MTimePoint timestamp() const {
    return MTimePoint(std::chrono::duration<std::uint64_t, std::nano>(
                          std::chrono::high_resolution_clock::now() - start_)
                          .count());
  }

private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};

} // namespace detail

inline detail::MTimeInstance *MTIME() {
  static detail::MTimeInstance instance;
  return &instance;
}
