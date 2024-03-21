#pragma once
#include <chrono>
#include <iostream>
#include <string>
namespace utils {
template <bool Enable> class Timer {
public:
  Timer(std::string name) : name_(name) {}
  void Tic() { start_ = std::chrono::high_resolution_clock::now(); }
  void Toc() {
    double time = std::chrono::duration<std::uint64_t, std::nano>(
                      std::chrono::high_resolution_clock::now() - start_)
                      .count() *
                  1e-6;
    std::cout << name_.c_str() << ": " << time << " ms" << std::endl;
  }

private:
  std::string name_;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
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
  void Tic() { start_ = std::chrono::high_resolution_clock::now(); }
  void Toc() {
    total_ += std::chrono::duration<std::uint64_t, std::nano>(
                  std::chrono::high_resolution_clock::now() - start_)
                  .count() *
              1e-6;
    count_++;
    std::cout << name_.c_str() << ": , iter: " << count_
              << ", average time: " << total_ / count_ << " ms" << std::endl;
  }

private:
  std::string name_;
  int count_;
  double total_;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
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
  void Tic() { start_ = std::chrono::high_resolution_clock::now(); }
  void Toc() {
    total_ += std::chrono::duration<std::uint64_t, std::nano>(
                  std::chrono::high_resolution_clock::now() - start_)
                  .count() *
              1e-6;
    count_++;
    if (count_ == period_) {
      std::cout << name_.c_str() << ": , iter: " << count_
                << ", average time: " << total_ / count_ << " ms" << std::endl;
      count_ = 0;
      total_ = 0.0;
    }
  }

private:
  std::string name_;
  int count_;
  double total_;
  int period_;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};

template <> class AveragePeriodTimer<false> {
public:
  AveragePeriodTimer(std::string name, int period) {}
  void Tic() {}
  void Toc() {}
};

template <bool Enable> class TotalTimer {
public:
  TotalTimer(std::string name) : name_(name), count_(0), total_(0.0) {}
  ~TotalTimer() {
    std::cout << name_.c_str() << ": , iter: " << count_
              << ", total time: " << total_ << " ms" << std::endl;
  }
  void Tic() { start_ = std::chrono::high_resolution_clock::now(); }
  void Toc() {
    total_ += std::chrono::duration<std::uint64_t, std::nano>(
                  std::chrono::high_resolution_clock::now() - start_)
                  .count() *
              1e-6;
    count_++;
  }

private:
  std::string name_;
  int count_;
  double total_;
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};

template <> class TotalTimer<false> {
public:
  TotalTimer(std::string name) {}
  void Tic() {}
  void Toc() {}
};
} // namespace utils
