#pragma once

#include "mlog_core/mlog_data_stream.h"
#include <cassert>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace msd_planning {
namespace utils {
inline std::string read_file(std::string path) {
  std::shared_ptr<FILE> fp(fopen(path.c_str(), "r"),
                           [](FILE *file) { fclose(file); });
  if (fp == nullptr)
    return "";
  (void)fseek(fp.get(), 0, SEEK_END);
  std::vector<char> content(ftell(fp.get()));
  (void)fseek(fp.get(), 0, SEEK_SET);
  auto read_bytes = fread(content.data(), 1, content.size(), fp.get());
  mph_assert(read_bytes == content.size());
  return std::string(content.begin(), content.end());
}

template <long unsigned int N, typename T>
std::string list_to_string(const T list_array) {
  if (sizeof(list_array) == 0)
    return std::string();
  mlog::MLogDataStream ss;
  for (size_t i = 0; i < N - 1; i++) {
    ss << list_array[i] << ",";
  }
  ss << list_array[N - 1];
  return ss.str();
}

template <typename T> std::string vec_to_string(const std::vector<T> &vec) {
  if (vec.empty())
    return std::string();
  mlog::MLogDataStream ss;
  for (int i = 0; i < (int)vec.size() - 1; i++) {
    ss << vec[i] << ",";
  }
  ss << vec.back();
  return ss.str();
}

} // namespace utils

#define MSD_DECLARE_SINGLETON(classname)                                       \
public:                                                                        \
  static classname *instance() {                                               \
    static std::shared_ptr<classname> instance_cached = nullptr;               \
    if (instance_cached == nullptr) {                                          \
      static std::once_flag flag;                                              \
      std::call_once(                                                          \
          flag, [&] { instance_cached = std::make_shared<classname>(); });     \
    }                                                                          \
    return instance_cached.get();                                              \
  }                                                                            \
                                                                               \
private:
} // namespace msd_planning
