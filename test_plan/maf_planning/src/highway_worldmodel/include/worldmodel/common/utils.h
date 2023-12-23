#pragma once

#include "mjson/mjson.hpp"
#include "msd/worldmodel/worldmodel_generator.h"
#include <mutex>

namespace msd_worldmodel {

template <typename T>
inline T clip(const T &x, const T &min_value, const T &max_value) {
  return std::min(std::max(x, min_value), max_value);
}

template <typename T>
inline bool value_between(const T &value, const T &a, const T &b) {
  return a <= value && value <= b;
}

inline bool property_available(const size_t &available,
                               const size_t &property) {
  return (available & property) > 0;
}

inline std::string get_frame_id() { return "car"; }

class FrequencyKeeper {
public:
  virtual ~FrequencyKeeper() = default;
  virtual void sync() = 0;
  virtual void run_immediately() = 0;
  virtual double get_waitting_time() = 0;
  static std::shared_ptr<FrequencyKeeper>
  make_simple_keeper(const double &frequency);
};

#define UNIQUE_LOCK(data_mutex)                                                \
  std::unique_lock<std::mutex> _utils_##data_mutex##_data_lock(data_mutex);

#define DECLARE_SINGLETON(classname)                                           \
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

#define DECLARE_REGISTER(classname)                                            \
public:                                                                        \
  bool register_by_name(                                                       \
      const std::string &name,                                                 \
      std::function<std::shared_ptr<classname>(const mjson::Reader &)>         \
          constructor) {                                                       \
    std::unique_lock<std::mutex> lock(register_mutex_);                        \
    constructor_map_[name] = constructor;                                      \
    return true;                                                               \
  }                                                                            \
                                                                               \
  std::shared_ptr<classname> build_by_name(const std::string &name,            \
                                           const mjson::Reader &reader = {}) { \
    if (constructor_map_.count(name) > 0)                                      \
      return constructor_map_[name](reader);                                   \
    return nullptr;                                                            \
  }                                                                            \
                                                                               \
private:                                                                       \
  std::mutex register_mutex_;                                                  \
  std::map<std::string,                                                        \
           std::function<std::shared_ptr<classname>(const mjson::Reader &)>>   \
      constructor_map_;

#define REGISTER_CLASS(classname, constructor_name, classimpl)                 \
  static bool classimpl##_register_value_##constructor_name =                  \
      classname##Factory::instance()->register_by_name(#constructor_name,      \
                                                       &classimpl::make)

#define DECLARE_FACTORY(classname)                                             \
  class classname##Factory {                                                   \
  public:                                                                      \
    DECLARE_REGISTER(classname)                                                \
    DECLARE_SINGLETON(classname##Factory)                                      \
  };

} // namespace msd_worldmodel
