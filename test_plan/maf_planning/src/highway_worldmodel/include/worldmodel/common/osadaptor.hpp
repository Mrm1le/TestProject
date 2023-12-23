#pragma once

#include "logging.h"
#include <iostream>
#include <thread>

#if defined(__QNX__)
#include <sys/neutrino.h>
#endif

namespace msd_worldmodel {

class OSAdaptor {
public:
  static inline bool set_thread_name(std::thread &fc_thread, const char *name);
  static inline std::string get_thread_name(std::thread &thread);
  static inline std::string get_thread_name(std::uint64_t thread_id);

private:
#if defined(__linux__)
  static constexpr auto kMaxNameLength = 16;
#elif defined(__QNX__)
  static constexpr auto kMaxNameLength = _NTO_THREAD_NAME_MAX;
#endif
};

inline std::string OSAdaptor::get_thread_name(std::thread &thread) {
  return get_thread_name(thread.native_handle());
}

inline std::string OSAdaptor::get_thread_name(std::uint64_t thread_id) {
#if defined(__linux__) || defined(__QNX__)
  char thread_name[kMaxNameLength]{};
  std::int32_t rc{pthread_getname_np(thread_id, thread_name, kMaxNameLength)};
  if (0 != rc) {
    MSD_LOG(INFO, "[mosadaptor] get thread name error: %d.\n", rc);
    return "";
  }

  return thread_name;
#else
  MSD_LOG(INFO, "[mosadaptor] pthread_getname_np() is not available.\n");
  return "";
#endif
}

inline bool OSAdaptor::set_thread_name(std::thread &fc_thread,
                                       const char *name) {
#if defined(__linux__) || defined(__QNX__)
  constexpr auto kNamePrefix = "wm/";

  if (nullptr == name) {
    MSD_LOG(INFO, "[OSAdaptor] thread name nullptr.");
  }
  std::string origin_name;
  if (name != nullptr) {
    origin_name = name;
  }
  auto pos = origin_name.find_last_of('/');
  if (std::string::npos != pos) {
    origin_name = origin_name.substr(pos + 1);
  }

  std::string internal_name = kNamePrefix + origin_name;
  if (internal_name.length() >= kMaxNameLength) {
    internal_name = internal_name.substr(0, kMaxNameLength - 1);
  }

  int rc = pthread_setname_np(fc_thread.native_handle(), internal_name.c_str());
  if (0 != rc) {
    MSD_LOG(INFO,
            "[OSAdaptor] origin name: [%s], set thread name [%s] error: %d.\n",
            name, internal_name.c_str(), rc);
    return false;
  }

  return true;
#else
  MSD_LOG(INFO,
          "[OSAdaptor] pthread_setname_np() is not available for thread name "
          "[%s].\n",
          name);
  return false;
#endif
}

} // namespace msd_worldmodel
