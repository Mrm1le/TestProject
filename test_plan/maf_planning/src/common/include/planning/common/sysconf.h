#ifndef NPP_COMMON_SYSCONF_H
#define NPP_COMMON_SYSCONF_H

namespace npp {
namespace common {

enum class SystemType : int {
  UNKNOWN = 0,
  LINUX = 1,
  WINDOWS = 2,
  DARWIN = 3,
  QNX = 4,
  ORIN = 5,
};

extern SystemType g_system_type;

inline bool running_on_cpu_limited_system() {
  return g_system_type == SystemType::QNX;
}

inline SystemType system_type() { return g_system_type; }

} // namespace common
} // namespace npp

#endif // NPP_COMMON_SYSCONF_H
