#include "planning/common/sysconf.h"

#include <stdlib.h>
#include <strings.h>
#include <sys/utsname.h>

namespace npp {
namespace common {

SystemType g_system_type = SystemType::UNKNOWN;

class SystemTypeHelper {
public:
  SystemTypeHelper() {
    struct utsname u;
    (void)uname(&u);
    if (strcasecmp("QNX", u.sysname) == 0) {
      g_system_type = SystemType::QNX;
    } else if (strcasecmp("Linux", u.sysname) == 0) {
      g_system_type = SystemType::LINUX;
    } else if (strcasecmp("Darwin", u.sysname) == 0) {
      g_system_type = SystemType::DARWIN;
    }
  }
};

static SystemTypeHelper __s_system_type_helper{};

} // namespace common
} // namespace npp
