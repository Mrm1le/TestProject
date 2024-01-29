
#pragma once

#include "../worldmodel_pec/utils/macro.h"
#include <string>

namespace worldmodel_pec {
class SlotReleaseContext {
private:
  // this is a singleton class
  DECLARE_SINGLETON(SlotReleaseContext);

public:
  const std::string &get_config_file_dir() const { return config_file_dir_; }
  void set_config_file_dir(const std::string &config_file_dir) {
    config_file_dir_ = config_file_dir;
  }

private:
  std::string config_file_dir_;
};
} // namespace worldmodel_pec