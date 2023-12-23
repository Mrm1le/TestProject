#include "worldmodel/common/logging.h"
#include "mjson/mjson.hpp"
#include "msd/worldmodel/common/logging.h"
#include "mtime_core/mtime.h"
#include <algorithm>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <iostream>
#include <string>

namespace msd_worldmodel {

namespace {

std::string read_file(const std::string &path) {
  FILE *file = fopen(path.c_str(), "r");
  if (nullptr == file) {
    return "";
  }
  std::shared_ptr<FILE> fp(file, [](FILE *file) { fclose(file); });
  (void)fseek(fp.get(), 0, SEEK_END);
  std::vector<char> content(ftell(fp.get()));
  (void)fseek(fp.get(), 0, SEEK_SET);
  auto read_bytes = fread(content.data(), 1, content.size(), fp.get());
  if (read_bytes != content.size()) {
    return "";
  }
  (void)read_bytes;
  return std::string(content.begin(), content.end());
}

} // namespace

bool MSDWorldModelLogger::set_config(const std::string &config) {

  constexpr auto PARAM_KEY_LOG_ENABLE_CALLBACK = "enable_callback";
  constexpr auto MANAGER_LOGGER = "logger";
  auto config_info = read_file(config);
  if (config_info.empty()) {
    return false;
  }
  auto config_reader = mjson::Reader(config_info);
  auto logger_reader =
      mjson::Reader(config_reader.get<mjson::Json>(MANAGER_LOGGER));
  enable_callback_ =
      logger_reader.get<bool>(PARAM_KEY_LOG_ENABLE_CALLBACK, false, false);

  settings_.config_file = config;
  return true;
}

void MSDWorldModelLogger::set_callback(
    mlog::MLogEndpointCallback outer_callback) {
  if (enable_callback_) {
    settings_.endpoint_callback = outer_callback;
    settings_.endpoint |= mlog::MLogEndpoint::MLOG_ENDPOINT_CUSTOM_CALLBACK;
  }
}

MSDWorldModelLogger &MSDWorldModelLogger::get_logger_instance() {
  static MSDWorldModelLogger instance{};
  return instance;
}

mlog::MLogManager *MSDWorldModelLogger::get_logger_manager_instance() {
  if (log_manager_ == nullptr) {
    static std::once_flag flag;
    std::call_once(
        flag, [&]() { log_manager_ = mlog::MLogManager::make(settings_); });
  }
  return log_manager_.get();
}

bool MSDWorldModel_set_log_config(const std::string &config) {
  return MSDWorldModelLogger::get_logger_instance().set_config(config);
}

void MSDWorldModel_set_log_callback(mlog::MLogEndpointCallback callback) {
  MSDWorldModelLogger::get_logger_instance().set_callback(callback);
}

void MSDWorldModel_init_log_manager() {
  MSDWorldModelLogger::get_logger_instance().get_logger_manager_instance();
}

} // namespace msd_worldmodel
