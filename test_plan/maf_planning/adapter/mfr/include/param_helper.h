#pragma once

#define PARAM_HELPER_STRINGIFY(x) #x

#define PARAM_HELPER_TOSTRING(x) PARAM_HELPER_STRINGIFY(x)

#define READ_STRING_PARAM(var, topic)                                          \
  var = param_reader.get<std::string>(topic).c_str();                          \
  log_manager.info("param '%s': %s", topic, var.c_str());
