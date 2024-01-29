#pragma once

// #include "mtime/mtime.h"
#include "common/mdebug_context.h"
#include "common/utils/macro.h"
#include "mjson/mjson.hpp"
#include "planning/common/sysconf.h"

#define DDP_DEBUG_LOG_HELPER(stmt)                                             \
  if (!::npp::common::running_on_cpu_limited_system()) {                       \
    stmt;                                                                      \
  }
namespace msquare {
class MdebugJsonDictHelper {
public:
  template <std::size_t N> MdebugJsonDictHelper(char const (&name)[N]) {
    MdebugContext::Instance()->json_dict_buffer().append_dict(name);
  }
  ~MdebugJsonDictHelper() {
    MdebugContext::Instance()->json_dict_buffer().end_dict();
  }
};

class MdebugJsonArrayHelper {
public:
  template <std::size_t N> MdebugJsonArrayHelper(char const (&name)[N]) {
    MdebugContext::Instance()->json_dict_buffer().append_array(name);
  }
  ~MdebugJsonArrayHelper() {
    MdebugContext::Instance()->json_dict_buffer().end_array();
  }
};

class MdebugJsonObjectHelper {
public:
  MdebugJsonObjectHelper() {
    MdebugContext::Instance()->json_dict_buffer().append_object();
  }
  ~MdebugJsonObjectHelper() {
    MdebugContext::Instance()->json_dict_buffer().end_object();
  }
};

} // namespace msquare

#define MDEBUG_JSON_BEGIN_DICT(name)                                           \
  do {                                                                         \
    msquare::MdebugJsonDictHelper mdebug_json_dict_helper_##name{#name};

#define MDEBUG_JSON_END_DICT(name)                                             \
  }                                                                            \
  while (0)                                                                    \
    ;

#define MDEBUG_JSON_ADD_ITEM(key, value, parent_name)                          \
  MdebugContext::Instance()->json_dict_buffer().add_item(#key, value);

#define MDEBUG_JSON_BEGIN_ARRAY(name)                                          \
  do {                                                                         \
    msquare::MdebugJsonArrayHelper mdebug_json_array_helper_##name{#name};

#define MDEBUG_JSON_END_ARRAY(name)                                            \
  }                                                                            \
  while (0)                                                                    \
    ;

#define MDEBUG_JSON_ADD_TO_ARRAY(value, array_name)                            \
  MdebugContext::Instance()->json_dict_buffer().add_to_array(value);

#define MDEBUG_JSON_BEGIN_OBJECT(...)                                          \
  do {                                                                         \
    msquare::MdebugJsonObjectHelper mdebug_json_object_helper{};

#define MDEBUG_JSON_END_OBJECT(...)                                            \
  }                                                                            \
  while (0)                                                                    \
    ;
