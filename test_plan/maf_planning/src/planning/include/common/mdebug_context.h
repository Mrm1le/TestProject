#ifndef COMMON_MDEBUG_CONTEXT_
#define COMMON_MDEBUG_CONTEXT_

#include <memory>
#include <string>
#include <unordered_map>

#include "common/utils/macro.h"
#include "mjson/mjson.hpp"
#include "rapidjson/internal/dtoa.h"
#include "rapidjson/internal/itoa.h"
#include <limits>
#include <stdarg.h>
#include <stdio.h>
#include <vector>

#include <functional>

namespace msquare {

#define MDEBUG_HELPER(stmt)                                                    \
  if (!::npp::common::running_on_cpu_limited_system()) {                       \
    stmt;                                                                      \
  }

#define MDEBUG_ADD_JSON(json_object)                                           \
  MDEBUG_HELPER(DdpDebugLogger::Instance()->add_json(json_object))

#define MDEBUG_BEGIN_TABLE(table_name)                                         \
  MDEBUG_HELPER(MdebugContext::Instance()->begin_table_entry(table_name))

#define MDEBUG_ADD_TABLE_COL(column_name, column_type)                         \
  MDEBUG_HELPER(                                                               \
      MdebugContext::Instance()->add_table_column(column_name, column_type))

#define MDEBUG_BEGIN_TABLE_ROW()                                               \
  MDEBUG_HELPER(MdebugContext::Instance()->begin_table_row())

#define MDEBUG_ADD_TABLE_DATA(data)                                            \
  MDEBUG_HELPER(MdebugContext::Instance()->add_table_row_data(data))

class MdebugSwitch {
  static std::map<std::string, std::function<void()>> &switch_register() {
    static std::map<std::string, std::function<void()>>
        switch_registor_instance;
    return switch_registor_instance;
  }

  static void register_switch(const std::string &name,
                              std::function<void()> set_open) {
    switch_register().emplace(name, set_open);
  }

  template <typename ConcreteSwitch> struct SwitchBase {
    static bool &is_open() {
      static bool is_open = false;
      return is_open;
    }

    SwitchBase() {
      register_switch(ConcreteSwitch::name(), [] { is_open() = true; });
    }
  };

public:
  struct VirtualLane : SwitchBase<VirtualLane> {
    static const std::string &name() {
      static std::string switch_name{"virtual_lane"};
      return switch_name;
    }
  } virtual_lane_switch_;

  template <typename T> bool is_open() { return T::is_open(); }

  static MdebugSwitch *Instance() {
    static MdebugSwitch instance{};
    return &instance;
  }
  MdebugSwitch(const MdebugSwitch &) = delete;
  void operator=(const MdebugSwitch &) = delete;

private:
  MdebugSwitch() {
    const std::string DDP_DEBUG_SWITCH_ENV_NAME = "MDEBUG_SWITCH";

    char *env = getenv(DDP_DEBUG_SWITCH_ENV_NAME.c_str());
    if (env != nullptr) {
      ddp_debug_switch_env_ = std::string{env};
      config_ddp_debug_switch_by_env();
    } else {
      open_all_switches();
    }
  }

  std::vector<std::string> split(const std::string &s,
                                 const std::string &delimiter) {
    std::vector<std::string> result{};
    size_t last = 0;
    size_t next = s.find(delimiter, last);
    while (next != std::string::npos) {
      result.emplace_back(s.substr(last, next - last));
      last = next + delimiter.size();
      next = s.find(delimiter, last);
    }
    result.emplace_back(s.substr(last));
    return result;
  }

  void config_ddp_debug_switch_by_env() {
    std::vector<std::string> opened_switches =
        split(ddp_debug_switch_env_, ",");
    if (opened_switches.empty()) {
      return;
    }
    std::map<std::string, std::function<void()>> &switch_openers =
        switch_register();

    for (const auto &opened_switch : opened_switches) {
      auto switch_opener = switch_openers.find(opened_switch);
      if (switch_opener == switch_openers.end()) {
        continue;
      }
      switch_opener->second();
    }
  }

  void open_all_switches() {
    std::map<std::string, std::function<void()>> &switch_openers =
        switch_register();

    for (const auto &switch_opener : switch_openers) {
      switch_opener.second();
    }
  }

  std::string ddp_debug_switch_env_{};
};

class JsonRawBuffer {
  enum : size_t {
    INIT_BUF_SIZE = 1 << 16,
  };

public:
  JsonRawBuffer() {
    buf_.resize(INIT_BUF_SIZE);
    reset();
  }

  template <typename T, typename... T_ARGS>
  void append_raw(T &&e, T_ARGS &&... args) {
    append_raw(std::forward<T>(e));
    append_raw(std::forward<T_ARGS>(args)...);
  }

  template <size_t N> void append_raw(const char (&s)[N]) {
    check_to_increase_buf(N);

    memcpy(&buf_[cursor_], s, N);
    cursor_ += N - 1;
  }

  void append_raw(double v) {
    constexpr size_t double_to_str_buf_size =
        4 + std::numeric_limits<double>::digits10 +
        -std::numeric_limits<double>::min_exponent10;
    check_to_increase_buf(double_to_str_buf_size);

    char *b = &buf_[cursor_];
    char *e = rapidjson::internal::dtoa(v, b);
    *e = 0;
    cursor_ += e - b;
  }

  void append_raw(int v) {
    constexpr size_t int32_to_str_buf_size = 12;
    check_to_increase_buf(int32_to_str_buf_size);

    char *b = &buf_[cursor_];
    char *e = rapidjson::internal::i32toa(v, b);
    *e = 0;
    cursor_ += e - b;
  }

  void append_raw(bool v) {
    if (v) {
      append_raw("true");
    } else {
      append_raw("false");
    }
  }

  void append_raw(const std::string &s) {
    check_to_increase_buf(s.size());

    memcpy(&buf_[cursor_], s.c_str(), s.size());
    cursor_ += s.size();
    buf_[cursor_] = 0;
  }

  void append_raw(const char *s, const size_t len) {
    check_to_increase_buf(len);

    memcpy(&buf_[cursor_], s, len);
    cursor_ += len;
    buf_[cursor_] = 0;
  }

  void pop_last() {
    if (cursor_ == 0) {
      return;
    }
    buf_[--cursor_] = 0;
  }

  void pop_last(const char c) {
    if (cursor_ == 0 || buf_[cursor_ - 1] != c) {
      return;
    }
    buf_[--cursor_] = 0;
  }

  char last() const { return buf_[cursor_ - 1]; }

  const char *c_str() {
    buf_[cursor_] = 0;
    return &buf_[0];
  }

  void reset() {
    cursor_ = 0;
    buf_[cursor_] = 0;
  }

  size_t size() const { return cursor_; }

private:
  void check_to_increase_buf(size_t n) {
    if (buf_.size() > cursor_ + n) {
      return;
    }
    size_t buf_size = buf_.size();
    while (buf_size <= cursor_ + n) {
      buf_size <<= 1;
    }
    buf_.resize(buf_size);
  }

  std::vector<char> buf_;
  size_t cursor_;
};

class JsonDictBuffer : private JsonRawBuffer {
public:
  template <std::size_t N> void append_dict(char const (&name)[N]) {
    if (JsonRawBuffer::last() != '{') {
      JsonRawBuffer::append_raw(",\"");
    } else {
      JsonRawBuffer::append_raw("\"");
    }
    JsonRawBuffer::append_raw(name);
    JsonRawBuffer::append_raw("\":{");
  }

  template <std::size_t N_K, bool is_str = true>
  void add_item(char const (&k)[N_K], const char *v) {
    add_key(k);
    size_t v_len = strlen(v);
    add_value<is_str>(v, v_len);
  }

  template <std::size_t N_K, bool is_str = true>
  void add_item(char const (&k)[N_K], const std::string &vs) {
    add_key(k);

    const char *v = vs.c_str();
    size_t v_len = vs.size();
    add_value<is_str>(v, v_len);
  }

  template <std::size_t N_K, typename T_V>
  void add_item(char const (&k)[N_K], T_V v) {
    add_key(k);
    add_value(v);
  }

  void begin_dict() { JsonRawBuffer::append_raw("{"); }

  void end_dict() { JsonRawBuffer::append_raw("}"); }

  template <std::size_t N> void append_array(char const (&name)[N]) {
    if (JsonRawBuffer::last() != '{') {
      JsonRawBuffer::append_raw(",\"");
    } else {
      JsonRawBuffer::append_raw("\"");
    }
    JsonRawBuffer::append_raw(name);
    JsonRawBuffer::append_raw("\":[");
  }

  template <bool is_str = true> void add_to_array(const std::string &vs) {
    if (JsonRawBuffer::last() != '[') {
      JsonRawBuffer::append_raw(",");
    }
    if (is_str) {
      JsonRawBuffer::append_raw("\"");
      JsonRawBuffer::append_raw(vs.c_str(), vs.size());
      JsonRawBuffer::append_raw("\"");
    } else {
      JsonRawBuffer::append_raw(vs.c_str(), vs.size());
    }
  }

  template <typename T> void add_to_array(T v) {
    if (JsonRawBuffer::last() != '[') {
      JsonRawBuffer::append_raw(",");
    }
    JsonRawBuffer::append_raw(v);
  }

  void end_array() { JsonRawBuffer::append_raw("]"); }

  void append_object() {
    if (JsonRawBuffer::last() != '[') {
      JsonRawBuffer::append_raw(",{");
    } else {
      JsonRawBuffer::append_raw("{");
    }
  }

  void end_object() { JsonRawBuffer::append_raw("}"); }

  const char *c_str() {
    if (!dumped_) {
      end_dict();
      dumped_ = true;
    }
    return JsonRawBuffer::c_str();
  }

  void reset() {
    JsonRawBuffer::reset();
    begin_dict();
    dumped_ = false;
  }

  size_t size() const { return JsonRawBuffer::size(); }

private:
  template <std::size_t N_K> void add_key(char const (&k)[N_K]) {
    if (JsonRawBuffer::last() != '{') {
      JsonRawBuffer::append_raw(",\"");
    } else {
      JsonRawBuffer::append_raw("\"");
    }
    JsonRawBuffer::append_raw(k);
    JsonRawBuffer::append_raw("\":");
  }

  template <bool is_str> void add_value(const char *v, size_t v_len) {
    if (is_str) {
      JsonRawBuffer::append_raw("\"");
      JsonRawBuffer::append_raw(v, v_len);
      JsonRawBuffer::append_raw("\"");
    } else {
      JsonRawBuffer::append_raw(v, v_len);
    }
  }

  template <typename T> void add_value(T v) { JsonRawBuffer::append_raw(v); }

  bool dumped_{false};
};

class MdebugContext {
private:
  // this is a singleton class
  DECLARE_SINGLETON(MdebugContext);

public:
  void reset() {
    table_cnt_map_.clear();
    mdebug_log_json_ = mjson::Json(mjson::Json::array());
    json_raw_buffer_.reset();
    json_dict_buffer_.reset();
  }

  const std::string dump_mdebug_log_string() { return mdebug_log_json_.dump(); }

  void add_entry(const mjson::Json entry) {
    mdebug_log_json_.array_value().push_back(entry);
  }

  /**
   * @brief entry API
   * entry {
   *   "type": number,
   *   "category": string, // unique id for each project
   *   "coordinate": number,
   *   "camera_source": number, // 同 perception_interface_msgs/CameraSourceEnum
   *   "data": object
   * }
   *
   * # type enum
   * POLYGON = 0
   * POLYLINE = 1
   * POINT = 2
   * CIRCLE = 3
   * STADIUM = 4
   * TEXT = 5
   * JSON = 100
   * TABLE = 101
   *
   * # coordinate enum
   * EGO = 1
   * BOOT = 2
   * CAMERA = -1
   */
  enum class ENTRY_TYPE : int {
    POLYGON = 0,
    POLYLINE = 1,
    POINT = 2,
    CIRCLE = 3,
    STADIUM = 4,
    TEXT = 5,
    JSON = 100,
    TABLE = 101
  };
  enum class COORDINATE : int { EGO = 1, BOOT = 2, CAMERA = -1 };
  enum class CAMERA_SOURCE : int { DEFAULT = 0 };

  void begin_entry(const ENTRY_TYPE type, const COORDINATE coordinate,
                   const CAMERA_SOURCE camera_source,
                   const std::string &category);

  // json
  void add_json_entry(mjson::Json const &json_object);

  // table
  void begin_table_entry(const std::string &table_name);
  void add_table_column(const std::string &col_name,
                        const std::string &col_type);
  void begin_table_row();
  void add_table_row_data(const double data);
  void add_table_row_data(const std::string &data);

  JsonRawBuffer &json_raw_buffer() { return json_raw_buffer_; }
  JsonDictBuffer &json_dict_buffer() { return json_dict_buffer_; }

  // ego info
  void append_ego_info(const std::string &&info) {
    ego_infos_.emplace_back(info);
  }
  void add_ego_info_entry(const double x, const double y, const double z);

private:
  class Node final {
  public:
    Node(const ENTRY_TYPE type, mjson::Json &json)
        : type_(type), json_node_(json), table_column_id_(0U) {}
    ENTRY_TYPE type() const { return type_; }
    mjson::Json &json_node() { return json_node_; }
    mjson::Json const &json_node() const { return json_node_; }
    size_t table_column_id() const { return table_column_id_; }
    size_t &multable_table_column_id() { return table_column_id_; }

  private:
    DISALLOW_COPY_AND_ASSIGN(Node);

    ENTRY_TYPE type_;
    mjson::Json &json_node_;
    size_t table_column_id_;
  };

  mjson::Json mdebug_log_json_{mjson::Json::array()};
  std::unique_ptr<Node> current_node_{nullptr};
  std::unordered_map<std::string, size_t> table_cnt_map_{};
  std::vector<std::string> ego_infos_{};
  JsonRawBuffer json_raw_buffer_;
  JsonDictBuffer json_dict_buffer_;
};

} // namespace msquare

#endif // COMMON_MDEBUG_CONTEXT_
