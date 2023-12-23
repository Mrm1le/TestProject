#pragma once

#include <assert.h>

#include <algorithm>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace argparse {

enum class Status {
  PERF_OK,
  PERF_FAILURE,
};

class Exception {
 public:
  Exception(Status status, std::string msg) : error_(status), msg_(msg){};
  Status get_status() { return error_; };
  std::string get_message() { return msg_; };

 private:
  Status error_ = Status::PERF_FAILURE;
  std::string msg_;
};

constexpr int ERROR_NUMBER_VALUE = 0;

enum class ErrorMode {
  EXIT,
  THROW_EXCEPTION,
};

enum class ActionType {
  NONE = 0,
  ACTION_STORE_TRUE,
  ACTION_STORE_FALSE,
  ACTION_STORE_CONST,
};
enum class NargsType {
  NONE = 0,
  NARGS_N,
  NARGS_STAR,
  NARGS_ADD,
  NARGS_QUESTION
};
enum class ValueType {
  NONE,
  STRING,
  DIGITS_INT,
  DIGITS_DOUBLE,
  BOOLEAN,
  STRING_ARRAY,
  INT_ARRAY,
  DOUBLE_ARRAY,
  BOOL_ARRAY
};

class EntryValue;
class EntryItem {

 public:
  typedef std::vector<EntryItem> array;

  EntryItem();
  EntryItem(const int);
  EntryItem(const double);
  EntryItem(const bool);
  EntryItem(const std::string&);
  EntryItem(const char*);
  EntryItem(const std::vector<int>&);
  EntryItem(const std::vector<double>&);
  EntryItem(const std::vector<bool>&);
  EntryItem(const std::vector<std::string>&);

  const double& digit_double_value() const;
  const int& digit_int_value() const;
  const bool& bool_value() const;
  const std::string& string_value() const;
  const std::vector<int>& int_array_value() const;
  const std::vector<double>& double_array_value() const;
  const std::vector<bool>& bool_array_value() const;
  const std::vector<std::string>& string_array_value() const;

  ValueType value_type() const;

  bool is_bool() const;
  bool is_double() const;
  bool is_int() const;
  bool is_string() const;
  bool is_int_array() const;
  bool is_double_array() const;
  bool is_bool_array() const;
  bool is_string_array() const;

  void dump(std::string&) const;
  std::string dump() const;

 private:
  std::shared_ptr<EntryValue> value_ptr_;
};
static const EntryItem EMPTY_ENTRY_ITEM;
static const std::vector<int> EMPTY_ENTRY_ITEM_INT_ARRAY;
static const std::vector<double> EMPTY_ENTRY_ITEM_DOUBLE_ARRAY;
static const std::vector<bool> EMPTY_ENTRY_ITEM_BOOL_ARRAY;
static const std::vector<std::string> EMPTY_ENTRY_ITEM_STRING_ARRAY;

struct ParamEntry {
  bool filled;
  bool required;
  std::string short_key;
  std::string long_key;
  std::string positional_key;
  std::string key;
  EntryItem value;
  ValueType value_type;
  std::string help;
  NargsType nargs;
  ActionType action;
  int nargs_number;
  EntryItem const_value;
  EntryItem default_value;
  EntryItem choices_array;
};

class EntryValue {

 public:
  virtual const double& digit_double_value() const {
    assert(ERROR_NUMBER_VALUE);
  }
  virtual const int& digit_int_value() const { assert(ERROR_NUMBER_VALUE); }
  virtual const bool& bool_value() const { assert(ERROR_NUMBER_VALUE); }
  virtual const std::string& string_value() const {
    assert(ERROR_NUMBER_VALUE);
  }
  virtual const std::vector<int>& int_array_value() const {
    assert(ERROR_NUMBER_VALUE);
  }
  virtual const std::vector<double>& double_array_value() const {
    assert(ERROR_NUMBER_VALUE);
  }
  virtual const std::vector<bool>& bool_array_value() const {
    assert(ERROR_NUMBER_VALUE);
  }
  virtual const std::vector<std::string>& string_array_value() const {
    assert(ERROR_NUMBER_VALUE);
  }
  virtual ValueType value_type() const { assert(ERROR_NUMBER_VALUE); }
  virtual bool is_bool() const { assert(ERROR_NUMBER_VALUE); }
  virtual bool is_double() const { assert(ERROR_NUMBER_VALUE); }
  virtual bool is_int() const { assert(ERROR_NUMBER_VALUE); }
  virtual bool is_string() const { assert(ERROR_NUMBER_VALUE); }
  virtual bool is_int_array() const { assert(ERROR_NUMBER_VALUE); }
  virtual bool is_double_array() const { assert(ERROR_NUMBER_VALUE); }
  virtual bool is_bool_array() const { assert(ERROR_NUMBER_VALUE); }
  virtual bool is_string_array() const { assert(ERROR_NUMBER_VALUE); }
  virtual void dump(std::string&) const { assert(ERROR_NUMBER_VALUE); }
  virtual ~EntryValue() {}
};

class ArgValue {
 public:
  ArgValue() = default;
  ArgValue(std::map<std::string, ParamEntry> value)
      : value_(value.begin(), value.end()) {}
  template <typename T>
  const T& get_value(std::string) const;

 private:
  std::map<std::string, ParamEntry> value_;
};

class ArgumentParser {
 public:
  ArgumentParser() : error_mode_(ErrorMode::EXIT) { init(); };
  virtual ~ArgumentParser() = default;

  void add_argument(std::string long_string, std::string default_string = "",
                    std::string help_string = "");

  ArgumentParser& long_name(const std::string& long_name);
  ArgumentParser& short_name(const std::string& short_name);
  ArgumentParser& positional_name(const std::string& name);

  // default
  ArgumentParser& default_value(const std::string& default_value);
  ArgumentParser& default_value(const char* temp);
  ArgumentParser& default_value(const int default_value);
  ArgumentParser& default_value(const double default_value);
  ArgumentParser& default_value(const bool default_value);
  ArgumentParser& default_value(const std::vector<std::string>& default_value);
  ArgumentParser& default_value(const std::vector<int>& default_value);
  ArgumentParser& default_value(const std::vector<double>& default_value);
  ArgumentParser& default_value(const std::vector<bool>& default_value);
  // const - if nargs='?' or action='store_const'
  ArgumentParser& const_value(const std::string& const_value);
  ArgumentParser& const_value(const char* temp);
  ArgumentParser& const_value(const int const_value);
  ArgumentParser& const_value(const double const_value);
  ArgumentParser& const_value(const bool const_value);
  ArgumentParser& const_value(const std::vector<std::string> const_value);
  ArgumentParser& const_value(const std::vector<int> const_value);
  ArgumentParser& const_value(const std::vector<double> const_value);
  ArgumentParser& const_value(const std::vector<bool> const_value);
  // help
  ArgumentParser& help(const std::string& help);
  ArgumentParser& help(const char* help);
  // nargs ? | + | * |  N
  ArgumentParser& nargs(const std::string& nargs_type);
  ArgumentParser& nargs(const int nargs_number);
  // action store_false / store_true / store_const
  ArgumentParser& action(const std::string& store_para);
  // dest - rename key
  ArgumentParser& dest(const std::string& dest);
  // choices
  ArgumentParser& choices(const std::vector<std::string>& string_array);
  ArgumentParser& choices(const std::vector<int>& int_array);
  ArgumentParser& choices(const std::vector<double>& double_array);
  ArgumentParser& choices(const std::vector<bool>& bool_array);
  // type : std::string int float bool
  ArgumentParser& type(const ValueType value_type);
  // submit a entry
  void done();

  // parse arguments
  ArgValue parse_args_any_type(int argc, char* argv[]);
  std::map<std::string, std::string> parse_args_string(int argc, char* argv[]);

  ArgumentParser& set_error_mode(ErrorMode mode) {
    error_mode_ = mode;
    return *this;
  }

 private:
  ParamEntry now_entry_;
  std::vector<ParamEntry> param_entries_;
  std::map<std::string, int> optional_short_key_;
  std::map<std::string, int> optional_long_key_;
  std::map<std::string, int> positional_key_;
  std::vector<std::shared_ptr<EntryValue>> value_store_;
  ErrorMode error_mode_;

  void check_error(bool condition, const std::string& error_msg);
  // print help
  void print_help();
  void init();

  EntryItem parse(const std::string& input, const ValueType value_type);
  bool is_entry_item_equal(const EntryItem item1, const EntryItem item2);
  bool parse_choices(const EntryItem item_value, const ParamEntry& entry);

  // positional p_input_index (--) / optional p_input_index + 1
  void parse_nargs(ParamEntry& entry, int& p_input_index, const int argc,
                   char* argv[]);
};

const std::string ERROR_STRING = "";
constexpr bool ERROR_BOOL = false;
constexpr int ERROR_INT = 0;
constexpr double ERROR_DOUBLE = 0.0;
constexpr size_t DOUBLE_TO_STRING_SIZE = 50;
constexpr size_t INT_TO_STRING_SIZE = 30;

class DigitsIntValue : public EntryValue {
 public:
  DigitsIntValue(const int value) : value_(value) {}
  const int& digit_int_value() const override { return value_; }
  ValueType value_type() const override { return value_type_; }
  void dump(std::string& output) const override {
    char temp[INT_TO_STRING_SIZE];
    snprintf(temp, INT_TO_STRING_SIZE, "%d", value_);
    output += temp;
  }

 private:
  int value_;
  static const ValueType value_type_ = ValueType::DIGITS_INT;
};
class DigitsDoubleValue : public EntryValue {
 public:
  DigitsDoubleValue(const double value) : value_(value) {}
  const double& digit_double_value() const override { return value_; }
  ValueType value_type() const override { return value_type_; }
  void dump(std::string& output) const override {
    char temp[DOUBLE_TO_STRING_SIZE];
    snprintf(temp, DOUBLE_TO_STRING_SIZE, "%.3f", value_);
    output += temp;
  }

 private:
  double value_;
  static const ValueType value_type_ = ValueType::DIGITS_DOUBLE;
};
class StringValue : public EntryValue {
 public:
  StringValue(const std::string& value) : value_(value) {}
  const std::string& string_value() const override { return value_; }
  ValueType value_type() const override { return value_type_; }
  void dump(std::string& output) const override { output += value_; }

 private:
  std::string value_;
  static const ValueType value_type_ = ValueType::STRING;
};
class BoolValue : public EntryValue {
 public:
  BoolValue(const bool value) : value_(value) {}
  const bool& bool_value() const override { return value_; }
  ValueType value_type() const override { return value_type_; }
  void dump(std::string& output) const override {
    if (value_) {
      output += "true";
    } else {
      output += "false";
    }
  }

 private:
  bool value_;
  static const ValueType value_type_ = ValueType::BOOLEAN;
};

class StringArrayValue : public EntryValue {
 public:
  StringArrayValue(const std::vector<std::string>& value)
      : value_(value.begin(), value.end()) {}

  const std::vector<std::string>& string_array_value() const override {
    return value_;
  }
  ValueType value_type() const override { return value_type_; }
  void dump(std::string& output) const override {
    output += "{";
    bool begin_flag = true;
    for (auto& item : value_) {
      if (!begin_flag) {
        output += ",";
      }
      EntryItem(item).dump(output);
      begin_flag = false;
    }
    output += "}";
  }

 private:
  std::vector<std::string> value_;
  static const ValueType value_type_ = ValueType::STRING_ARRAY;
};

class IntArrayValue : public EntryValue {
 public:
  IntArrayValue(const std::vector<int>& value)
      : value_(value.begin(), value.end()) {}

  const std::vector<int>& int_array_value() const override { return value_; }
  ValueType value_type() const override { return value_type_; }
  void dump(std::string& output) const override {
    output += "{";
    bool begin_flag = true;
    for (auto& item : value_) {
      if (!begin_flag) {
        output += ",";
      }
      EntryItem(item).dump(output);
      begin_flag = false;
    }
    output += "}";
  }

 private:
  std::vector<int> value_;
  static const ValueType value_type_ = ValueType::INT_ARRAY;
};

class DoubleArrayValue : public EntryValue {
 public:
  DoubleArrayValue(const std::vector<double>& value)
      : value_(value.begin(), value.end()) {}

  const std::vector<double>& double_array_value() const override {
    return value_;
  }
  ValueType value_type() const override { return value_type_; }
  void dump(std::string& output) const override {
    output += "{";
    bool begin_flag = true;
    for (auto& item : value_) {
      if (!begin_flag) {
        output += ",";
      }
      EntryItem(item).dump(output);
      begin_flag = false;
    }
    output += "}";
  }

 private:
  std::vector<double> value_;
  static const ValueType value_type_ = ValueType::DOUBLE_ARRAY;
};

class BoolArrayValue : public EntryValue {
 public:
  BoolArrayValue(const std::vector<bool>& value)
      : value_(value.begin(), value.end()) {}

  const std::vector<bool>& bool_array_value() const override { return value_; }
  ValueType value_type() const override { return value_type_; }
  void dump(std::string& output) const override {
    output += "{";
    bool begin_flag = true;
    for (auto item : value_) {
      if (!begin_flag) {
        output += ",";
      }
      EntryItem(item).dump(output);
      begin_flag = false;
    }
    output += "}";
  }

 private:
  std::vector<bool> value_;
  static const ValueType value_type_ = ValueType::BOOL_ARRAY;
};

template <>
inline const int& ArgValue::get_value<int>(std::string key) const {
  if (value_.find(key) == value_.end()) {
    return ERROR_INT;
  }
  auto temp = value_.find(key);
  auto value = temp->second.value;
  return value.digit_int_value();
}

template <>
inline const double& ArgValue::get_value<double>(std::string key) const {
  if (value_.find(key) == value_.end()) {
    return ERROR_DOUBLE;
  }
  auto temp = value_.find(key);
  auto value = temp->second.value;
  return value.digit_double_value();
}

template <>
inline const std::string& ArgValue::get_value<std::string>(
    std::string key) const {
  if (value_.find(key) == value_.end()) {
    return ERROR_STRING;
  }
  auto temp = value_.find(key);
  auto value = temp->second.value;
  return value.string_value();
}

template <>
inline const bool& ArgValue::get_value<bool>(std::string key) const {
  if (value_.find(key) == value_.end()) {
    return ERROR_BOOL;
  }
  auto temp = value_.find(key);
  auto value = temp->second.value;
  return value.bool_value();
}

template <>
inline const std::vector<int>& ArgValue::get_value<std::vector<int>>(
    std::string key) const {
  if (value_.find(key) == value_.end()) {
    return EMPTY_ENTRY_ITEM_INT_ARRAY;
  }
  auto temp = value_.find(key);
  return temp->second.value.int_array_value();
}

template <>
inline const std::vector<double>& ArgValue::get_value<std::vector<double>>(
    std::string key) const {
  if (value_.find(key) == value_.end()) {
    return EMPTY_ENTRY_ITEM_DOUBLE_ARRAY;
  }
  auto temp = value_.find(key);
  return temp->second.value.double_array_value();
}

template <>
inline const std::vector<bool>& ArgValue::get_value<std::vector<bool>>(
    std::string key) const {
  if (value_.find(key) == value_.end()) {
    return EMPTY_ENTRY_ITEM_BOOL_ARRAY;
  }
  auto temp = value_.find(key);
  return temp->second.value.bool_array_value();
}

template <>
inline const std::vector<std::string>&
ArgValue::get_value<std::vector<std::string>>(std::string key) const {
  if (value_.find(key) == value_.end()) {
    return EMPTY_ENTRY_ITEM_STRING_ARRAY;
  }
  auto temp = value_.find(key);
  return temp->second.value.string_array_value();
}

// construtor
inline EntryItem::EntryItem() : value_ptr_(nullptr) {}
inline EntryItem::EntryItem(const int value)
    : value_ptr_(std::make_shared<DigitsIntValue>(value)) {}
inline EntryItem::EntryItem(const double value)
    : value_ptr_(std::make_shared<DigitsDoubleValue>(value)) {}
inline EntryItem::EntryItem(const bool value)
    : value_ptr_(std::make_shared<BoolValue>(value)) {}
inline EntryItem::EntryItem(const std::string& value)
    : value_ptr_(std::make_shared<StringValue>(value)) {}
inline EntryItem::EntryItem(const std::vector<int>& value)
    : value_ptr_(std::make_shared<IntArrayValue>(value)) {}
inline EntryItem::EntryItem(const std::vector<double>& value)
    : value_ptr_(std::make_shared<DoubleArrayValue>(value)) {}
inline EntryItem::EntryItem(const std::vector<bool>& value)
    : value_ptr_(std::make_shared<BoolArrayValue>(value)) {}
inline EntryItem::EntryItem(const std::vector<std::string>& value)
    : value_ptr_(std::make_shared<StringArrayValue>(value)) {}
inline EntryItem::EntryItem(const char* value)
    : value_ptr_(std::make_shared<StringValue>(std::string(value))) {}

// check type
inline ValueType EntryItem::value_type() const {
  if (!value_ptr_) {
    return ValueType::NONE;
  }
  return value_ptr_->value_type();
}
inline bool EntryItem::is_double() const {
  return value_type() == ValueType::DIGITS_DOUBLE;
}
inline bool EntryItem::is_int() const {
  return value_type() == ValueType::DIGITS_INT;
}
inline bool EntryItem::is_string() const {
  return value_type() == ValueType::STRING;
}
inline bool EntryItem::is_bool() const {
  return value_type() == ValueType::BOOLEAN;
}
inline bool EntryItem::is_int_array() const {
  return value_type() == ValueType::INT_ARRAY;
}
inline bool EntryItem::is_double_array() const {
  return value_type() == ValueType::DOUBLE_ARRAY;
}
inline bool EntryItem::is_string_array() const {
  return value_type() == ValueType::STRING_ARRAY;
}
inline bool EntryItem::is_bool_array() const {
  return value_type() == ValueType::BOOL_ARRAY;
}

// accessor
inline const double& EntryItem::digit_double_value() const {
  if (is_double()) {
    return value_ptr_->digit_double_value();
  }
  return ERROR_DOUBLE;
}
inline const int& EntryItem::digit_int_value() const {
  if (is_int()) {
    return value_ptr_->digit_int_value();
  }
  return ERROR_INT;
}
inline const bool& EntryItem::bool_value() const {
  if (is_bool()) {
    return value_ptr_->bool_value();
  }
  return ERROR_BOOL;
}
inline const std::string& EntryItem::string_value() const {
  if (is_string()) {
    return value_ptr_->string_value();
  }
  return ERROR_STRING;
}
inline const std::vector<int>& EntryItem::int_array_value() const {
  if (is_int_array()) {
    return value_ptr_->int_array_value();
  }
  return EMPTY_ENTRY_ITEM_INT_ARRAY;
}
inline const std::vector<double>& EntryItem::double_array_value() const {
  if (is_double_array()) {
    return value_ptr_->double_array_value();
  }
  return EMPTY_ENTRY_ITEM_DOUBLE_ARRAY;
}
inline const std::vector<bool>& EntryItem::bool_array_value() const {
  if (is_bool_array()) {
    return value_ptr_->bool_array_value();
  }
  return EMPTY_ENTRY_ITEM_BOOL_ARRAY;
}
inline const std::vector<std::string>& EntryItem::string_array_value() const {
  if (is_string_array()) {
    return value_ptr_->string_array_value();
  }
  return EMPTY_ENTRY_ITEM_STRING_ARRAY;
}
// decode
inline void EntryItem::dump(std::string& output) const {
  value_ptr_->dump(output);
}
inline std::string EntryItem::dump() const {
  std::string output;
  dump(output);
  return output;
}

inline void ArgumentParser::add_argument(std::string long_string,
                                         std::string default_string,
                                         std::string help_string) {
  long_name(long_string);
  default_value(default_string);
  help(help_string);
  done();
}

inline ArgumentParser& ArgumentParser::long_name(const std::string& long_name) {
  check_error(long_name.find("--") == 0, "invaild option string " + long_name);
  check_error(now_entry_.positional_key.empty(),
              "invalid option string " + long_name);
  auto long_key = long_name.substr(2);
  check_error(optional_long_key_.find(long_key) == optional_long_key_.end(),
              "conflicting option string " + long_key);
  now_entry_.long_key = long_key;
  now_entry_.required = false;
  return *this;
}

inline ArgumentParser& ArgumentParser::short_name(
    const std::string& short_name) {
  check_error(short_name.find("-") == 0, "invaild option string " + short_name);
  check_error(now_entry_.positional_key.empty(),
              "invalid option string " + short_name);
  auto short_key = short_name.substr(1);
  check_error(optional_short_key_.find(short_key) == optional_short_key_.end(),
              "conflicting option string " + short_key);
  now_entry_.short_key = short_key;
  now_entry_.required = false;
  return *this;
}

inline ArgumentParser& ArgumentParser::positional_name(
    const std::string& name) {
  check_error(name.find("-") != 0, "invaild positionl string " + name);
  check_error(now_entry_.short_key.empty() && now_entry_.long_key.empty(),
              "invalid positional string " + name +
                  ": must not start with a character -");
  check_error(positional_key_.find(name) == positional_key_.end(),
              "conflicting positionl string " + name);
  check_error(now_entry_.key.empty(),
              "dest supplied twice for positional argument");
  now_entry_.positional_key = name;
  now_entry_.required = true;
  return *this;
}

// default
inline ArgumentParser& ArgumentParser::default_value(
    const std::string& default_value) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::STRING,
              "not boolean type");
  now_entry_.value_type = ValueType::STRING;
  now_entry_.default_value = EntryItem(default_value);
  return *this;
}
inline ArgumentParser& ArgumentParser::default_value(const char* temp) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::STRING,
              "not boolean type");
  now_entry_.value_type = ValueType::STRING;
  std::string default_value = temp;
  now_entry_.default_value = EntryItem(default_value);
  return *this;
}
inline ArgumentParser& ArgumentParser::default_value(const int default_value) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::DIGITS_INT,
              "not boolean type");
  now_entry_.value_type = ValueType::DIGITS_INT;
  now_entry_.default_value = EntryItem(default_value);
  return *this;
}
inline ArgumentParser& ArgumentParser::default_value(
    const double default_value) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::DIGITS_DOUBLE,
              "not boolean type");
  now_entry_.value_type = ValueType::DIGITS_DOUBLE;
  now_entry_.default_value = EntryItem(default_value);
  return *this;
}
inline ArgumentParser& ArgumentParser::default_value(const bool default_value) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::BOOLEAN,
              "not boolean type");
  now_entry_.value_type = ValueType::BOOLEAN;
  now_entry_.default_value = EntryItem(default_value);
  return *this;
}
inline ArgumentParser& ArgumentParser::default_value(
    const std::vector<std::string>& default_value) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::STRING,
              "not boolean type");
  now_entry_.value_type = ValueType::STRING;
  now_entry_.default_value = EntryItem(default_value);
  return *this;
}
inline ArgumentParser& ArgumentParser::default_value(
    const std::vector<int>& default_value) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::DIGITS_INT,
              "not boolean type");
  now_entry_.value_type = ValueType::DIGITS_INT;
  now_entry_.default_value = EntryItem(default_value);
  return *this;
}
inline ArgumentParser& ArgumentParser::default_value(
    const std::vector<double>& default_value) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::DIGITS_DOUBLE,
              "not boolean type");
  now_entry_.value_type = ValueType::DIGITS_DOUBLE;
  now_entry_.default_value = EntryItem(default_value);
  return *this;
}
inline ArgumentParser& ArgumentParser::default_value(
    const std::vector<bool>& default_value) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::BOOLEAN,
              "not boolean type");
  now_entry_.value_type = ValueType::BOOLEAN;
  now_entry_.default_value = EntryItem(default_value);
  return *this;
}

// const - if nargs='?' or action='store_const'
inline ArgumentParser& ArgumentParser::const_value(
    const std::string& const_value) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::STRING,
              "not boolean type");
  now_entry_.value_type = ValueType::STRING;
  if (now_entry_.nargs == NargsType::NARGS_QUESTION) {
    now_entry_.const_value = EntryItem(const_value);
  } else if (now_entry_.action == ActionType::ACTION_STORE_CONST) {
    now_entry_.const_value = EntryItem(const_value);
  } else {
    check_error(false,
                "nargs must be '?' or action=store_const to supply const");
  }
  return *this;
}
inline ArgumentParser& ArgumentParser::const_value(const char* temp) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::STRING,
              "not boolean type");
  now_entry_.value_type = ValueType::STRING;
  std::string const_value = temp;
  if (now_entry_.nargs == NargsType::NARGS_QUESTION) {
    now_entry_.const_value = EntryItem(const_value);
  } else if (now_entry_.action == ActionType::ACTION_STORE_CONST) {
    now_entry_.const_value = EntryItem(const_value);
  } else {
    check_error(false,
                "nargs must be '?' or action=store_const to supply const");
  }
  return *this;
}
inline ArgumentParser& ArgumentParser::const_value(const int const_value) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::DIGITS_INT,
              "not boolean type");
  now_entry_.value_type = ValueType::DIGITS_INT;
  if (now_entry_.nargs == NargsType::NARGS_QUESTION) {
    now_entry_.const_value = EntryItem(const_value);
  } else if (now_entry_.action == ActionType::ACTION_STORE_CONST) {
    now_entry_.const_value = EntryItem(const_value);
  } else {
    check_error(false,
                "nargs must be '?' or action=store_const to supply const");
  }
  return *this;
}
inline ArgumentParser& ArgumentParser::const_value(const double const_value) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::DIGITS_DOUBLE,
              "not boolean type");
  now_entry_.value_type = ValueType::DIGITS_DOUBLE;
  if (now_entry_.nargs == NargsType::NARGS_QUESTION) {
    now_entry_.const_value = EntryItem(const_value);
  } else if (now_entry_.action == ActionType::ACTION_STORE_CONST) {
    now_entry_.const_value = EntryItem(const_value);
  } else {
    check_error(false,
                "nargs must be '?' or action=store_const to supply const");
  }
  return *this;
}
inline ArgumentParser& ArgumentParser::const_value(const bool const_value) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::BOOLEAN,
              "not boolean type");
  now_entry_.value_type = ValueType::BOOLEAN;
  if (now_entry_.nargs == NargsType::NARGS_QUESTION) {
    now_entry_.const_value = EntryItem(const_value);
  } else if (now_entry_.action == ActionType::ACTION_STORE_CONST) {
    now_entry_.const_value = EntryItem(const_value);
  } else {
    check_error(false,
                "nargs must be '?' or action=store_const to supply const");
  }
  return *this;
}
inline ArgumentParser& ArgumentParser::const_value(
    const std::vector<std::string> const_value) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::STRING,
              "not boolean type");
  now_entry_.value_type = ValueType::STRING;
  if (now_entry_.nargs == NargsType::NARGS_QUESTION) {
    now_entry_.const_value = EntryItem(const_value);
  } else if (now_entry_.action == ActionType::ACTION_STORE_CONST) {
    now_entry_.const_value = EntryItem(const_value);
  } else {
    check_error(false,
                "nargs must be '?' or action=store_const to supply const");
  }
  return *this;
}
inline ArgumentParser& ArgumentParser::const_value(
    const std::vector<int> const_value) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::DIGITS_INT,
              "not boolean type");
  now_entry_.value_type = ValueType::DIGITS_INT;
  if (now_entry_.nargs == NargsType::NARGS_QUESTION) {
    now_entry_.const_value = EntryItem(const_value);
  } else if (now_entry_.action == ActionType::ACTION_STORE_CONST) {
    now_entry_.const_value = EntryItem(const_value);
  } else {
    check_error(false,
                "nargs must be '?' or action=store_const to supply const");
  }
  return *this;
}
inline ArgumentParser& ArgumentParser::const_value(
    const std::vector<double> const_value) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::DIGITS_DOUBLE,
              "not boolean type");
  now_entry_.value_type = ValueType::DIGITS_DOUBLE;
  if (now_entry_.nargs == NargsType::NARGS_QUESTION) {
    now_entry_.const_value = EntryItem(const_value);
  } else if (now_entry_.action == ActionType::ACTION_STORE_CONST) {
    now_entry_.const_value = EntryItem(const_value);
  } else {
    check_error(false,
                "nargs must be '?' or action=store_const to supply const");
  }
  return *this;
}
inline ArgumentParser& ArgumentParser::const_value(
    const std::vector<bool> const_value) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::BOOLEAN,
              "not boolean type");
  now_entry_.value_type = ValueType::BOOLEAN;
  if (now_entry_.nargs == NargsType::NARGS_QUESTION) {
    now_entry_.const_value = EntryItem(const_value);
  } else if (now_entry_.action == ActionType::ACTION_STORE_CONST) {
    now_entry_.const_value = EntryItem(const_value);
  } else {
    check_error(false,
                "nargs must be '?' or action=store_const to supply const");
  }
  return *this;
}

// help
inline ArgumentParser& ArgumentParser::help(const std::string& help) {
  now_entry_.help = help;
  return *this;
}
inline ArgumentParser& ArgumentParser::help(const char* help) {
  now_entry_.help = help;
  return *this;
}

// nargs ? | + | * |  N
inline ArgumentParser& ArgumentParser::nargs(const std::string& nargs_type) {
  check_error(now_entry_.action == ActionType::NONE,
              "nargs error, unknown action");
  if (nargs_type == "*") {
    now_entry_.nargs = NargsType::NARGS_STAR;
  } else if (nargs_type == "+") {
    now_entry_.nargs = NargsType::NARGS_ADD;
  } else if (nargs_type == "?") {
    now_entry_.nargs = NargsType::NARGS_QUESTION;
  } else {
    check_error(false, nargs_type + " does not match nargs");
  }
  return *this;
}
inline ArgumentParser& ArgumentParser::nargs(const int nargs_number) {
  check_error(nargs_number > 0, "nargs for store actions must be > 0");
  now_entry_.nargs = NargsType::NARGS_N;
  now_entry_.nargs_number = nargs_number;
  return *this;
}

// action store_false / store_true / store_const
inline ArgumentParser& ArgumentParser::action(const std::string& store_para) {
  check_error(now_entry_.action == ActionType::NONE, "set action once!");
  check_error(
      now_entry_.choices_array.value_type() != ValueType::INT_ARRAY &&
          now_entry_.choices_array.value_type() != ValueType::STRING_ARRAY &&
          now_entry_.choices_array.value_type() != ValueType::BOOL_ARRAY &&
          now_entry_.choices_array.value_type() != ValueType::DOUBLE_ARRAY &&
          now_entry_.nargs == NargsType::NONE,
      "action error");
  if (store_para == "store_false") {
    now_entry_.action = ActionType::ACTION_STORE_FALSE;
  } else if (store_para == "store_true") {
    now_entry_.action = ActionType::ACTION_STORE_TRUE;
  } else if (store_para == "store_const") {
    now_entry_.action = ActionType::ACTION_STORE_CONST;
  } else {
    check_error(false, "set action error: unknown action " + store_para);
  }
  return *this;
}

// dest - rename key
inline ArgumentParser& ArgumentParser::dest(const std::string& dest) {
  check_error(now_entry_.positional_key.empty(),
              "dest supplied twice for positional argument");
  now_entry_.key = dest;
  return *this;
}

// choices
inline ArgumentParser& ArgumentParser::choices(
    const std::vector<std::string>& string_array) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::STRING,
              "not boolean type");
  now_entry_.value_type = ValueType::STRING;
  now_entry_.choices_array = EntryItem(string_array);
  return *this;
}
inline ArgumentParser& ArgumentParser::choices(
    const std::vector<int>& int_array) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::DIGITS_INT,
              "not boolean type");
  now_entry_.value_type = ValueType::DIGITS_INT;
  now_entry_.choices_array = EntryItem(int_array);
  return *this;
}
inline ArgumentParser& ArgumentParser::choices(
    const std::vector<double>& double_array) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::DIGITS_DOUBLE,
              "not boolean type");
  now_entry_.value_type = ValueType::DIGITS_DOUBLE;
  now_entry_.choices_array = EntryItem(double_array);
  return *this;
}
inline ArgumentParser& ArgumentParser::choices(
    const std::vector<bool>& bool_array) {
  check_error(now_entry_.value_type == ValueType::NONE ||
                  now_entry_.value_type == ValueType::BOOLEAN,
              "not boolean type");
  now_entry_.value_type = ValueType::BOOLEAN;
  now_entry_.choices_array = EntryItem(bool_array);
  return *this;
}

// type : std::string int float bool
inline ArgumentParser& ArgumentParser::type(const ValueType value_type) {
  now_entry_.value_type = value_type;
  return *this;
}

inline void ArgumentParser::init() {
  now_entry_.filled = false;
  now_entry_.required = false;
  now_entry_.short_key = "";
  now_entry_.long_key = "";
  now_entry_.positional_key = "";
  now_entry_.key = "";
  now_entry_.help = "";
  now_entry_.action = ActionType::NONE;
  now_entry_.nargs = NargsType::NONE;
  now_entry_.value_type = ValueType::NONE;
  now_entry_.value = EMPTY_ENTRY_ITEM;
  now_entry_.const_value = EMPTY_ENTRY_ITEM;
  now_entry_.default_value = EMPTY_ENTRY_ITEM;
  now_entry_.choices_array = EMPTY_ENTRY_ITEM;
}

inline void ArgumentParser::done() {
  if (now_entry_.key.empty()) {
    if (!now_entry_.short_key.empty()) {
      now_entry_.key = now_entry_.short_key;
    }
    if (!now_entry_.long_key.empty()) {
      now_entry_.key = now_entry_.long_key;
    }
    if (!now_entry_.positional_key.empty()) {
      now_entry_.key = now_entry_.positional_key;
    }
  }
  check_error(!now_entry_.key.empty(), "required positional argument");
  if (!now_entry_.short_key.empty()) {
    optional_short_key_[now_entry_.short_key] = 1;
  }
  if (!now_entry_.long_key.empty()) {
    optional_long_key_[now_entry_.long_key] = 1;
  }
  if (!now_entry_.positional_key.empty()) {
    positional_key_[now_entry_.positional_key] = 1;
  }
  // action
  check_error(now_entry_.action != ActionType::ACTION_STORE_CONST ||
                  now_entry_.const_value.value_type() != ValueType::NONE,
              "required positional argument: 'const'");
  param_entries_.push_back(now_entry_);
  now_entry_ = ParamEntry();
  init();
}

inline EntryItem ArgumentParser::parse(const std::string& input,
                                       const ValueType value_type) {
  if (value_type == ValueType::DIGITS_INT) {
    return EntryItem(std::atoi(input.c_str()));
  } else if (value_type == ValueType::DIGITS_DOUBLE) {
    return EntryItem(std::strtod(input.c_str(), nullptr));
  } else if (value_type == ValueType::BOOLEAN) {
    if (input == "true") {
      return EntryItem(true);
    } else {
      return EntryItem(false);
    }
  } else {
    return EntryItem(input);
  }
}

inline bool ArgumentParser::is_entry_item_equal(const EntryItem item1,
                                                const EntryItem item2) {
  if (item1.value_type() != item2.value_type()) {
    return false;
  }
  if (item1.is_int_array()) {
    auto item1_value = item1.int_array_value();
    auto item2_value = item2.int_array_value();
    if (item1_value.size() != item2_value.size()) {
      return false;
    }
    for (size_t i = 0; i < item2_value.size(); i++) {
      if (item1_value[i] != item2_value[i]) {
        return false;
      }
    }
    return true;
  }

  if (item1.is_double_array()) {
    auto item1_value = item1.double_array_value();
    auto item2_value = item2.double_array_value();
    if (item1_value.size() != item2_value.size()) {
      return false;
    }
    for (size_t i = 0; i < item2_value.size(); i++) {
      if (item1_value[i] != item2_value[i]) {
        return false;
      }
    }
    return true;
  }
  if (item1.is_string_array()) {
    auto item1_value = item1.string_array_value();
    auto item2_value = item2.string_array_value();
    if (item1_value.size() != item2_value.size()) {
      return false;
    }
    for (size_t i = 0; i < item2_value.size(); i++) {
      if (item1_value[i] != item2_value[i]) {
        return false;
      }
    }
    return true;
  }
  if (item1.is_bool_array()) {
    auto item1_value = item1.bool_array_value();
    auto item2_value = item2.bool_array_value();
    if (item1_value.size() != item2_value.size()) {
      return false;
    }
    for (size_t i = 0; i < item2_value.size(); i++) {
      if (item1_value[i] != item2_value[i]) {
        return false;
      }
    }
    return true;
  }
  if (item1.is_bool()) {
    if (item1.bool_value() != item2.bool_value()) {
      return false;
    }
  }
  if (item1.is_int()) {
    if (item1.digit_int_value() != item2.digit_int_value()) {
      return false;
    }
  }
  if (item1.is_double()) {
    if (item1.digit_double_value() != item2.digit_double_value()) {
      return false;
    }
  }
  if (item1.is_string()) {
    if (item1.string_value() != item2.string_value()) {
      return false;
    }
  }
  return true;
}

inline bool ArgumentParser::parse_choices(const EntryItem item_value,
                                          const ParamEntry& entry) {
  auto temp = entry.choices_array;
  if (!temp.is_int_array() && !temp.is_double_array() &&
      !temp.is_string_array() && !temp.is_bool_array()) {
    return true;
  }
  if (temp.is_int_array()) {
    auto temp_array = temp.int_array_value();
    for (auto item : temp_array) {
      if (item == item_value.digit_int_value()) {
        return true;
      }
    }
  }
  if (temp.is_double_array()) {
    auto temp_array = temp.double_array_value();
    for (auto item : temp_array) {
      if (item == item_value.digit_double_value()) {
        return true;
      }
    }
  }
  if (temp.is_string_array()) {
    auto temp_array = temp.string_array_value();
    for (auto& item : temp_array) {
      if (item == item_value.string_value()) {
        return true;
      }
    }
  }
  if (temp.is_bool_array()) {
    auto temp_array = temp.bool_array_value();
    for (auto item : temp_array) {
      if (item == item_value.bool_value()) {
        return true;
      }
    }
  }
  // 输出 EntryItem
  check_error(false, "argument " + entry.key +
                         ": invalid choice: " + item_value.dump());
  return false;
}

// positional p_input_index (--) / optional p_input_index + 1
inline void ArgumentParser::parse_nargs(ParamEntry& entry, int& p_input_index,
                                        const int argc, char* argv[]) {
  // p_input_index + 1
  if (entry.nargs == NargsType::NARGS_QUESTION) {  // nargs = '?'
    EntryItem item_result;
    std::string input;
    // optional
    if (p_input_index + 1 == argc) {
      entry.value = entry.const_value;
      return;
    }
    input = argv[p_input_index + 1];
    if (input.find("-") == 0) {
      entry.value = entry.const_value;
      return;
    }
    // const / default
    item_result = parse(input, entry.value_type);
    parse_choices(item_result, entry);
    p_input_index++;
    entry.value = item_result;
  } else {  // nargs='*' >= 0 / nargs='+'  >0 / nargs=N  N(>0)
    std::string input;
    int count;
    if (entry.nargs == NargsType::NARGS_ADD ||
        entry.nargs == NargsType::NARGS_N) {
      check_error(p_input_index + 1 < argc,
                  "the following arguments are required: " + entry.key);
      input = argv[p_input_index + 1];
      check_error(input.find("-") != 0,
                  "the following arguments are required: " + entry.key);
    }
    if (entry.nargs == NargsType::NARGS_N) {
      count = entry.nargs_number;
    } else {
      count = -1;
    }
    EntryItem::array item_result;
    while (true) {
      if (count == 0) {
        break;
      }
      if (p_input_index + 1 == argc) {
        check_error(count <= 0,
                    "the following arguments are required: " + entry.key);
        break;
      }
      std::string input = argv[p_input_index + 1];
      if (input.find("-") == 0) {
        check_error(count <= 0,
                    "the following arguments are required: " + entry.key);
        break;
      }
      p_input_index++;
      auto item_value = parse(input, entry.value_type);
      parse_choices(item_value, entry);
      item_result.push_back(item_value);
      count--;
    }
    if (item_result.empty()) {
      entry.value = EMPTY_ENTRY_ITEM;
    } else {
      if (item_result[0].is_int()) {
        std::vector<int> temp_result;
        for (auto item : item_result) {
          temp_result.push_back(item.digit_int_value());
        }
        entry.value = EntryItem(temp_result);
      } else if (item_result[0].is_double()) {
        std::vector<double> temp_result;
        for (auto item : item_result) {
          temp_result.push_back(item.digit_double_value());
        }
        entry.value = EntryItem(temp_result);
      } else if (item_result[0].is_string()) {
        std::vector<std::string> temp_result;
        for (auto item : item_result) {
          temp_result.push_back(item.string_value());
        }
        entry.value = EntryItem(temp_result);
      } else if (item_result[0].is_bool()) {
        std::vector<bool> temp_result;
        for (auto item : item_result) {
          temp_result.push_back(item.bool_value());
        }
        entry.value = EntryItem(temp_result);
      } else {
        entry.value = EMPTY_ENTRY_ITEM;
      }
    }
  }
}

// parse arguments
inline ArgValue ArgumentParser::parse_args_any_type(int argc, char* argv[]) {

  std::stable_sort(param_entries_.begin(), param_entries_.end(),
                   [](const ParamEntry& e1, const ParamEntry& e2) {
                     return e1.required > e2.required;
                   });
  std::vector<ParamEntry> param_entries(param_entries_.begin(),
                                        param_entries_.end());
  std::vector<int> positional_index;
  int p_entry_index = 0;
  for (auto& entry : param_entries) {
    if (!entry.short_key.empty()) {
      optional_short_key_[entry.short_key] = p_entry_index;
    }
    if (!entry.long_key.empty()) {
      optional_long_key_[entry.long_key] = p_entry_index;
    }
    if (!entry.positional_key.empty()) {
      positional_key_[entry.positional_key] = p_entry_index;
      positional_index.push_back(p_entry_index);
    }
    p_entry_index++;
  }

  p_entry_index = 0;
  int p_input_index = 1;
  while (p_input_index < argc) {
    std::string input = argv[p_input_index];
    if (input == "--help" || input == "-h") {
      print_help();
      exit(0);
    }
    // 先处理 positional
    // positional
    if (input.find("-") != static_cast<size_t>(0)) {
      while (static_cast<size_t>(p_entry_index) < positional_index.size()) {
        auto entry = param_entries[positional_index[p_entry_index]];
        if (entry.action != ActionType::NONE) {
          p_entry_index++;
        } else {
          break;
        }
      }
      check_error(static_cast<size_t>(p_entry_index) < positional_index.size(),
                  "unrecognized arguments: " + input);
      EntryItem item_result;
      auto& entry = param_entries[positional_index[p_entry_index]];
      if (entry.action != ActionType::NONE) {
        if (entry.action == ActionType::ACTION_STORE_FALSE) {
          entry.value = EntryItem(false);
        } else if (entry.action == ActionType::ACTION_STORE_TRUE) {
          entry.value = EntryItem(true);
        }
      } else if (entry.nargs != NargsType::NONE) {
        p_input_index--;
        parse_nargs(entry, p_input_index, argc, argv);
      } else {
        input = argv[p_input_index];
        // save value points array
        item_result = parse(input, entry.value_type);
        parse_choices(item_result, entry);
        entry.value = item_result;
      }
      entry.filled = true;
      p_input_index++;
      p_entry_index++;
    } else {
      // optional -f --foo
      int input_key = 0;
      if (input.find("--") == 0) {
        input = input.substr(2);
        check_error(optional_long_key_.find(input) != optional_long_key_.end(),
                    "unrecognized arguments: " + input);
        input_key = optional_long_key_[input];
      } else if (input.find("-") == 0) {
        input = input.substr(1);
        check_error(
            optional_short_key_.find(input) != optional_short_key_.end(),
            "unrecognized arguments: " + input);
        input_key = optional_short_key_[input];
      }
      auto& entry = param_entries[input_key];
      if (entry.action != ActionType::NONE) {
        if (entry.action == ActionType::ACTION_STORE_FALSE) {
          entry.value = EntryItem(false);
        } else if (entry.action == ActionType::ACTION_STORE_TRUE) {
          entry.value = EntryItem(true);
        } else if (entry.action == ActionType::ACTION_STORE_CONST) {
          entry.value = entry.const_value;
        }
      } else if (entry.nargs != NargsType::NONE) {
        parse_nargs(entry, p_input_index, argc, argv);
      } else {
        p_input_index++;
        check_error(p_input_index < argc,
                    "the following arguments are required: " + input);
        input = argv[p_input_index];
        auto item_result = parse(input, entry.value_type);
        parse_choices(item_result, entry);
        entry.value = item_result;
      }
      entry.filled = true;
      p_input_index++;
    }
  }

  for (auto& item : param_entries) {
    if (item.filled) {
      continue;
    }
    if (!item.positional_key.empty()) {
      if (item.nargs == NargsType::NARGS_QUESTION ||
          item.nargs == NargsType::NARGS_STAR) {
        item.value = item.default_value;
        item.filled = true;
      } else if (item.action != ActionType::NONE) {
        if (item.action == ActionType::ACTION_STORE_TRUE) {
          item.value = EntryItem(true);
          item.filled = true;
        } else if (item.action == ActionType::ACTION_STORE_FALSE) {
          item.value = EntryItem(false);
          item.filled = true;
        } else if (item.action == ActionType::ACTION_STORE_CONST) {
          item.value = item.const_value;
          item.filled = true;
        }
      } else {
        check_error(false, "the following arguments are required: " + item.key);
      }
    } else {  // optional
      if (item.default_value.value_type() != ValueType::NONE) {
        item.value = item.default_value;
        item.filled = true;
      } else if (item.action == ActionType::ACTION_STORE_TRUE) {
        item.value = EntryItem(false);
        item.filled = true;
      } else if (item.action == ActionType::ACTION_STORE_FALSE) {
        item.value = EntryItem(true);
        item.filled = true;
      } else if (item.action == ActionType::ACTION_STORE_CONST) {
        item.value = item.const_value;
        item.filled = true;
      }
    }
  }
  std::map<std::string, ParamEntry> result;
  for (auto& item : param_entries) {
    if (item.filled) {
      result[item.key] = item;
    }
  }
  return ArgValue(result);
}
// parse arguments
inline std::map<std::string, std::string> ArgumentParser::parse_args_string(
    int argc, char* argv[]) {

  std::stable_sort(param_entries_.begin(), param_entries_.end(),
                   [](const ParamEntry& e1, const ParamEntry& e2) {
                     return e1.required > e2.required;
                   });
  std::vector<ParamEntry> param_entries(param_entries_.begin(),
                                        param_entries_.end());
  std::vector<int> positional_index;
  int p_entry_index = 0;
  for (auto& entry : param_entries) {
    if (!entry.short_key.empty()) {
      optional_short_key_[entry.short_key] = p_entry_index;
    }
    if (!entry.long_key.empty()) {
      optional_long_key_[entry.long_key] = p_entry_index;
    }
    if (!entry.positional_key.empty()) {
      positional_key_[entry.positional_key] = p_entry_index;
      positional_index.push_back(p_entry_index);
    }
    p_entry_index++;
  }

  p_entry_index = 0;
  int p_input_index = 1;
  while (p_input_index < argc) {
    std::string input = argv[p_input_index];
    if (input == "--help" || input == "-h") {
      print_help();
      exit(0);
    }
    // positional
    if (input.find("-") != static_cast<size_t>(0)) {
      while (static_cast<size_t>(p_entry_index) < positional_index.size()) {
        auto entry = param_entries[positional_index[p_entry_index]];
        if (entry.action != ActionType::NONE) {
          p_entry_index++;
        } else {
          break;
        }
      }
      check_error(static_cast<size_t>(p_entry_index) < positional_index.size(),
                  "unrecognized arguments: " + input);
      EntryItem item_result;
      auto& entry = param_entries[positional_index[p_entry_index]];
      if (entry.action != ActionType::NONE) {
        if (entry.action == ActionType::ACTION_STORE_FALSE) {
          entry.value = EntryItem(false);
        } else if (entry.action == ActionType::ACTION_STORE_TRUE) {
          entry.value = EntryItem(true);
        }
      } else if (entry.nargs != NargsType::NONE) {
        p_input_index--;
        parse_nargs(entry, p_input_index, argc, argv);
      } else {
        input = argv[p_input_index];
        // save value points array
        item_result = parse(input, entry.value_type);
        parse_choices(item_result, entry);
        entry.value = item_result;
      }
      entry.filled = true;
      p_input_index++;
      p_entry_index++;
    } else {
      // optional -f --foo
      int input_key = 0;
      if (input.find("--") == 0) {
        input = input.substr(2);
        check_error(optional_long_key_.find(input) != optional_long_key_.end(),
                    "unrecognized arguments: " + input);
        input_key = optional_long_key_[input];
      } else if (input.find("-") == 0) {
        input = input.substr(1);
        check_error(
            optional_short_key_.find(input) != optional_short_key_.end(),
            "unrecognized arguments: " + input);
        input_key = optional_short_key_[input];
      }
      auto& entry = param_entries[input_key];
      if (entry.action != ActionType::NONE) {
        if (entry.action == ActionType::ACTION_STORE_FALSE) {
          entry.value = EntryItem(false);
        } else if (entry.action == ActionType::ACTION_STORE_TRUE) {
          entry.value = EntryItem(true);
        } else if (entry.action == ActionType::ACTION_STORE_CONST) {
          entry.value = entry.const_value;
        }
      } else if (entry.nargs != NargsType::NONE) {
        parse_nargs(entry, p_input_index, argc, argv);
      } else {
        p_input_index++;
        check_error(p_input_index < argc,
                    "the following arguments are required: " + input);
        input = argv[p_input_index];
        auto item_result = parse(input, entry.value_type);
        parse_choices(item_result, entry);
        entry.value = item_result;
      }
      entry.filled = true;
      p_input_index++;
    }
  }

  for (auto& item : param_entries) {
    if (item.filled) {
      continue;
    }
    if (!item.positional_key.empty()) {
      if (item.nargs == NargsType::NARGS_QUESTION ||
          item.nargs == NargsType::NARGS_STAR) {
        item.value = item.default_value;
        item.filled = true;
      } else if (item.action != ActionType::NONE) {
        if (item.action == ActionType::ACTION_STORE_TRUE) {
          item.value = EntryItem(true);
          item.filled = true;
        } else if (item.action == ActionType::ACTION_STORE_FALSE) {
          item.value = EntryItem(false);
          item.filled = true;
        } else if (item.action == ActionType::ACTION_STORE_CONST) {
          item.value = item.const_value;
          item.filled = true;
        }
      } else {
        check_error(false, "the following arguments are required: " + item.key);
      }
    } else {  // optional
      if (item.default_value.value_type() != ValueType::NONE) {
        item.value = item.default_value;
        item.filled = true;
      } else if (item.action == ActionType::ACTION_STORE_TRUE) {
        item.value = EntryItem(false);
        item.filled = true;
      } else if (item.action == ActionType::ACTION_STORE_FALSE) {
        item.value = EntryItem(true);
        item.filled = true;
      } else if (item.action == ActionType::ACTION_STORE_CONST) {
        item.value = item.const_value;
        item.filled = true;
      }
    }
  }
  std::map<std::string, std::string> result;
  for (auto& item : param_entries) {
    if (item.filled) {
      result[item.key] = item.value.string_value();
    }
  }
  return result;
}

inline void ArgumentParser::check_error(bool condition,
                                        const std::string& error_msg) {
  if (!condition) {
    print_help();
    if (!error_msg.empty()) {
      fprintf(stderr, "[error]: %s\n", error_msg.c_str());
    }
    if (error_mode_ == ErrorMode::EXIT) {
      exit(1);
    } else {
      throw Exception(Status::PERF_FAILURE, error_msg);
    }
  }
}

// print help
inline void ArgumentParser::print_help() {
  auto& entries = param_entries_;
  std::sort(entries.begin(), entries.end(),
            [](const ParamEntry& e1, const ParamEntry& e2) {
              return e1.required > e2.required;
            });
  printf("Usage: \n");
  size_t i = 0;
  if (i < entries.size() && entries[i].required) {
    printf("positional arguments: \n");
    while (i < entries.size() && entries[i].required) {
      printf("%s", entries[i].key.c_str());
      if (false == entries[i].help.empty()) {
        printf(" HELP %s", entries[i].help.c_str());
      }
      if (entries[i].choices_array.value_type() == ValueType::INT_ARRAY ||
          entries[i].choices_array.value_type() == ValueType::DOUBLE_ARRAY ||
          entries[i].choices_array.value_type() == ValueType::STRING_ARRAY ||
          entries[i].choices_array.value_type() == ValueType::BOOL_ARRAY) {
        printf(" CHOOSE FROM %s", entries[i].choices_array.dump().c_str());
      }
      if (!entries[i].default_value.string_value().empty()) {
        printf(" DEFAULT %s", entries[i].default_value.string_value().c_str());
      }
      printf("\n");
      i++;
    }
  }
  if (i < entries.size()) {
    printf("optional arguments: \n");
    while (i < entries.size()) {
      bool exist_prefix = false;
      if (!entries[i].short_key.empty()) {
        exist_prefix = true;
        printf("-%s", entries[i].short_key.c_str());
      }
      if (!entries[i].long_key.empty()) {
        if (exist_prefix) {
          printf(", --%s", entries[i].long_key.c_str());
        } else {
          printf("--%s", entries[i].long_key.c_str());
        }
      }
      if (false == entries[i].help.empty()) {
        printf(" HELP %s", entries[i].help.c_str());
      }
      if (entries[i].choices_array.value_type() == ValueType::INT_ARRAY ||
          entries[i].choices_array.value_type() == ValueType::DOUBLE_ARRAY ||
          entries[i].choices_array.value_type() == ValueType::STRING_ARRAY ||
          entries[i].choices_array.value_type() == ValueType::BOOL_ARRAY) {
        printf(" CHOOSE FROM %s", entries[i].choices_array.dump().c_str());
      }
      if (!entries[i].default_value.string_value().empty()) {
        printf(" DEFAULT %s", entries[i].default_value.string_value().c_str());
      }
      printf("\n");
      i++;
    }
  }
}
}  // namespace argparse
