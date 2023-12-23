#pragma once

#include "logging.h"
#include "struct.h"
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>

namespace msd_worldmodel {

class MSDException : public std::exception {
public:
  MSDException(MSDStatus status) : error_(status) {
    msg_ = "MSD Exception " + std::to_string(error_);
  };
  const char *what() const noexcept { return msg_.c_str(); };
  MSDStatus get_status() { return error_; };

private:
  MSDStatus error_ = MSDStatus::MSD_FAILURE;
  std::string msg_;
};

#if defined(MSD_ENABLE_EXCEPTION)
#define MSD_THROW(e) throw(e)
#else
#define MSD_THROW(e) std::abort()
#endif

#define MSD_THROW_EXCEPTION_IF(cond, e)                                        \
  do {                                                                         \
    if ((cond))                                                                \
      MSD_THROW(e);                                                            \
  } while (0)

/* ================ helper macros ================  */
//! throw MpaException{\p _fail} if \p _cond evaluates to false; also log \p
//! _msg
#define MSD_CHECK_THROW(_cond, _fail, ...)                                     \
  do {                                                                         \
    if (!(_cond)) {                                                            \
      MSD_LOG(ERROR, __VA_ARGS__);                                             \
      MSD_THROW(MSDException{_fail});                                          \
    }                                                                          \
  } while (0)

//! return \p _fail immediately and log \p _msg
#define MSD_THROW_FAILURE(_fail, ...)                                          \
  do {                                                                         \
    MSD_LOG(ERROR, __VA_ARGS__);                                               \
    MSD_THROW(MSDException{_fail});                                            \
  } while (0)

//! return \p _expr immediately if it is not MSD_OK
#define MSD_CHECK_RET(_expr)                                                   \
  do {                                                                         \
    MSDStatus _s = (_expr);                                                    \
    if (_s != MSD_OK) {                                                        \
      MSD_LOG(ERROR, "Error: %d", _s);                                         \
      return _s;                                                               \
    }                                                                          \
  } while (0)

//! return \p _fail if \p _cond evaluates to false; also log \p _msg
#define MSD_CHECK_COND(_cond, _fail, ...)                                      \
  do {                                                                         \
    if (!(_cond)) {                                                            \
      MSD_LOG(ERROR, __VA_ARGS__);                                             \
      return _fail;                                                            \
    }                                                                          \
  } while (0)

//! return \p _fail immediately and log \p _msg
#define MSD_RETURN_FAILURE(_fail, ...)                                         \
  do {                                                                         \
    MSD_LOG(ERROR, __VA_ARGS__);                                               \
    return _fail;                                                              \
  } while (0)

#if defined(MSD_ENABLE_EXCEPTION)
#define MSD_ERROR_HANDLE_BEGIN try {
#define MSD_ERROR_HANDLE_END                                                   \
  }                                                                            \
  catch (MSDException & e) {                                                   \
    return e.get_status();                                                     \
  }                                                                            \
  catch (std::string msg) {                                                    \
    MSD_LOG(ERROR, "MSD", msg.c_str());                                        \
    return MSD_FAILURE;                                                        \
  }
#else
#define MSD_ERROR_HANDLE_BEGIN {
#define MSD_ERROR_HANDLE_END }
#endif

} // namespace msd_worldmodel

// vim: syntax=cpp.doxygen foldmethod=marker foldmarker=f{{{,f}}}
