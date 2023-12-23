#pragma once

/**
 * @brief Macro definition for class feature including
 *        DISALLOW_COPY, DISALLOW_ASSIGN, DISALLOW_COPY_AND_ASSIGN,
 *        DISALLOW_IMPLICIT_CONSTRUCTORS and DECLARE_SINGLETON
 */
#define DISALLOW_COPY(ClassName)                                               \
public:                                                                        \
  ClassName(ClassName const &) = delete;

#define DISALLOW_ASSIGN(ClassName)                                             \
public:                                                                        \
  void operator=(ClassName const &) = delete;

#define DISALLOW_COPY_AND_ASSIGN(ClassName)                                    \
public:                                                                        \
  ClassName(ClassName const &) = delete;                                       \
  void operator=(ClassName const &) = delete;

#define DISALLOW_IMPLICIT_CONSTRUCTORS(ClassName)                              \
private:                                                                       \
  ClassName() = default;                                                       \
  DISALLOW_COPY_AND_ASSIGN(ClassName);

#define DECLARE_SINGLETON(ClassName)                                           \
public:                                                                        \
  static ClassName *Instance() {                                               \
    static ClassName instance;                                                 \
    return &instance;                                                          \
  }                                                                            \
  DISALLOW_IMPLICIT_CONSTRUCTORS(ClassName);                                   \
                                                                               \
private:
