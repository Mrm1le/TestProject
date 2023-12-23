#ifndef LOCAL_LOG_H
#define LOCAL_LOG_H

#include <iostream>
#include <stdio.h>
#include <string>
// local log
// #define DLOCAL_LOG 1

#define LOCAL_INFO "INFO"
#define LOCAL_DEBUG "DEBUG"
#ifdef DLOCAL_LOG
#define LOCAL_LOG(prio, msg, ...)                                              \
  do {                                                                         \
    std::string str;                                                           \
    str = prio;                                                                \
    if (strcmp(prio, LOCAL_INFO)==0)                                                    \
      fprintf(stdout, "\033[0;32m[%s, %s: line%d]: " msg " \033[0m\n",         \
              str.c_str(), __FUNCTION__, __LINE__, ##__VA_ARGS__);             \
    else if (strcmp(prio, LOCAL_INFO)==0)                                              \
      fprintf(stdout, "\033[0;31m[%s, %s: line%d]: " msg " \033[0m\n",         \
              str.c_str(), __FUNCTION__, __LINE__, ##__VA_ARGS__);             \
    fflush(stdout);                                                            \
  } while (0)
#else
#define LOCAL_LOG(prio, msg, ...)                                              \
  do {                                                                         \
  } while (0)
#endif

#endif