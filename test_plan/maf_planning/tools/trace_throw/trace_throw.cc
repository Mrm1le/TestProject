#include <dlfcn.h>
#include <stdio.h>
#include "stacktrace.h"

extern "C" {

typedef void (*FuncCxaThrow)(void* thrown_exception, struct type_info* tinfo,
                             void (*dest)(void*));
FuncCxaThrow real_cxa_throw = 0;

int init() {
  real_cxa_throw = (FuncCxaThrow)dlsym(RTLD_NEXT, "__cxa_throw");
  return 0;
}

void __cxa_throw(void* thrown_exception, struct type_info* tinfo,
                 void (*dest)(void*)) {
  static bool initialized = init();
  (void)initialized;
  fprintf(stderr, "__cxa_throw called\n");
  print_stacktrace();
  real_cxa_throw(thrown_exception, tinfo, dest);
}
}
