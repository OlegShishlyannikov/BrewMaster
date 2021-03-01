#ifndef DEBUG_APP_HPP
#define DEBUG_APP_HPP

#include "FreeRTOS.h"
#include "fio/unistd.hpp"
#include "semphr.h"
#include "task.h"

#include "fio/unistd.hpp"

struct debug_app_s {
  static void entry(void *);
  static int32_t debug_printfmt(const char *, ...);
private:
  static char **split_string_(char *, const char *);
};

#endif /* DEBUG_APP_HPP */
