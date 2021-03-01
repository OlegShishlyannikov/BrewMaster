#ifndef CLEAR_APP_HPP
#define CLEAR_APP_HPP

#include "FreeRTOS.h"
#include "fio/unistd.hpp"
#include "semphr.h"
#include "task.h"

#include "fio/unistd.hpp"

struct clear_app_s {
  static void entry(void *);
  static int32_t clear_printfmt(const char *, ...);
};

#endif /* CLEAR_APP_HPP */
