#ifndef REBOOT_APP_HPP
#define REBOOT_APP_HPP

#include "FreeRTOS.h"
#include "fio/unistd.hpp"
#include "semphr.h"
#include "task.h"

#include "drivers/io/rcc_ioctl.hpp"
#include "fio/unistd.hpp"

struct reboot_app_s {
  static void entry(void *);
  static int32_t reboot_printfmt(const char *, ...);
};

#endif /* REBOOT_APP_HPP */
