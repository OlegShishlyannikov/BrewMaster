#ifndef MODBUS_APP_HPP
#define MODBUS_APP_HPP

#include "FreeRTOS.h"
#include "fio/unistd.hpp"
#include "semphr.h"
#include "task.h"

#include "drivers/io/buzzer_ioctl.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "drivers/io/usart_ioctl.hpp"
#include "drivers/io/modbus_ioctl.hpp"
#include "fio/unistd.hpp"

struct modbus_app_s {
  static void entry(void *);
  static int32_t modbus_dbg_printfmt(const char *, ...);
  static int32_t modbus_dbg_nprint(void *, size_t);
  static int32_t printfmt(const char *, ...);

private:
  static char **split_string_(char *, const char *);
};

#endif /* MODBUS_APP_HPP */
