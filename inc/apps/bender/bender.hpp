#ifndef BENDER_APP_HPP
#define BENDER_APP_HPP

#include "FreeRTOS.h"
#include "fio/unistd.hpp"
#include "semphr.h"
#include "task.h"

#include "drivers/io/button_event.hpp"
#include "drivers/io/button_ioctl.hpp"
#include "drivers/io/buzzer_ioctl.hpp"
#include "drivers/io/gpio_ioctl.hpp"
#include "drivers/io/leds_ioctl.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "drivers/io/ui_ioctl.hpp"
#include "drivers/io/uln2003_ioctl.hpp"
#include "drivers/io/zsp3806g2e_ioctl.hpp"
#include "drivers/io/74hc595_ioctl.hpp"
#include "drivers/io/relay_load_ioctl.hpp"
#include "drivers/io/usart_ioctl.hpp"
#include "fio/unistd.hpp"

struct bender_app_s {
  static void entry(void *);
  static int32_t bender_dbg_printfmt(const char *, ...);
  static int32_t dbg_printfmt(const char *, ...);
  static int32_t bender_dbg_nprint(void *, size_t);

private:
  static char **split_string_(char *, const char *);
};

#endif /* BENDER_APP_HPP */
