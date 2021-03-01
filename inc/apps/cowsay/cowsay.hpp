#ifndef COWSAY_APP_HPP
#define COWSAY_APP_HPP

#include "FreeRTOS.h"
#include "fio/unistd.hpp"
#include "semphr.h"
#include "task.h"

#include "drivers/io/ac_load_ioctl.hpp"
#include "drivers/io/ads1118_ioctl.hpp"
#include "drivers/io/button_event.hpp"
#include "drivers/io/button_ioctl.hpp"
#include "drivers/io/gpio_ioctl.hpp"
#include "drivers/io/hd44780_ioctl.hpp"
#include "drivers/io/leds_ioctl.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "drivers/io/relay_load_ioctl.hpp"
#include "drivers/io/timer_ioctl.hpp"
#include "drivers/io/usart_ioctl.hpp"
#include "drivers/io/w25qxx_ioctl.hpp"
#include "fio/unistd.hpp"

struct cowsay_app_s {
  static void entry(void *);
  static int32_t cowsay_printfmt(const char *, ...);
  static int32_t printfmt(const char *, ...);

private:
  static char **split_string_(char *, const char *);
};

#endif /* COWSAY_APP_HPP */
