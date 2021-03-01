#ifndef BREWERY_APP_HPP
#define BREWERY_APP_HPP

#include "FreeRTOS.h"
#include "fio/unistd.hpp"
#include "semphr.h"
#include "task.h"

#include "drivers/io/ac_load_ioctl.hpp"
#include "drivers/io/ads1118_ioctl.hpp"
#include "drivers/io/button_event.hpp"
#include "drivers/io/button_ioctl.hpp"
#include "drivers/io/buzzer_ioctl.hpp"
#include "drivers/io/gpio_ioctl.hpp"
#include "drivers/io/hd44780_ioctl.hpp"
#include "drivers/io/leds_ioctl.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "drivers/io/relay_load_ioctl.hpp"
#include "drivers/io/rtc_ioctl.hpp"
#include "drivers/io/timer_ioctl.hpp"
#include "drivers/io/usart_ioctl.hpp"
#include "drivers/io/w25qxx_ioctl.hpp"
#include "fio/unistd.hpp"

enum lcd_align_e : uint32_t { LCD_ALIGN_LEFT = 0u, LCD_ALIGN_CENTER, LCD_ALIGN_RIGHT };

struct brewery_app_s {
  static void entry(void *);
  static int32_t brewery_dbg_printfmt(const char *, ...);
  static int32_t dbg_printfmt(const char *, ...);
  static int32_t brewery_dbg_nprint(void *, size_t);

private:
  static char **split_string_(char *, const char *);
};

#endif /* BREWERY_APP_HPP */
