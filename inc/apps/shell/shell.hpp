#ifndef SHELL_APP_HPP
#define SHELL_APP_HPP

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

enum shell_fsm_state_e : uint8_t {
  SHELL_APP_FSM_STATE_STARTUP = 0u,
  SHELL_APP_FSM_STATE_WAITING,
  SHELL_APP_FSM_STATE_IDLE,
  SHELL_APP_FSM_STATE_ESC_SEQ_HANDLING,
  SHELL_APP_FSM_STATE_CURSOR_POS_UPDATING
};

struct shell_app_s {
  static void entry(void *);
  static void console_char_recvd_callback(const void *, size_t);
  static int32_t exec(const char *, int, char **);
  static int32_t shell_printfmt(const char *, ...);

private:
  static bool starts_with_(const char *, const char *, size_t);
  static bool ends_with_(const char *, const char *, size_t);
  static char **split_string_(char *, const char *);
  static char *append_char_(char *, const char *, size_t);
  static char *insert_char_(char *, const char *, size_t, size_t);
  static char *remove_char_left_(char *, size_t);
  static char *remove_char_right_(char *, size_t);
  static bool update_cursor_pos_(uint32_t *, uint32_t *, const char *);
  static void handle_esc_seq_(const char);
  static void handle_ctrl_char_(const char);
  static void handle_eol_char_(const char);
  static void handle_generic_ascii_(const char);
  static void handle_kbd_(const char);
};

#endif /* SHELL_APP_HPP */
