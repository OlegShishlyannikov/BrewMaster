#include <cstdarg>

#include "apps/clear/clear.hpp"
#include "apps/shell/vt100.hpp"
#include "sys/system.hpp"

extern struct sys_impl_s &sys;
extern bool debug_log_enabled;

static constexpr const char *console_driver_name = "usart";
static constexpr const char *console_device_name = "usart2";
static constexpr const char *console_device_path = "usart/usart2";

void clear_app_s::entry(void *) { clear_printfmt("%s%s", ASCII_CONTROL_ERASE_SCREEN, ASCII_CONTROL_CURSOR_HOME); }

int32_t clear_app_s::clear_printfmt(const char *fmt, ...) {
  int32_t rc, usart_fd, strlen;
  static char *temp;
  const struct drv_model_cmn_s *usart;

  std::va_list arg;
  va_start(arg, fmt);
  size_t bufsz = std::vsnprintf(nullptr, 0u, fmt, arg);
  temp = static_cast<char *>(calloc(bufsz, sizeof(char)));
  strlen = std::vsprintf(temp, fmt, arg);
  va_end(arg);

  if (!(usart = sys.drv(console_driver_name))) {
    goto error;
  }

  if (strlen) {
    if ((usart_fd = ::open(usart, console_device_name, 3, 3u)) < 0) {
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, "[clear] : ", std::strlen("[clear] : "))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        goto error;
      }
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, temp, std::strlen(temp))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        goto error;
      }

      goto error;
    }

    if ((rc = ::close(usart, usart_fd)) < 0) {
      goto error;
    }
  }

exit:
  free(temp);
  return strlen;
error:

  free(temp);
  return -1;
}
