#include <cstdarg>

#include "apps/reboot/reboot.hpp"
#include "apps/shell/vt100.hpp"
#include "sys/system.hpp"

extern struct sys_impl_s &sys;
extern bool debug_log_enabled;

static constexpr const char *console_driver_name = "usart";
static constexpr const char *console_device_name = "usart2";
static constexpr const char *console_device_path = "usart/usart2";

void reboot_app_s::entry(void *) {
  int32_t rc, rcc_fd;
  const struct drv_model_cmn_s *rcc;
  reboot_printfmt("Rebooting...\r\n");

  if (!(rcc = sys.drv("rcc"))) {
    goto exit;
  }

  if ((rc = ::open(&sys, "rcc/rcc0", 3, 2u)) < 0) {
    goto exit;
  }

  if ((rc = ::ioctl(&sys, "rcc/rcc0", rcc_drv_ioctl_cmd_e::RCC_REBOOT, nullptr, 0u)) < 0) {
    goto exit;
  }

  if ((rc = ::close(&sys, "rcc/rcc0")) < 0) {
    goto exit;
  }

exit:
  reboot_printfmt("Reboot error...\r\n");
}

int32_t reboot_app_s::reboot_printfmt(const char *fmt, ...) {
  int32_t rc, usart_fd, strlen;
  static char *temp;
  const struct drv_model_cmn_s *usart;

  std::va_list arg;
  va_start(arg, fmt);
  size_t bufsz = std::vsnprintf(nullptr, 0u, fmt, arg);
  temp = static_cast<char *>(calloc(bufsz, sizeof(char)));
  strlen = std::vsprintf(temp, fmt, arg);
  va_end(arg);

  if (!debug_log_enabled) {
    goto exit;
  }

  if (!(usart = sys.drv(console_driver_name))) {
    goto error;
  }

  if (strlen) {
    if ((usart_fd = ::open(usart, console_device_path, 3, 3u)) < 0) {
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, "[reboot] : ", std::strlen("[reboot] : "))) < 0) {
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
