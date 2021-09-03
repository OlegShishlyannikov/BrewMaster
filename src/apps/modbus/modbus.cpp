#include <cstdarg>

#include "apps/modbus/modbus.hpp"
#include "apps/shell/vt100.hpp"
#include "sys/system.hpp"

extern struct sys_impl_s &sys;
extern bool debug_log_enabled;

static constexpr const char *console_driver_name = "usart";
static constexpr const char *console_device_name = "usart1";
static constexpr const char *console_device_path = "usart/usart1";

static uint8_t vfd_device_id = 10u;
static uint16_t vfd_min_freq = 35u;
static uint16_t vfd_max_freq = 500u;
static uint16_t vfd_acceleration_time = 2u;
static uint16_t vfd_deacceleration_time = 2u;
static uint32_t vfd_ctrl_reg_addr = 0x2000u;
static uint32_t vfd_freq_reg_addr = 0x2001u;
static uint32_t vfd_accel_time_reg_addr = 0x2002u;
static uint32_t vfd_deaccel_time_reg_addr = 0x2003u;
static uint16_t vfd_start_cmd = 2u;
static uint16_t vfd_stop_cmd = 1u;
static uint16_t change_dir_cmd = 0x30;

static void on_seq_recvd(const void *, size_t);
static void on_protocol_err(uint8_t);

void modbus_app_s::entry(void *args) {
  char **arglist;
  int32_t modbus_fd, rc;

  struct modbus_cbk_req_s modbus_cbk_req {
    .cbk = on_seq_recvd
  };

  struct modbus_err_cbk_req_s modbus_err_cbk_req {
    .cbk = on_protocol_err
  };

  struct modbus_req_s read_request {
    .device_id = vfd_device_id, .func_code = modbus_func_code_e::MODBUS_RTU_READ_AO, .reg_addr = 0x0001, .data = nullptr, .size = 1u
  };

  struct modbus_req_s write_request {
    .device_id = vfd_device_id, .func_code = modbus_func_code_e::MODBUS_RTU_WRITE_AO, .reg_addr = static_cast<uint16_t>(vfd_ctrl_reg_addr)
  };

  // Set console char recvd callback
  if ((modbus_fd = ::open(&sys, "modbus/modbus0", 3, 2u)) < 0) {
    modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, "modbus/modbus0", modbus_ioctl_cmd_e::MODBUS_RTU_ON_SEQ_RECVD, &modbus_cbk_req, sizeof(modbus_cbk_req))) < 0) {
    if ((rc = ::close(&sys, "modbus/modbus0")) < 0) {
      modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, "modbus/modbus0", modbus_ioctl_cmd_e::MODBUS_RTU_ON_ERR, &modbus_err_cbk_req, sizeof(modbus_err_cbk_req))) < 0) {
    if ((rc = ::close(&sys, "modbus/modbus0")) < 0) {
      modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

loop:
  while (true) {

    if ((rc = ::ioctl(&sys, "modbus/modbus0", modbus_ioctl_cmd_e::MODBUS_QUERY, &read_request, sizeof(read_request))) < 0) {
      if ((rc = ::close(&sys, "modbus/modbus0")) < 0) {
        modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    // Set freq to minimum
    write_request.data = reinterpret_cast<uint8_t *>(&vfd_min_freq);
    write_request.size = sizeof(vfd_min_freq);
    write_request.reg_addr = vfd_freq_reg_addr;

    if ((rc = ::ioctl(&sys, "modbus/modbus0", modbus_ioctl_cmd_e::MODBUS_QUERY, &write_request, sizeof(write_request))) < 0) {
      if ((rc = ::close(&sys, "modbus/modbus0")) < 0) {
        modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    vTaskDelay(100u);

    // Set accel/deaccel time to minimum
    write_request.data = reinterpret_cast<uint8_t *>(&vfd_acceleration_time);
    write_request.size = sizeof(vfd_acceleration_time);
    write_request.reg_addr = vfd_accel_time_reg_addr;

    if ((rc = ::ioctl(&sys, "modbus/modbus0", modbus_ioctl_cmd_e::MODBUS_QUERY, &write_request, sizeof(write_request))) < 0) {
      if ((rc = ::close(&sys, "modbus/modbus0")) < 0) {
        modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    vTaskDelay(100u);

    // Set accel/deaccel time to minimum
    write_request.data = reinterpret_cast<uint8_t *>(&vfd_deacceleration_time);
    write_request.size = sizeof(vfd_deacceleration_time);
    write_request.reg_addr = vfd_deaccel_time_reg_addr;

    if ((rc = ::ioctl(&sys, "modbus/modbus0", modbus_ioctl_cmd_e::MODBUS_QUERY, &write_request, sizeof(write_request))) < 0) {
      if ((rc = ::close(&sys, "modbus/modbus0")) < 0) {
        modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    vTaskDelay(100u);

    write_request.data = reinterpret_cast<uint8_t *>(&vfd_start_cmd);
    write_request.size = sizeof(vfd_start_cmd);
    write_request.reg_addr = vfd_ctrl_reg_addr;

    if ((rc = ::ioctl(&sys, "modbus/modbus0", modbus_ioctl_cmd_e::MODBUS_QUERY, &write_request, sizeof(write_request))) < 0) {
      if ((rc = ::close(&sys, "modbus/modbus0")) < 0) {
        modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    vTaskDelay(100u);

    // Increase freq
    for (uint32_t i = vfd_min_freq + 1u; i < vfd_max_freq; i++) {
      uint16_t freq = vfd_min_freq + i;
      write_request.data = reinterpret_cast<uint8_t *>(&freq);
      write_request.size = sizeof(vfd_min_freq);
      write_request.reg_addr = vfd_freq_reg_addr;

      if ((rc = ::ioctl(&sys, "modbus/modbus0", modbus_ioctl_cmd_e::MODBUS_QUERY, &write_request, sizeof(write_request))) < 0) {
        if ((rc = ::close(&sys, "modbus/modbus0")) < 0) {
          modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
          goto error;
        }

        modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      vTaskDelay(100u);
    }

    write_request.data = reinterpret_cast<uint8_t *>(&change_dir_cmd);
    write_request.size = sizeof(change_dir_cmd);
    write_request.reg_addr = vfd_ctrl_reg_addr;

    if ((rc = ::ioctl(&sys, "modbus/modbus0", modbus_ioctl_cmd_e::MODBUS_QUERY, &write_request, sizeof(write_request))) < 0) {
      if ((rc = ::close(&sys, "modbus/modbus0")) < 0) {
        modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    vTaskDelay(100u);

    // Decrease freq
    for (uint32_t i = vfd_max_freq; i != vfd_min_freq; i--) {
      uint16_t freq = i;
      write_request.data = reinterpret_cast<uint8_t *>(&freq);
      write_request.size = sizeof(vfd_min_freq);
      write_request.reg_addr = vfd_freq_reg_addr;

      if ((rc = ::ioctl(&sys, "modbus/modbus0", modbus_ioctl_cmd_e::MODBUS_QUERY, &write_request, sizeof(write_request))) < 0) {
        if ((rc = ::close(&sys, "modbus/modbus0")) < 0) {
          modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
          goto error;
        }

        modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      vTaskDelay(100u);
    }

    write_request.data = reinterpret_cast<uint8_t *>(&vfd_stop_cmd);
    write_request.size = sizeof(vfd_stop_cmd);
    write_request.reg_addr = vfd_ctrl_reg_addr;

    if ((rc = ::ioctl(&sys, "modbus/modbus0", modbus_ioctl_cmd_e::MODBUS_QUERY, &write_request, sizeof(write_request))) < 0) {
      if ((rc = ::close(&sys, "modbus/modbus0")) < 0) {
        modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    vTaskDelay(100u);
  }

  if ((rc = ::close(&sys, "modbus/modbus0")) < 0) {
    modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

exit:
error:
  vTaskDelete(nullptr);
}

char **modbus_app_s::split_string_(char *str, const char *delim) {
  char **res = nullptr, *p;
  int32_t n = 0;

  if (!str || !delim) {
    return nullptr;
  }

  p = std::strtok(str, delim);

  while (p) {
    n++;
    res = static_cast<char **>(realloc(res, sizeof(char **) * n));
    res[n - 1u] = p;
    p = std::strtok(nullptr, " ");
  }

  // Terminate list
  res = static_cast<char **>(realloc(res, sizeof(char **) * n + 1u));
  res[n] = nullptr;
  return res;
}

int32_t modbus_app_s::printfmt(const char *fmt, ...) {
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

  free(temp);
  return strlen;
error:

  free(temp);
  return -1;
}

int32_t modbus_app_s::modbus_dbg_nprint(void *ptr, size_t n) {
  int32_t rc, usart_fd;
  const struct drv_model_cmn_s *usart = sys.drv(console_driver_name);

  if ((usart_fd = ::open(usart, console_device_name, 3, 3u)) < 0) {
    modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::write(usart, usart_fd, "[modbus] : ", std::strlen("[modbus] : "))) < 0) {
    if ((rc = ::close(usart, usart_fd)) < 0) {
      modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
    modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::write(usart, usart_fd, ptr, n)) < 0) {
    if ((rc = ::close(usart, usart_fd)) < 0) {
      modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
    modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(usart, usart_fd)) < 0) {
    if ((rc = ::close(usart, usart_fd)) < 0) {
      modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
    modbus_app_s::modbus_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return n;
error:
  return -1;
}

int32_t modbus_app_s::modbus_dbg_printfmt(const char *fmt, ...) {
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
    if ((usart_fd = ::open(usart, console_device_name, 3, 3u)) < 0) {
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, "[modbus] : ", std::strlen("[modbus] : "))) < 0) {
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

static void on_seq_recvd(const void *data, size_t size) {}

static void on_protocol_err(uint8_t err) {}
