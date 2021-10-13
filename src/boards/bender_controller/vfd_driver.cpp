#include <cstdarg>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "arch/mcu_tags.hpp"
#include "boards/board_tags.hpp"
#include "drivers/io/gpio_ioctl.hpp"
#include "drivers/io/modbus_ioctl.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "drivers/io/usart_ioctl.hpp"
#include "drivers/io/vfd_ioctl.hpp"
#include "drivers/vfd_driver.hpp"
#include "fio/unistd.hpp"
#include "sys/events_worker.hpp"

// TODO : 9.05 -> 0
// 0.03 -> 25.0
// 4.00 -> 1

/* This pointer will be used in interrupt handlers and will be initialized in driver init function */
static const struct drv_model_cmn_s *drv_ptr;
extern bool debug_log_enabled;

/* Latch, clock and data pins used */
static constexpr const char *vfd_drv_modbus_devstr = "modbus0";
static constexpr const char *console_devstr = "usart2";

/* 74HC595 lock & fifos */
static xSemaphoreHandle vfd_lock;
extern xQueueHandle events_worker_queue;

enum vfd_ctrl_reg_addr_e : uint64_t { VFD_CTRL_REG_ADDR = 0x1000u };
enum vfd_ctrl_reg_cmd_e : uint64_t { VFD_CTRL_RUN_FWD_CMD = 0x01u, VFD_CTRL_RUN_REV_CMD = 0x02u, VFD_CTRL_STOP_CMD = 0x05u };

static constexpr const uint16_t vfd_device_id = 1u;

// Printf to console
static int32_t vfd_printf(const char *, ...);

/* 74HC595 file IO functions forward reference */
static int32_t vfd_drv_open(int32_t, mode_t);
static int32_t vfd_drv_ioctl(uint64_t, const void *, size_t);
static int32_t vfd_drv_read(void *const, size_t);
static int32_t vfd_drv_write(const void *, size_t);
static int32_t vfd_drv_close();

/* 74HC595 helper functions */
static int32_t vfd_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreTakeRecursive(vfd_lock, 0u)) != pdPASS) {
    // errno = ???
    vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t vfd_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreGiveRecursive(vfd_lock)) != pdPASS) {
    // errno = ???
    vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

struct drv_ops_s vfd_drv_ops {
  .init = vfd_drv_init, .exit = vfd_drv_exit
};

/* 74HC595 driver file operations secification */
struct file_ops_s vfd_drv_fops {
  .flock = vfd_flock, .funlock = vfd_funlock, .open = vfd_drv_open, .ioctl = vfd_drv_ioctl, .read = vfd_drv_read, .write = vfd_drv_write, .close = vfd_drv_close
};

void vfd_drv_init(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *modbus;
  int32_t modbus_fd, rc;
  uint16_t cmd;

  struct modbus_req_s modbus_req {
    .device_id = vfd_device_id, .func_code = modbus_func_code_e::MODBUS_RTU_WRITE_AO
  };

  if (!(modbus = drv->dep("modbus"))) {
    vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((modbus_fd = ::open(modbus, vfd_drv_modbus_devstr, 3, 3u)) < 0) {
    vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  modbus_req.reg_addr = VFD_CTRL_REG_ADDR;
  cmd = VFD_CTRL_STOP_CMD;
  modbus_req.data = reinterpret_cast<uint8_t *>(&cmd);

  if ((rc = ::ioctl(modbus, modbus_fd, modbus_ioctl_cmd_e::MODBUS_QUERY, &modbus_req, sizeof(modbus_req))) < 0) {
    if ((rc = ::close(modbus, modbus_fd)) < 0) {
      vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(modbus, modbus_fd)) < 0) {
    vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  vfd_drv_fops.owner = drv;

  /* Init locks */
  vfd_lock = xSemaphoreCreateRecursiveMutex();
  xSemaphoreGive(vfd_lock);

  /* Register char device for each GPIO port */
  drv->register_chardev("vfd0", &vfd_drv_fops);

  /* Initialize GPIO driver pointer */
  drv_ptr = drv;
  return;

  /* Something went wrong -- reset drv_ptr and exit */
error:
  vfd_drv_exit(drv);
  return;
}

void vfd_drv_exit(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *modbus;
  int32_t rc, modbus_fd;
  uint16_t cmd;

  struct modbus_req_s modbus_req {
    .device_id = vfd_device_id, .func_code = modbus_func_code_e::MODBUS_RTU_WRITE_AO
  };

  if (!(modbus = drv->dep("modbus"))) {
    vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((modbus_fd = ::open(modbus, vfd_drv_modbus_devstr, 3, 3u)) < 0) {
    vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  modbus_req.reg_addr = VFD_CTRL_REG_ADDR;
  cmd = VFD_CTRL_STOP_CMD;
  modbus_req.data = reinterpret_cast<uint8_t *>(&cmd);

  if ((rc = ::ioctl(modbus, modbus_fd, modbus_ioctl_cmd_e::MODBUS_QUERY, &modbus_req, sizeof(modbus_req))) < 0) {
    if ((rc = ::close(modbus, modbus_fd)) < 0) {
      vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(modbus, modbus_fd)) < 0) {
    vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  vfd_drv_fops.owner = nullptr;

  /* Remove locks */
  vSemaphoreDelete(vfd_lock);

  /* Register char device for each GPIO port */
  drv->unregister_chardev("vfd0");

  /* Initialize GPIO driver pointer */
  drv_ptr = nullptr;
  return;
error:
  return;
}

/* 74HC595 file IO functions */
static int32_t vfd_drv_open(int32_t, mode_t) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t vfd_drv_ioctl(uint64_t req, const void *buf, size_t size) {
  const struct drv_model_cmn_s *modbus;
  int32_t rc, modbus_fd;
  uint16_t cmd;
  struct modbus_req_s modbus_req {
    .device_id = vfd_device_id, .func_code = modbus_func_code_e::MODBUS_RTU_WRITE_AO
  };

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(modbus = drv_ptr->dep("modbus"))) {
    vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((modbus_fd = ::open(modbus, vfd_drv_modbus_devstr, 3, 3u)) < 0) {
    vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (req) {

  case vfd_ioctl_cmd_e::VFD_STOP: {
    modbus_req.reg_addr = VFD_CTRL_REG_ADDR;
    cmd = VFD_CTRL_STOP_CMD;
    modbus_req.data = reinterpret_cast<uint8_t *>(&cmd);

    if ((rc = ::ioctl(modbus, modbus_fd, modbus_ioctl_cmd_e::MODBUS_QUERY, &modbus_req, sizeof(modbus_req))) < 0) {
      if ((rc = ::close(modbus, modbus_fd)) < 0) {
        vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case vfd_ioctl_cmd_e::VFD_RUN_FWD: {
    modbus_req.reg_addr = VFD_CTRL_REG_ADDR;
    cmd = VFD_CTRL_RUN_FWD_CMD;
    modbus_req.data = reinterpret_cast<uint8_t *>(&cmd);

    if ((rc = ::ioctl(modbus, modbus_fd, modbus_ioctl_cmd_e::MODBUS_QUERY, &modbus_req, sizeof(modbus_req))) < 0) {
      if ((rc = ::close(modbus, modbus_fd)) < 0) {
        vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case vfd_ioctl_cmd_e::VFD_RUN_REV: {
    modbus_req.reg_addr = VFD_CTRL_REG_ADDR;
    cmd = VFD_CTRL_RUN_REV_CMD;
    modbus_req.data = reinterpret_cast<uint8_t *>(&cmd);

    if ((rc = ::ioctl(modbus, modbus_fd, modbus_ioctl_cmd_e::MODBUS_QUERY, &modbus_req, sizeof(modbus_req))) < 0) {
      if ((rc = ::close(modbus, modbus_fd)) < 0) {
        vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;
  default:
    break;
  }

  if ((rc = ::close(modbus, modbus_fd)) < 0) {
    vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  if ((rc = ::close(modbus, modbus_fd)) < 0) {
    vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  }

  return -1;
}

static int32_t vfd_drv_read(void *const, size_t) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t vfd_drv_write(const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t vfd_drv_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    vfd_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t vfd_printf(const char *fmt, ...) {
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

  if (!(usart = drv_ptr)) {
    goto error;
  }

  if (strlen) {
    if ((usart_fd = ::open(usart, console_devstr, 3, 3u)) < 0) {
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, "[vfd] : ", std::strlen("[vfd] : "))) < 0) {
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
