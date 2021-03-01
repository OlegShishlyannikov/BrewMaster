#include <cstdarg>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "arch/mcu_tags.hpp"
#include "boards/board_tags.hpp"
#include "drivers/ac_load_driver.hpp"
#include "drivers/io/ac_load_ioctl.hpp"
#include "drivers/io/gpio_ioctl.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "fio/unistd.hpp"

static const struct drv_model_cmn_s *drv_ptr;
extern xQueueHandle events_worker_queue;
static constexpr const uint32_t acloads_num = 4u;
extern bool debug_log_enabled;

/* AC load locks */
static xSemaphoreHandle acload_locks[acloads_num];
static const uint8_t acload_gpio_pins[acloads_num]{0u, 1u, 2u, 8u};
enum ac_loads_e : uint32_t { ACLOAD0 = 0u, ACLOAD1, ACLOAD2, ACLOAD3 };

extern xQueueHandle events_worker_queue;

// Global flags
extern bool debug_log_enabled;

static int32_t ac_load_printf(const char *fmt, ...);
template <enum ac_loads_e acload_no> static int32_t ac_load_drv_open(int32_t, mode_t);
template <enum ac_loads_e acload_no> static int32_t ac_load_drv_ioctl(uint64_t, const void *, size_t);
template <enum ac_loads_e acload_no> static int32_t ac_load_drv_read(void *const, size_t);
template <enum ac_loads_e acload_no> static int32_t ac_load_drv_write(const void *, size_t);
template <enum ac_loads_e acload_no> static int32_t ac_load_drv_close();

/* AC loads helper functions */
template <enum ac_loads_e acload_no> static int32_t acload_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreTakeRecursive(acload_locks[acload_no], portIO_MAX_DELAY)) != pdPASS) {
    // errno = ???
    ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

template <enum ac_loads_e acload_no> static int32_t acload_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreGiveRecursive(acload_locks[acload_no])) != pdPASS) {
    // errno = ???
    ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

struct drv_ops_s ac_load_drv_ops {
  .init = ac_load_drv_init, .exit = ac_load_drv_exit
};

/* AC load driver file operations secification */
struct file_ops_s acload_drv_acload0_fops {
  .flock = acload_flock<ACLOAD0>, .funlock = acload_funlock<ACLOAD0>, .open = ac_load_drv_open<ACLOAD0>, .ioctl = ac_load_drv_ioctl<ACLOAD0>, .read = ac_load_drv_read<ACLOAD0>,
  .write = ac_load_drv_write<ACLOAD0>, .close = ac_load_drv_close<ACLOAD0>
};

struct file_ops_s acload_drv_acload1_fops {
  .flock = acload_flock<ACLOAD1>, .funlock = acload_funlock<ACLOAD1>, .open = ac_load_drv_open<ACLOAD1>, .ioctl = ac_load_drv_ioctl<ACLOAD1>, .read = ac_load_drv_read<ACLOAD1>,
  .write = ac_load_drv_write<ACLOAD1>, .close = ac_load_drv_close<ACLOAD1>
};

struct file_ops_s acload_drv_acload2_fops {
  .flock = acload_flock<ACLOAD2>, .funlock = acload_funlock<ACLOAD2>, .open = ac_load_drv_open<ACLOAD2>, .ioctl = ac_load_drv_ioctl<ACLOAD2>, .read = ac_load_drv_read<ACLOAD2>,
  .write = ac_load_drv_write<ACLOAD2>, .close = ac_load_drv_close<ACLOAD2>
};

struct file_ops_s acload_drv_acload3_fops {
  .flock = acload_flock<ACLOAD3>, .funlock = acload_funlock<ACLOAD3>, .open = ac_load_drv_open<ACLOAD3>, .ioctl = ac_load_drv_ioctl<ACLOAD3>, .read = ac_load_drv_read<ACLOAD3>,
  .write = ac_load_drv_write<ACLOAD3>, .close = ac_load_drv_close<ACLOAD3>
};

/* Driver operations */
void ac_load_drv_init(const struct drv_model_cmn_s *drv) {
  int32_t gpio_fd, rc;
  struct gpio_setup_req_s gpio_setup_req;
  const struct drv_model_cmn_s *gpio_drv;

  if (!(gpio_drv = drv->dep("gpio"))) {
    ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = open(gpio_drv, "B", 3, 3u)) < 0) {
    ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  for (uint32_t ac_load_pin_no : {ACLOAD0, ACLOAD1, ACLOAD2, ACLOAD3}) {
    gpio_setup_req = {acload_gpio_pins[ac_load_pin_no]};
    if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_PP, &gpio_setup_req, sizeof(gpio_setup_req))) < 0) {
      ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((rc = close(gpio_drv, gpio_fd)) < 0) {
    ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  acload_drv_acload0_fops.owner = drv;
  acload_drv_acload1_fops.owner = drv;
  acload_drv_acload2_fops.owner = drv;
  acload_drv_acload3_fops.owner = drv;

  acload_locks[ACLOAD0] = xSemaphoreCreateRecursiveMutex();
  acload_locks[ACLOAD1] = xSemaphoreCreateRecursiveMutex();
  acload_locks[ACLOAD2] = xSemaphoreCreateRecursiveMutex();
  acload_locks[ACLOAD3] = xSemaphoreCreateRecursiveMutex();

  drv->register_chardev("acload0", &acload_drv_acload0_fops);
  drv->register_chardev("acload1", &acload_drv_acload1_fops);
  drv->register_chardev("acload2", &acload_drv_acload2_fops);
  drv->register_chardev("acload3", &acload_drv_acload3_fops);

  drv_ptr = drv;
  return;

error:
  ac_load_drv_exit(drv);
  return;
}

void ac_load_drv_exit(const struct drv_model_cmn_s *drv) {
  int32_t gpio_fd, rc;
  struct gpio_setup_req_s gpio_setup_req;
  const struct drv_model_cmn_s *gpio_drv;

  if (!(gpio_drv = drv->dep("gpio"))) {
    ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = open(gpio_drv, "C", 3, 3u)) < 0) {
    ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  for (uint32_t ac_load_pin_no : {ACLOAD0, ACLOAD1, ACLOAD2, ACLOAD3}) {
    gpio_setup_req = {acload_gpio_pins[ac_load_pin_no]};
    if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_setup_req, sizeof(gpio_setup_req))) < 0) {
      ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((rc = close(gpio_drv, gpio_fd)) < 0) {
    ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  acload_drv_acload0_fops.owner = nullptr;
  acload_drv_acload1_fops.owner = nullptr;
  acload_drv_acload2_fops.owner = nullptr;
  acload_drv_acload2_fops.owner = nullptr;

  vSemaphoreDelete(acload_locks[ACLOAD0]);
  vSemaphoreDelete(acload_locks[ACLOAD1]);
  vSemaphoreDelete(acload_locks[ACLOAD2]);
  vSemaphoreDelete(acload_locks[ACLOAD3]);

  drv->unregister_chardev("acload0");
  drv->unregister_chardev("acload1");
  drv->unregister_chardev("acload2");
  drv->unregister_chardev("acload3");

  drv_ptr = nullptr;
  return;

error:
  return;
}

template <enum ac_loads_e acload_no> static int32_t ac_load_drv_open(int32_t oflags, mode_t mode) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

template <enum ac_loads_e acload_no> static int32_t ac_load_drv_ioctl(uint64_t req, const void *buf, size_t size) {
  const struct drv_model_cmn_s *gpio;
  int32_t rc, gpio_fd;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (req) {
  case AC_LOAD_ON: {
    const struct gpio_write_pin_req_s gpio_req { .pin = acload_gpio_pins[acload_no], .val = 1u };
    if (!(gpio = drv_ptr->dep("gpio"))) {
      ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((gpio_fd = ::open(gpio, "B", 3, 3u)) < 0) {
      ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_PIN_WRITE, &gpio_req, sizeof(gpio_req))) < 0) {
      // errno = ???
      ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    /* Close gpio/B file */
    if ((rc = ::close(gpio, gpio_fd)) < 0) {
      ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case AC_LOAD_OFF: {
    const struct gpio_write_pin_req_s gpio_req { .pin = acload_gpio_pins[acload_no], .val = 0u };
    if (!(gpio = drv_ptr->dep("gpio"))) {
      ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((gpio_fd = ::open(gpio, "B", 3, 3u)) < 0) {
      ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_PIN_WRITE, &gpio_req, sizeof(gpio_req))) < 0) {
      // errno = ???
      ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    /* Close gpio/B file */
    if ((rc = ::close(gpio, gpio_fd)) < 0) {
      ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case AC_LOAD_GET_STATE: {
    const struct ac_load_get_state_req_s *state_req = static_cast<const struct ac_load_get_state_req_s *>(buf);
    uint16_t gpio_val;
    if (!(gpio = drv_ptr->dep("gpio"))) {
      ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((gpio_fd = ::open(gpio, "B", 3, 3u)) < 0) {
      ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::read(gpio, gpio_fd, &gpio_val, sizeof(gpio_val))) < 0) {
      // errno = ???
      ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    /* Close gpio/B file */
    if ((rc = ::close(gpio, gpio_fd)) < 0) {
      ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if (state_req->state) {
      *state_req->state = (gpio_val >> acload_gpio_pins[acload_no]) & 0x01 ? ac_load_state_e::AC_LOAD_STATE_ON : ac_load_state_e::AC_LOAD_STATE_OFF;
    }
  } break;
  }

  return 0;
error:
  return -1;
}

template <enum ac_loads_e acload_no> static int32_t ac_load_drv_read(void *const buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

template <enum ac_loads_e acload_no> static int32_t ac_load_drv_write(const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

template <enum ac_loads_e acload_no> static int32_t ac_load_drv_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t ac_load_printf(const char *fmt, ...) {
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

  if (!(usart = drv_ptr->dep("usart"))) {
    ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (strlen) {
    if ((usart_fd = ::open(usart, "usart1", 3, 3u)) < 0) {
      ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, "[ac_load] : ", std::strlen("[ac_load] : "))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }
      ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, temp, std::strlen(temp))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::close(usart, usart_fd)) < 0) {
      ac_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
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
