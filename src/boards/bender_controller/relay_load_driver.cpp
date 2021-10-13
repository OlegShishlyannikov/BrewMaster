#include <cstdarg>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "arch/mcu_tags.hpp"
#include "boards/board_tags.hpp"
#include "drivers/io/gpio_ioctl.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "drivers/io/relay_load_ioctl.hpp"
#include "drivers/relay_load_driver.hpp"
#include "fio/unistd.hpp"

static const struct drv_model_cmn_s *drv_ptr;
extern xQueueHandle events_worker_queue;
static constexpr const uint32_t relayloads_num = 2u;
extern bool debug_log_enabled;

static constexpr const char *console_devstr = "usart2";

/* AC load locks */
static xSemaphoreHandle relayload_locks[relayloads_num];
static const uint8_t relayload_gpio_pins[relayloads_num]{12u, 13u};
enum relay_loads_e : uint32_t { RELAYLOAD0, RELAYLOAD1 };

extern xQueueHandle events_worker_queue;

static int32_t relay_load_printf(const char *fmt, ...);
template <enum relay_loads_e relayload_no> static int32_t relay_load_drv_open(int32_t, mode_t);
template <enum relay_loads_e relayload_no> static int32_t relay_load_drv_ioctl(uint64_t, const void *, size_t);
template <enum relay_loads_e relayload_no> static int32_t relay_load_drv_read(void *const, size_t);
template <enum relay_loads_e relayload_no> static int32_t relay_load_drv_write(const void *, size_t);
template <enum relay_loads_e relayload_no> static int32_t relay_load_drv_close();

/* AC loads helper functions */
template <enum relay_loads_e relayload_no> static int32_t relayload_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreTakeRecursive(relayload_locks[relayload_no], portMAX_DELAY)) != pdPASS) {
    // errno = ???
    relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

template <enum relay_loads_e relayload_no> static int32_t relayload_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreGiveRecursive(relayload_locks[relayload_no])) != pdPASS) {
    // errno = ???
    relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

struct drv_ops_s relay_load_drv_ops {
  .init = relay_load_drv_init, .exit = relay_load_drv_exit
};

/* Relay load driver file operations secification */
struct file_ops_s relayload_drv_relayload0_fops {
  .flock = relayload_flock<RELAYLOAD0>, .funlock = relayload_funlock<RELAYLOAD0>, .open = relay_load_drv_open<RELAYLOAD0>, .ioctl = relay_load_drv_ioctl<RELAYLOAD0>,
  .read = relay_load_drv_read<RELAYLOAD0>, .write = relay_load_drv_write<RELAYLOAD0>, .close = relay_load_drv_close<RELAYLOAD0>
};

struct file_ops_s relayload_drv_relayload1_fops {
  .flock = relayload_flock<RELAYLOAD1>, .funlock = relayload_funlock<RELAYLOAD1>, .open = relay_load_drv_open<RELAYLOAD1>, .ioctl = relay_load_drv_ioctl<RELAYLOAD1>,
  .read = relay_load_drv_read<RELAYLOAD1>, .write = relay_load_drv_write<RELAYLOAD1>, .close = relay_load_drv_close<RELAYLOAD1>
};

/* Driver operations */
void relay_load_drv_init(const struct drv_model_cmn_s *drv) {
  int32_t gpio_fd, rc;
  struct gpio_setup_req_s gpio_setup_req;
  const struct drv_model_cmn_s *gpio_drv;

  if (!(gpio_drv = drv->dep("gpio"))) {
    relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = open(gpio_drv, "C", 3, 3u)) < 0) {
    relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  for (uint32_t relay_load_pin_no : {RELAYLOAD0, RELAYLOAD1}) {
    gpio_setup_req = {.pin = relayload_gpio_pins[relay_load_pin_no]};
    if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_PP, &gpio_setup_req, sizeof(gpio_setup_req))) < 0) {
      relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((rc = close(gpio_drv, gpio_fd)) < 0) {
    relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  relayload_drv_relayload0_fops.owner = drv;
  relayload_drv_relayload1_fops.owner = drv;

  relayload_locks[RELAYLOAD0] = xSemaphoreCreateRecursiveMutex();
  relayload_locks[RELAYLOAD1] = xSemaphoreCreateRecursiveMutex();

  drv->register_chardev("relayload0", &relayload_drv_relayload0_fops);
  drv->register_chardev("relayload1", &relayload_drv_relayload1_fops);

  drv_ptr = drv;
  return;

error:
  relay_load_drv_exit(drv);
  return;
}

void relay_load_drv_exit(const struct drv_model_cmn_s *drv) {
  int32_t gpio_fd, rc;
  struct gpio_setup_req_s gpio_setup_req;
  const struct drv_model_cmn_s *gpio_drv;

  if (!(gpio_drv = drv->dep("gpio"))) {
    relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = open(gpio_drv, "C", 3, 3u)) < 0) {
    relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  for (uint32_t relay_load_pin_no : {RELAYLOAD0, RELAYLOAD1}) {
    gpio_setup_req = {relayload_gpio_pins[relay_load_pin_no]};
    if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_setup_req, sizeof(gpio_setup_req))) < 0) {
      relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((rc = close(gpio_drv, gpio_fd)) < 0) {
    relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  relayload_drv_relayload0_fops.owner = nullptr;
  relayload_drv_relayload1_fops.owner = nullptr;

  vSemaphoreDelete(relayload_locks[RELAYLOAD0]);
  vSemaphoreDelete(relayload_locks[RELAYLOAD1]);

  drv->unregister_chardev("relayload0");
  drv->unregister_chardev("relayload1");

  drv_ptr = nullptr;
  return;

error:
  return;
}

template <enum relay_loads_e relayload_no> static int32_t relay_load_drv_open(int32_t oflags, mode_t mode) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

template <enum relay_loads_e relayload_no> static int32_t relay_load_drv_ioctl(uint64_t req, const void *buf, size_t size) {
  const struct drv_model_cmn_s *gpio;
  int32_t rc, gpio_fd;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(gpio = drv_ptr->dep("gpio"))) {
    relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = ::open(gpio, "C", 3, 3u)) < 0) {
    relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (req) {
  case RELAY_LOAD_ON: {
    const struct gpio_write_pin_req_s gpio_req { .pin = relayload_gpio_pins[relayload_no], .val = 1u };
    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_PIN_WRITE, &gpio_req, sizeof(gpio_req))) < 0) {
      // errno = ???
      relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case RELAY_LOAD_OFF: {
    const struct gpio_write_pin_req_s gpio_req { .pin = relayload_gpio_pins[relayload_no], .val = 0u };
    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_PIN_WRITE, &gpio_req, sizeof(gpio_req))) < 0) {
      // errno = ???
      relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case RELAY_LOAD_GET_STATE: {
    const struct relay_load_get_state_req_s *state_req = static_cast<const struct relay_load_get_state_req_s *>(buf);
    uint16_t gpio_val;
    if ((rc = ::read(gpio, gpio_fd, &gpio_val, sizeof(gpio_val))) < 0) {
      // errno = ???
      relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if (state_req->state) {
      *state_req->state = (gpio_val >> relayload_gpio_pins[relayload_no]) & 0x01 ? relay_load_state_e::RELAY_LOAD_STATE_ON : relay_load_state_e::RELAY_LOAD_STATE_OFF;
    }
  } break;
  }

  /* Close gpio/C file */
  if ((rc = ::close(gpio, gpio_fd)) < 0) {
    relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  /* Close gpio/C file */
  if ((rc = ::close(gpio, gpio_fd)) < 0) {
    relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  }

  return -1;
}

template <enum relay_loads_e relayload_no> static int32_t relay_load_drv_read(void *const buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

template <enum relay_loads_e relayload_no> static int32_t relay_load_drv_write(const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

template <enum relay_loads_e relayload_no> static int32_t relay_load_drv_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t relay_load_printf(const char *fmt, ...) {
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
    relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (strlen) {
    if ((usart_fd = ::open(usart, console_devstr, 3, 3u)) < 0) {
      relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, "[relay_load] : ", std::strlen("[relay_load] : "))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }
      relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, temp, std::strlen(temp))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::close(usart, usart_fd)) < 0) {
      relay_load_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
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
