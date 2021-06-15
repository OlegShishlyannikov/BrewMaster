#include <cstdarg>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "arch/mcu_tags.hpp"
#include "boards/board_tags.hpp"
#include "drivers/io/gpio_ioctl.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "drivers/io/spi_ioctl.hpp"
#include "drivers/io/uln2003_ioctl.hpp"
#include "drivers/io/usart_ioctl.hpp"
#include "drivers/uln2003_driver.hpp"
#include "fio/unistd.hpp"
#include "sys/events_worker.hpp"

/* This pointer will be used in interrupt handlers and will be initialized in driver init function */
static const struct drv_model_cmn_s *drv_ptr;
extern bool debug_log_enabled;

/* Latch, clock and data pins used */
static constexpr uint8_t uln2003_channel_nums[] = {8u, 11u, 12u, 9u, 10u, 15u};
static constexpr const char *uln2003_gpio_letter = "A";

/* 74HC595 lock & fifos */
static xSemaphoreHandle uln2003_lock;
extern xQueueHandle events_worker_queue;

// Printf to console
static int32_t uln2003_printf(const char *fmt, ...);

/* 74HC595 file IO functions forward reference */
static int32_t uln2003_drv_open(int32_t, mode_t);
static int32_t uln2003_drv_ioctl(uint64_t, const void *, size_t);
static int32_t uln2003_drv_read(void *const, size_t);
static int32_t uln2003_drv_write(const void *, size_t);
static int32_t uln2003_drv_close();

/* 74HC595 helper functions */
static int32_t uln2003_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreTakeRecursive(uln2003_lock, portIO_MAX_DELAY)) != pdPASS) {
    // errno = ???
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t uln2003_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreGiveRecursive(uln2003_lock)) != pdPASS) {
    // errno = ???
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

struct drv_ops_s uln2003_drv_ops {
  .init = uln2003_drv_init, .exit = uln2003_drv_exit
};

/* 74HC595 driver file operations secification */
struct file_ops_s uln2003_drv_fops {
  .flock = uln2003_flock, .funlock = uln2003_funlock, .open = uln2003_drv_open, .ioctl = uln2003_drv_ioctl, .read = uln2003_drv_read, .write = uln2003_drv_write, .close = uln2003_drv_close
};

void uln2003_drv_init(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *gpio, *tim;
  int32_t gpio_fd, tim_fd, rc;
  struct gpio_setup_req_s gpio_req;

  if (!(gpio = drv->dep("gpio"))) {
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = ::open(gpio, uln2003_gpio_letter, 3, 3u)) < 0) {
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  for (uint32_t i = 0u; i < sizeof(uln2003_channel_nums) / sizeof(uint8_t); i++) {
    gpio_req.pin = uln2003_channel_nums[i];

    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_PP, &gpio_req, sizeof(gpio_req))) < 0) {
      if ((rc = ::close(gpio, gpio_fd)) < 0) {
        uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((rc = ::close(gpio, gpio_fd)) < 0) {
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  uln2003_drv_fops.owner = drv;

  /* Init locks */
  uln2003_lock = xSemaphoreCreateRecursiveMutex();
  xSemaphoreGive(uln2003_lock);

  /* Register char device for each GPIO port */
  drv->register_chardev("uln2003", &uln2003_drv_fops);

  /* Initialize GPIO driver pointer */
  drv_ptr = drv;
  return;

  /* Something went wrong -- reset drv_ptr and exit */
error:
  uln2003_drv_exit(drv);
  return;
}

void uln2003_drv_exit(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *gpio, *tim;
  int32_t gpio_fd, tim_fd, rc;
  struct gpio_setup_req_s gpio_req;

  if (!(gpio = drv->dep("gpio"))) {
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = ::open(gpio, uln2003_gpio_letter, 3, 3u)) < 0) {
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  for (uint32_t i = 0u; i < sizeof(uln2003_channel_nums) / sizeof(uint8_t); i++) {
    gpio_req.pin = uln2003_channel_nums[i];

    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IPL, &gpio_req, sizeof(gpio_req))) < 0) {
      if ((rc = ::close(gpio, gpio_fd)) < 0) {
        uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((rc = ::close(gpio, gpio_fd)) < 0) {
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  uln2003_drv_fops.owner = nullptr;

  /* Remove locks */
  vSemaphoreDelete(uln2003_lock);

  /* Register char device for each GPIO port */
  drv->unregister_chardev("uln2003");

  /* Initialize GPIO driver pointer */
  drv_ptr = nullptr;
  return;
error:
  return;
}

/* 74HC595 file IO functions */
static int32_t uln2003_drv_open(int32_t, mode_t) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t uln2003_drv_ioctl(uint64_t req, const void *buf, size_t size) {
  const struct drv_model_cmn_s *gpio;
  int32_t gpio_fd, rc;
  struct gpio_write_pin_req_s gpio_req;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(gpio = drv_ptr->dep("gpio"))) {
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (req) {
  case ULN2003_CHANNEL_ON: {
    const struct uln2003_ch_on_off_req_s *req = reinterpret_cast<const struct uln2003_ch_on_off_req_s *>(buf);
    if ((gpio_fd = ::open(gpio, uln2003_gpio_letter, 3, 3u)) < 0) {
      uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
    gpio_req.pin = uln2003_channel_nums[req->ch_num - 1u];

    // Shifting by one more because the first channel of uln2003 isn't used on this board
    gpio_req.val = 0x01;
    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_PIN_WRITE, &gpio_req, sizeof(gpio_req))) < 0) {
      if ((gpio_fd = ::close(gpio, gpio_fd)) < 0) {
        uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((gpio_fd = ::close(gpio, gpio_fd)) < 0) {
      uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;
  case ULN2003_CHANNEL_OFF: {
    const struct uln2003_ch_on_off_req_s *req = reinterpret_cast<const struct uln2003_ch_on_off_req_s *>(buf);
    if ((gpio_fd = ::open(gpio, uln2003_gpio_letter, 3, 3u)) < 0) {
      uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
    gpio_req.pin = uln2003_channel_nums[req->ch_num - 1u];

    // Shifting by one more because the first channel of uln2003 isn't used on this board
    gpio_req.val = 0x00;
    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_PIN_WRITE, &gpio_req, sizeof(gpio_req))) < 0) {
      if ((gpio_fd = ::close(gpio, gpio_fd)) < 0) {
        uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((gpio_fd = ::close(gpio, gpio_fd)) < 0) {
      uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

  } break;

  case ULN2003_GET_STATE: {
    const struct uln2003_get_state_req_s *req = reinterpret_cast<const struct uln2003_get_state_req_s *>(buf);
    uint16_t gpio_val;

    if ((gpio_fd = ::open(gpio, uln2003_gpio_letter, 3, 3u)) < 0) {
      uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::read(gpio, gpio_fd, &gpio_val, sizeof(gpio_val))) < 0) {
      if ((gpio_fd = ::close(gpio, gpio_fd)) < 0) {
        uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    *req->state_reg = (gpio_val >> uln2003_channel_nums[req->ch_num]) & 0x01;

    if ((gpio_fd = ::close(gpio, gpio_fd)) < 0) {
      uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  default:
    break;
  }

  return 0;
error:
  return -1;
}

static int32_t uln2003_drv_read(void *const buf, size_t size) {
  const struct drv_model_cmn_s *gpio;
  int32_t rc, gpio_fd;
  uint16_t gpio_val;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(gpio = drv_ptr->dep("gpio"))) {
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = ::open(gpio, uln2003_gpio_letter, 3, 3u)) < 0) {
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::read(gpio, gpio_fd, &gpio_val, sizeof(gpio_val))) < 0) {
    if ((gpio_fd = ::close(gpio, gpio_fd)) < 0) {
      uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  for (uint32_t i = 0u; i < sizeof(uln2003_channel_nums) / sizeof(uint8_t); i++) {
    *reinterpret_cast<uint8_t *const>(buf) |= ((gpio_val >> uln2003_channel_nums[i]) & 0x01) << (sizeof(uln2003_channel_nums) / sizeof(uint8_t) - i);
  }

  if ((gpio_fd = ::close(gpio, gpio_fd)) < 0) {
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t uln2003_drv_write(const void *buf, size_t size) {
  const struct drv_model_cmn_s *gpio;
  int32_t gpio_fd, rc;
  struct gpio_write_pin_req_s gpio_req;

  /* Assuming that buffer has data type and size of uint8 */
  const uint8_t data = *reinterpret_cast<const uint8_t *>(buf) & 0b00111111;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(gpio = drv_ptr->dep("gpio"))) {
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = ::open(gpio, uln2003_gpio_letter, 3, 3u)) < 0) {
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  for (uint32_t i = 0u; i < sizeof(uln2003_channel_nums) / sizeof(uint8_t); i++) {
    gpio_req.pin = uln2003_channel_nums[i];

    // Shifting by one more because the first channel of uln2003 isn't used on this board
    gpio_req.val = (data >> ((sizeof(uln2003_channel_nums) / sizeof(uint8_t) - 1u) - i)) & 0x01;
    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_PIN_WRITE, &gpio_req, sizeof(gpio_req))) < 0) {
      if ((gpio_fd = ::close(gpio, gpio_fd)) < 0) {
        uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((gpio_fd = ::close(gpio, gpio_fd)) < 0) {
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t uln2003_drv_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    uln2003_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t uln2003_printf(const char *fmt, ...) {
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
    if ((usart_fd = ::open(usart, "usart1", 3, 3u)) < 0) {
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, "[uln2003] : ", std::strlen("[uln2003] : "))) < 0) {
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
