#include <cstdarg>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "arch/mcu_tags.hpp"
#include "boards/board_tags.hpp"
#include "drivers/buzzer_driver.hpp"
#include "drivers/io/buzzer_ioctl.hpp"
#include "drivers/io/gpio_ioctl.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "fio/unistd.hpp"

static const struct drv_model_cmn_s *drv_ptr;

/* Buzzer lock */
static xSemaphoreHandle buzzer_lock;
extern xQueueHandle events_worker_queue;
static constexpr const uint32_t buzzer_pin_no = 10u;
extern bool debug_log_enabled;

static int32_t buzzer_printf(const char *fmt, ...);

/* BSP dependent file operations functions -- forward reference */
static int32_t buzzer_drv_open(int32_t, mode_t);
static int32_t buzzer_drv_ioctl(uint64_t, const void *, size_t);
static int32_t buzzer_drv_read(void *const, size_t);
static int32_t buzzer_drv_write(const void *, size_t);
static int32_t buzzer_drv_close();

/* Helper functions */
static int32_t buzzer_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  BaseType_t rc;
  if (!(rc = xSemaphoreTakeRecursive(buzzer_lock, portIO_MAX_DELAY))) {
    // errno = ???
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t buzzer_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  BaseType_t rc;
  if (!(rc = xSemaphoreGiveRecursive(buzzer_lock))) {
    // errno = ???
    goto error;
  }

  return 0;
error:
  return -1;
}

struct drv_ops_s buzzer_drv_ops {
  .init = buzzer_drv_init, .exit = buzzer_drv_exit
};

/* Driver file operations specification */
struct file_ops_s buzzer_drv_fops {
  .flock = buzzer_flock, .funlock = buzzer_funlock, .open = buzzer_drv_open, .ioctl = buzzer_drv_ioctl, .read = buzzer_drv_read, .write = buzzer_drv_write, .close = buzzer_drv_close
};

/* Driver operations */
void buzzer_drv_init(const struct drv_model_cmn_s *drv) {
  int32_t gpio_fd, rc;
  const struct drv_model_cmn_s *gpio_drv;
  struct gpio_setup_req_s gpio_setup_req {
    .pin = buzzer_pin_no
  };

  if (!(gpio_drv = drv->dep("gpio"))) {
    buzzer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = open(gpio_drv, "C", 3, 3u)) < 0) {
    buzzer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_PP, &gpio_setup_req, sizeof(gpio_setup_req))) < 0) {
    buzzer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = close(gpio_drv, gpio_fd)) < 0) {
    buzzer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  buzzer_drv_fops.owner = drv;
  buzzer_lock = xSemaphoreCreateRecursiveMutex();
  drv->register_chardev("buzzer0", &buzzer_drv_fops);

  drv_ptr = drv;
  return;

error:
  buzzer_drv_exit(drv);
  return;
};

void buzzer_drv_exit(const struct drv_model_cmn_s *drv) {
  int32_t gpio_fd, rc;
  const struct drv_model_cmn_s *gpio_drv;
  struct gpio_setup_req_s gpio_setup_req {
    .pin = buzzer_pin_no
  };

  if (!(gpio_drv = drv->dep("gpio"))) {
    buzzer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = open(gpio_drv, "A", 3, 3u)) < 0) {
    buzzer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_setup_req, sizeof(gpio_setup_req))) < 0) {
    buzzer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = close(gpio_drv, gpio_fd)) < 0) {
    buzzer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  drv->unregister_chardev("buzzer0");
  vSemaphoreDelete(buzzer_lock);
  drv_ptr = nullptr;
  return;

error:
  drv_ptr = nullptr;
  return;
};

static int32_t buzzer_drv_open(int32_t oflags, mode_t mode) { /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t buzzer_drv_ioctl(uint64_t req, const void *buf, size_t size) {
  int32_t rc, gpio_fd;
  const struct drv_model_cmn_s *gpio;
  struct gpio_write_pin_req_s gpio_req {
    .pin = buzzer_pin_no, .val = 1u
  };

  if (!(gpio = drv_ptr->dep("gpio"))) {
    buzzer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  switch (req) {
  case BUZZER_BEEP: {
    const struct buzzer_beep_req_s *req = static_cast<const struct buzzer_beep_req_s *>(buf);
    if ((gpio_fd = ::open(gpio, "C", 3, 3u)) < 0) {
      buzzer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    for (uint32_t i = 0u; i < req->n; i++) {
      gpio_req.val = 1u;
      if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_PIN_WRITE, &gpio_req, sizeof(gpio_req))) < 0) {
        buzzer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      for (uint32_t j = 0u; j < req->up; j++) {
        req->delay_fn(1u);
      }

      gpio_req.val = 0u;
      if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_PIN_WRITE, &gpio_req, sizeof(gpio_req))) < 0) {
        buzzer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      for (uint32_t k = 0u; k < req->down; k++) {
        req->delay_fn(1u);
      }
    }

    /* Close gpio/A file */
    if ((rc = ::close(gpio, gpio_fd)) < 0) {
      buzzer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;
  default: {
    rc = -1;
  } break;
  }

  return rc;
error:
  return -1;
}

static int32_t buzzer_drv_read(void *const buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t buzzer_drv_write(const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t buzzer_drv_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t buzzer_printf(const char *fmt, ...) {
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
    buzzer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (strlen) {
    if ((usart_fd = ::open(usart, "usart1", 3, 3u)) < 0) {
      buzzer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, "[buzzer] : ", std::strlen("[buzzer] : "))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        buzzer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }
      buzzer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, temp, std::strlen(temp))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        buzzer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      buzzer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::close(usart, usart_fd)) < 0) {
      buzzer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
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
