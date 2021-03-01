#include <cstdarg>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "arch/mcu_tags.hpp"
#include "boards/board_tags.hpp"
#include "drivers/button_driver.hpp"
#include "drivers/io/button_event.hpp"
#include "drivers/io/button_ioctl.hpp"
#include "drivers/io/gpio_event.hpp"
#include "drivers/io/gpio_ioctl.hpp"
#include "fio/unistd.hpp"

static const struct drv_model_cmn_s *drv_ptr;
extern xQueueHandle events_worker_queue;

// Global flags
extern bool debug_log_enabled;

/* Button0 lock */
static xSemaphoreHandle BUTTON0_lock;

/* Button1 lock */
static xSemaphoreHandle BUTTON1_lock;

/* Button2 lock */
static xSemaphoreHandle BUTTON2_lock;

struct drv_ops_s button_drv_ops {
  .init = button_drv_init, .exit = button_drv_exit
};

static constexpr const uint32_t button0_gpio_pin_no = 0u, button1_gpio_pin_no = 1u, button2_gpio_pin_no = 2u;
static constexpr const uint32_t button0_port_no = 0u, button1_port_no = 0u, button2_port_no = 0u;

static int32_t buttons_printf(const char *fmt, ...);

template <uint32_t BtnN> static void btn_callback(const void *, size_t);
static void (*btn_callback_nested[3u])(const void *, size_t);

static int32_t button_drv_BUTTON0_open(int32_t, mode_t);
static int32_t button_drv_BUTTON0_ioctl(uint64_t, const void *, size_t);
static int32_t button_drv_BUTTON0_read(void *const, size_t);
static int32_t button_drv_BUTTON0_write(const void *, size_t);
static int32_t button_drv_BUTTON0_close();

static int32_t button_drv_BUTTON1_open(int32_t, mode_t);
static int32_t button_drv_BUTTON1_ioctl(uint64_t, const void *, size_t);
static int32_t button_drv_BUTTON1_read(void *const, size_t);
static int32_t button_drv_BUTTON1_write(const void *, size_t);
static int32_t button_drv_BUTTON1_close();

static int32_t button_drv_BUTTON2_open(int32_t, mode_t);
static int32_t button_drv_BUTTON2_ioctl(uint64_t, const void *, size_t);
static int32_t button_drv_BUTTON2_read(void *const, size_t);
static int32_t button_drv_BUTTON2_write(const void *, size_t);
static int32_t button_drv_BUTTON2_close();

/* BUTTON0 helper functions */
static int32_t BUTTON0_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreTakeRecursive(BUTTON0_lock, portIO_MAX_DELAY)) != pdPASS) {
    // errno = ???
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t BUTTON0_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreGiveRecursive(BUTTON0_lock)) != pdPASS) {
    // errno = ???
    goto error;
  }

  return 0;
error:
  return -1;
}

/* BUTTON0 helper functions */
static int32_t BUTTON1_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreTakeRecursive(BUTTON0_lock, portIO_MAX_DELAY)) != pdPASS) {
    // errno = ???
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t BUTTON1_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreGiveRecursive(BUTTON0_lock)) != pdPASS) {
    // errno = ???
    goto error;
  }

  return 0;
error:
  return -1;
}

/* BUTTON0 helper functions */
static int32_t BUTTON2_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreTakeRecursive(BUTTON0_lock, portIO_MAX_DELAY)) != pdPASS) {
    // errno = ???
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t BUTTON2_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreGiveRecursive(BUTTON0_lock)) != pdPASS) {
    // errno = ???
    goto error;
  }

  return 0;
error:
  return -1;
}

/* BUTTONSS driver file operations secification */
struct file_ops_s button_drv_BUTTON0_fops {
  .flock = BUTTON0_flock, .funlock = BUTTON0_funlock, .open = button_drv_BUTTON0_open, .ioctl = button_drv_BUTTON0_ioctl, .read = button_drv_BUTTON0_read, .write = button_drv_BUTTON0_write,
  .close = button_drv_BUTTON0_close
};

/* BUTTONS driver file operations secification */
struct file_ops_s button_drv_BUTTON1_fops {
  .flock = BUTTON1_flock, .funlock = BUTTON1_funlock, .open = button_drv_BUTTON1_open, .ioctl = button_drv_BUTTON1_ioctl, .read = button_drv_BUTTON1_read, .write = button_drv_BUTTON1_write,
  .close = button_drv_BUTTON1_close
};

/* BUTTONS driver file operations secification */
struct file_ops_s button_drv_BUTTON2_fops {
  .flock = BUTTON2_flock, .funlock = BUTTON2_funlock, .open = button_drv_BUTTON2_open, .ioctl = button_drv_BUTTON2_ioctl, .read = button_drv_BUTTON2_read, .write = button_drv_BUTTON2_write,
  .close = button_drv_BUTTON2_close
};

void button_drv_init(const struct drv_model_cmn_s *drv) {
  int32_t gpio_fd, rc;
  struct gpio_setup_req_s gpio_setup_req;
  const struct drv_model_cmn_s *gpio_drv;

  if (!(gpio_drv = drv->dep("gpio"))) {
    goto error;
  }

  if ((gpio_fd = open(gpio_drv, "A", 3, 3u)) < 0) {
    goto error;
  }

  gpio_setup_req = {button0_gpio_pin_no};
  if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_setup_req, sizeof(gpio_setup_req))) < 0) {
    if ((rc = close(gpio_drv, gpio_fd)) < 0) {
      goto error;
    }

    goto error;
  }

  gpio_setup_req = {button1_gpio_pin_no};
  if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_setup_req, sizeof(gpio_setup_req))) < 0) {
    if ((rc = close(gpio_drv, gpio_fd)) < 0) {
      goto error;
    }

    goto error;
  }

  gpio_setup_req = {button2_gpio_pin_no};
  if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_setup_req, sizeof(gpio_setup_req))) < 0) {
    if ((rc = close(gpio_drv, gpio_fd)) < 0) {
      goto error;
    }

    goto error;
  }

  if ((rc = close(gpio_drv, gpio_fd)) < 0) {
    goto error;
  }

  button_drv_BUTTON0_fops.owner = drv;
  button_drv_BUTTON1_fops.owner = drv;
  button_drv_BUTTON2_fops.owner = drv;

  BUTTON0_lock = xSemaphoreCreateRecursiveMutex();
  BUTTON1_lock = xSemaphoreCreateRecursiveMutex();
  BUTTON2_lock = xSemaphoreCreateRecursiveMutex();

  drv->register_chardev("button0", &button_drv_BUTTON0_fops);
  drv->register_chardev("button1", &button_drv_BUTTON1_fops);
  drv->register_chardev("button2", &button_drv_BUTTON2_fops);

  drv_ptr = drv;
  return;

error:
  button_drv_exit(drv);
  return;
}

void button_drv_exit(const struct drv_model_cmn_s *drv) {
  int32_t gpio_fd, rc;
  struct gpio_setup_req_s gpio_setup_req;
  const struct drv_model_cmn_s *gpio_drv;

  if (!(gpio_drv = drv->dep("gpio"))) {
    goto error;
  }

  if ((gpio_fd = open(gpio_drv, "A", 3, 3u)) < 0) {
    goto error;
  }

  gpio_setup_req = {button0_gpio_pin_no};
  if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_setup_req, sizeof(gpio_setup_req))) < 0) {
    if ((rc = close(gpio_drv, gpio_fd)) < 0) {
      goto error;
    }

    goto error;
  }

  gpio_setup_req = {button1_gpio_pin_no};
  if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_setup_req, sizeof(gpio_setup_req))) < 0) {
    if ((rc = close(gpio_drv, gpio_fd)) < 0) {
      goto error;
    }

    goto error;
  }

  gpio_setup_req = {button2_gpio_pin_no};
  if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_setup_req, sizeof(gpio_setup_req))) < 0) {
    if ((rc = close(gpio_drv, gpio_fd)) < 0) {
      goto error;
    }

    goto error;
  }

  if ((rc = close(gpio_drv, gpio_fd)) < 0) {
    goto error;
  }

  button_drv_BUTTON0_fops.owner = nullptr;
  button_drv_BUTTON1_fops.owner = nullptr;
  button_drv_BUTTON2_fops.owner = nullptr;

  vSemaphoreDelete(BUTTON0_lock);
  vSemaphoreDelete(BUTTON1_lock);
  vSemaphoreDelete(BUTTON2_lock);

  drv->unregister_chardev("button0");
  drv->unregister_chardev("button1");
  drv->unregister_chardev("button2");

  drv_ptr = nullptr;
  return;
error:
  return;
}

static int32_t button_drv_BUTTON0_open(int32_t oflags, mode_t mode) {
  int32_t rc;
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t button_drv_BUTTON0_ioctl(uint64_t req, const void *buf, size_t size) {
  int32_t gpio_fd, rc;
  const struct drv_model_cmn_s *gpio_drv;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  switch (req) {
  case BUTTON_IRQ_ENABLE: {
    const struct button_irq_mgm_req_s *button_req = reinterpret_cast<const struct button_irq_mgm_req_s *>(buf);
    struct gpio_irq_mgm_req_s gpio_req {
      .exti_line = button0_gpio_pin_no, .priority = 5u, .trigger = static_cast<enum exti_triggers_e>(button_req->trigger), .callback = btn_callback<0u>
    };

    btn_callback_nested[0u] = button_req->callback;

    if (!(gpio_drv = drv_ptr->dep("gpio"))) {
      goto error;
    }

    if ((gpio_fd = open(gpio_drv, "A", 3, 3u)) < 0) {
      goto error;
    }

    if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_IRQ_ENABLE, &gpio_req, sizeof(gpio_req))) < 0) {
      if ((rc = close(gpio_drv, gpio_fd)) < 0) {
        goto error;
      }

      goto error;
    }

    if ((rc = close(gpio_drv, gpio_fd)) < 0) {
      goto error;
    }
  } break;

  case BUTTON_IRQ_DISABLE: {
    const struct button_irq_mgm_req_s *button_req = reinterpret_cast<const struct button_irq_mgm_req_s *>(buf);
    struct gpio_irq_mgm_req_s gpio_req {
      .exti_line = button0_gpio_pin_no, .priority = 5u, .trigger = static_cast<enum exti_triggers_e>(button_req->trigger), .callback = nullptr
    };

    btn_callback_nested[0u] = nullptr;

    if (!(gpio_drv = drv_ptr->dep("gpio"))) {
      goto error;
    }

    if ((gpio_fd = open(gpio_drv, "A", 3, 3u)) < 0) {
      goto error;
    }

    if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_IRQ_DISABLE, &gpio_req, sizeof(gpio_req))) < 0) {
      if ((rc = close(gpio_drv, gpio_fd)) < 0) {
        goto error;
      }

      goto error;
    }

    if ((rc = close(gpio_drv, gpio_fd)) < 0) {
      goto error;
    }
  } break;

  case BUTTON_GET_STATE: {
    const struct button_get_state_req_s *button_req = reinterpret_cast<const struct button_get_state_req_s *>(buf);
    uint16_t gpio_val;

    if (!(gpio_drv = drv_ptr->dep("gpio"))) {
      goto error;
    }

    if ((gpio_fd = open(gpio_drv, "A", 3, 3u)) < 0) {
      goto error;
    }

    if ((rc = ::read(gpio_drv, gpio_fd, &gpio_val, sizeof(gpio_val))) < 0) {
      if ((rc = close(gpio_drv, gpio_fd)) < 0) {
        goto error;
      }

      goto error;
    }

    if ((rc = close(gpio_drv, gpio_fd)) < 0) {
      goto error;
    }

    if ((gpio_val >> button0_gpio_pin_no) & 0x01u) {
      *button_req->state = button_state_e::BUTTON_RELEASED;
    } else {
      *button_req->state = button_state_e::BUTTON_PRESSED;
    }
  } break;
  default:
    break;
  }

  return rc;
error:
  return -1;
}

static int32_t button_drv_BUTTON0_read(void *const buf, size_t size) {
  int32_t rc, gpio_fd;
  const struct drv_model_cmn_s *gpio_drv;
  uint16_t gpio_val;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  if (!buf) {
    goto error;
  }

  if (!(gpio_drv = drv_ptr->dep("gpio"))) {
    goto error;
  }

  if ((gpio_fd = open(gpio_drv, "A", 3, 3u)) < 0) {
    goto error;
  }

  if ((rc = ::read(gpio_drv, gpio_fd, &gpio_val, sizeof(gpio_val))) < 0) {
    if ((rc = close(gpio_drv, gpio_fd)) < 0) {
      goto error;
    }

    goto error;
  }

  if ((rc = close(gpio_drv, gpio_fd)) < 0) {
    goto error;
  }

  *reinterpret_cast<uint16_t *const>(buf) = (gpio_val >> button0_gpio_pin_no) & 0x01u;
  return rc;
error:
  return -1;
}

static int32_t button_drv_BUTTON0_write(const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t button_drv_BUTTON0_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t button_drv_BUTTON1_open(int32_t oflags, mode_t mode) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t button_drv_BUTTON1_ioctl(uint64_t req, const void *buf, size_t size) {
  int32_t gpio_fd, rc;
  const struct drv_model_cmn_s *gpio_drv;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  switch (req) {
  case BUTTON_IRQ_ENABLE: {
    const struct button_irq_mgm_req_s *button_req = reinterpret_cast<const struct button_irq_mgm_req_s *>(buf);
    struct gpio_irq_mgm_req_s gpio_req {
      .exti_line = button1_gpio_pin_no, .priority = 5u, .trigger = static_cast<enum exti_triggers_e>(button_req->trigger), .callback = btn_callback<1u>
    };

    btn_callback_nested[1u] = button_req->callback;

    if (!(gpio_drv = drv_ptr->dep("gpio"))) {
      goto error;
    }

    if ((gpio_fd = open(gpio_drv, "A", 3, 3u)) < 0) {
      goto error;
    }

    if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_IRQ_ENABLE, &gpio_req, sizeof(gpio_req))) < 0) {
      if ((rc = close(gpio_drv, gpio_fd)) < 0) {
        goto error;
      }

      goto error;
    }

    if ((rc = close(gpio_drv, gpio_fd)) < 0) {
      goto error;
    }
  } break;

  case BUTTON_IRQ_DISABLE: {
    const struct button_irq_mgm_req_s *button_req = reinterpret_cast<const struct button_irq_mgm_req_s *>(buf);
    struct gpio_irq_mgm_req_s gpio_req {
      .exti_line = button1_gpio_pin_no, .priority = 5u, .trigger = static_cast<enum exti_triggers_e>(button_req->trigger), .callback = nullptr
    };

    btn_callback_nested[1u] = nullptr;

    if (!(gpio_drv = drv_ptr->dep("gpio"))) {
      goto error;
    }

    if ((gpio_fd = open(gpio_drv, "A", 3, 3u)) < 0) {
      goto error;
    }

    if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_IRQ_DISABLE, &gpio_req, sizeof(gpio_req))) < 0) {
      if ((rc = close(gpio_drv, gpio_fd)) < 0) {
        goto error;
      }

      goto error;
    }

    if ((rc = close(gpio_drv, gpio_fd)) < 0) {
      goto error;
    }
  } break;

  case BUTTON_GET_STATE: {
    const struct button_get_state_req_s *button_req = reinterpret_cast<const struct button_get_state_req_s *>(buf);
    uint16_t gpio_val;

    if (!(gpio_drv = drv_ptr->dep("gpio"))) {
      goto error;
    }

    if ((gpio_fd = open(gpio_drv, "A", 3, 3u)) < 0) {
      goto error;
    }

    if ((rc = ::read(gpio_drv, gpio_fd, &gpio_val, sizeof(gpio_val))) < 0) {
      if ((rc = close(gpio_drv, gpio_fd)) < 0) {
        goto error;
      }

      goto error;
    }

    if ((rc = close(gpio_drv, gpio_fd)) < 0) {
      goto error;
    }

    if ((gpio_val >> button1_gpio_pin_no) & 0x01u) {
      *button_req->state = button_state_e::BUTTON_RELEASED;
    } else {
      *button_req->state = button_state_e::BUTTON_PRESSED;
    }
  } break;
  default:
    break;
  }

  return rc;
error:
  return -1;
}

static int32_t button_drv_BUTTON1_read(void *const buf, size_t size) {
  int32_t rc, gpio_fd;
  const struct drv_model_cmn_s *gpio_drv;
  uint16_t gpio_val;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  if (!buf) {
    goto error;
  }

  if (!(gpio_drv = drv_ptr->dep("gpio"))) {
    goto error;
  }

  if ((gpio_fd = open(gpio_drv, "A", 3, 3u)) < 0) {
    goto error;
  }

  if ((rc = ::read(gpio_drv, gpio_fd, &gpio_val, sizeof(gpio_val))) < 0) {
    if ((rc = close(gpio_drv, gpio_fd)) < 0) {
      goto error;
    }

    goto error;
  }

  if ((rc = close(gpio_drv, gpio_fd)) < 0) {
    goto error;
  }

  *reinterpret_cast<uint16_t *const>(buf) = (gpio_val >> button1_gpio_pin_no) & 0x01u;
  return rc;
error:
  return -1;
}

static int32_t button_drv_BUTTON1_write(const void *oflags, size_t mode) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t button_drv_BUTTON1_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t button_drv_BUTTON2_open(int32_t oflags, mode_t mode) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t button_drv_BUTTON2_ioctl(uint64_t req, const void *buf, size_t size) {
  int32_t gpio_fd, rc;
  const struct drv_model_cmn_s *gpio_drv;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  switch (req) {
  case BUTTON_IRQ_ENABLE: {
    const struct button_irq_mgm_req_s *button_req = reinterpret_cast<const struct button_irq_mgm_req_s *>(buf);
    struct gpio_irq_mgm_req_s gpio_req {
      .exti_line = button2_gpio_pin_no, .priority = 5u, .trigger = static_cast<enum exti_triggers_e>(button_req->trigger), .callback = btn_callback<2u>
    };

    btn_callback_nested[2u] = button_req->callback;

    if (!(gpio_drv = drv_ptr->dep("gpio"))) {
      goto error;
    }

    if ((gpio_fd = open(gpio_drv, "A", 3, 3u)) < 0) {
      goto error;
    }

    if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_IRQ_ENABLE, &gpio_req, sizeof(gpio_req))) < 0) {
      if ((rc = close(gpio_drv, gpio_fd)) < 0) {
        goto error;
      }

      goto error;
    }

    if ((rc = close(gpio_drv, gpio_fd)) < 0) {
      goto error;
    }
  } break;

  case BUTTON_IRQ_DISABLE: {
    const struct button_irq_mgm_req_s *button_req = reinterpret_cast<const struct button_irq_mgm_req_s *>(buf);
    struct gpio_irq_mgm_req_s gpio_req {
      .exti_line = button2_gpio_pin_no, .priority = 5u, .trigger = static_cast<enum exti_triggers_e>(button_req->trigger), .callback = nullptr
    };

    btn_callback_nested[2u] = nullptr;
    if (!(gpio_drv = drv_ptr->dep("gpio"))) {
      goto error;
    }

    if ((gpio_fd = open(gpio_drv, "A", 3, 3u)) < 0) {
      goto error;
    }

    if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_IRQ_DISABLE, &gpio_req, sizeof(gpio_req))) < 0) {
      if ((rc = close(gpio_drv, gpio_fd)) < 0) {
        goto error;
      }

      goto error;
    }

    if ((rc = close(gpio_drv, gpio_fd)) < 0) {
      goto error;
    }
  } break;
  case BUTTON_GET_STATE: {
    const struct button_get_state_req_s *button_req = reinterpret_cast<const struct button_get_state_req_s *>(buf);
    uint16_t gpio_val;

    if (!(gpio_drv = drv_ptr->dep("gpio"))) {
      goto error;
    }

    if ((gpio_fd = open(gpio_drv, "A", 3, 3u)) < 0) {
      goto error;
    }

    if ((rc = ::read(gpio_drv, gpio_fd, &gpio_val, sizeof(gpio_val))) < 0) {
      if ((rc = close(gpio_drv, gpio_fd)) < 0) {
        goto error;
      }

      goto error;
    }

    if ((rc = close(gpio_drv, gpio_fd)) < 0) {
      goto error;
    }

    if ((gpio_val >> button2_gpio_pin_no) & 0x01u) {
      *button_req->state = button_state_e::BUTTON_RELEASED;
    } else {
      *button_req->state = button_state_e::BUTTON_PRESSED;
    }
  } break;
  default:
    break;
  }

  return rc;
error:
  return -1;
}

static int32_t button_drv_BUTTON2_read(void *const buf, size_t size) {
  int32_t rc, gpio_fd;
  const struct drv_model_cmn_s *gpio_drv;
  uint16_t gpio_val;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  if (!buf) {
    goto error;
  }

  if (!(gpio_drv = drv_ptr->dep("gpio"))) {
    goto error;
  }

  if ((gpio_fd = open(gpio_drv, "A", 3, 3u)) < 0) {
    goto error;
  }

  if ((rc = ::read(gpio_drv, gpio_fd, &gpio_val, sizeof(gpio_val))) < 0) {
    if ((rc = close(gpio_drv, gpio_fd)) < 0) {
      goto error;
    }

    goto error;
  }

  if ((rc = close(gpio_drv, gpio_fd)) < 0) {
    goto error;
  }

  *reinterpret_cast<uint16_t *const>(buf) = (gpio_val >> button2_gpio_pin_no) & 0x01u;
  return rc;
error:
  return -1;
}

static int32_t button_drv_BUTTON2_write(const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t button_drv_BUTTON2_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

template <uint32_t BtnN> static void btn_callback(const void *data, size_t size) {
  // Get GPIO event
  const struct gpio_event_s *gpio_event = reinterpret_cast<const struct gpio_event_s *>(data);

  struct button_event_s event {
    .pin_val = gpio_event->pin_val[BtnN == 0u ? button0_port_no : BtnN == 1u ? button1_port_no : BtnN == 2u ? button2_port_no : 0u], .btn_num = BtnN
  };

  // Free pin values vector
  free(gpio_event->pin_val);

  // Call callback function
  if (btn_callback_nested[BtnN]) {
    btn_callback_nested[BtnN](&event, sizeof(struct button_event_s));
  }
}

static int32_t buttons_printf(const char *fmt, ...) {
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
    goto error;
  }

  if (strlen) {
    if ((usart_fd = ::open(usart, "usart1", 3, 3u)) < 0) {
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, "[buttons] : ", std::strlen("[buttons] : "))) < 0) {
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
