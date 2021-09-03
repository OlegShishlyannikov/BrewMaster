#include <cstdarg>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "arch/mcu_tags.hpp"
#include "boards/board_tags.hpp"
#include "drivers/io/gpio_ioctl.hpp"
#include "drivers/io/leds_ioctl.hpp"
#include "drivers/leds_driver.hpp"
#include "fio/unistd.hpp"

/* Leds lock */
static xSemaphoreHandle LED0_lock;
static xSemaphoreHandle LED1_lock;
static xSemaphoreHandle LED2_lock;
extern xQueueHandle events_worker_queue;

static constexpr const char *console_devstr = "usart1";

static const struct drv_model_cmn_s *drv_ptr;
static const uint16_t led0_gpio_pin_no = 12u;
static const uint16_t led1_gpio_pin_no = 13u;
static const uint16_t led2_gpio_pin_no = 14u;
extern bool debug_log_enabled;

static int32_t leds_printf(const char *fmt, ...);

/* LED driver file operations forward declaration */
static int32_t leds_drv_LED0_open(int32_t, mode_t);
static int32_t leds_drv_LED0_ioctl(uint64_t, const void *, size_t);
static int32_t leds_drv_LED0_read(void *const, size_t);
static int32_t leds_drv_LED0_write(const void *, size_t);
static int32_t leds_drv_LED0_close();

static int32_t leds_drv_LED1_open(int32_t, mode_t);
static int32_t leds_drv_LED1_ioctl(uint64_t, const void *, size_t);
static int32_t leds_drv_LED1_read(void *const, size_t);
static int32_t leds_drv_LED1_write(const void *, size_t);
static int32_t leds_drv_LED1_close();

static int32_t leds_drv_LED2_open(int32_t, mode_t);
static int32_t leds_drv_LED2_ioctl(uint64_t, const void *, size_t);
static int32_t leds_drv_LED2_read(void *const, size_t);
static int32_t leds_drv_LED2_write(const void *, size_t);
static int32_t leds_drv_LED2_close();

/* LED0 helper functions */
static int32_t LED0_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreTakeRecursive(LED0_lock, portIO_MAX_DELAY)) != pdPASS) {
    // errno = ???
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t LED0_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreGiveRecursive(LED0_lock)) != pdPASS) {
    // errno = ???
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

/* LED1 helper functions */
static int32_t LED1_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreTakeRecursive(LED1_lock, portIO_MAX_DELAY)) != pdPASS) {
    // errno = ???
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t LED1_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreGiveRecursive(LED1_lock)) != pdPASS) {
    // errno = ???
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

/* LED2 helper functions */
static int32_t LED2_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreTakeRecursive(LED2_lock, portIO_MAX_DELAY)) != pdPASS) {
    // errno = ???
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t LED2_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreGiveRecursive(LED2_lock)) != pdPASS) {
    // errno = ???
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

struct drv_ops_s leds_drv_ops {
  .init = leds_drv_init, .exit = leds_drv_exit
};

/* LEDS driver file operations secification */
struct file_ops_s leds_drv_LED0_fops {
  .flock = LED0_flock, .funlock = LED0_funlock, .open = leds_drv_LED0_open, .ioctl = leds_drv_LED0_ioctl, .read = leds_drv_LED0_read, .write = leds_drv_LED0_write, .close = leds_drv_LED0_close
};

/* LEDS driver file operations secification */
struct file_ops_s leds_drv_LED1_fops {
  .flock = LED1_flock, .funlock = LED1_funlock, .open = leds_drv_LED1_open, .ioctl = leds_drv_LED1_ioctl, .read = leds_drv_LED1_read, .write = leds_drv_LED1_write, .close = leds_drv_LED1_close
};

/* LEDS driver file operations secification */
struct file_ops_s leds_drv_LED2_fops {
  .flock = LED2_flock, .funlock = LED2_funlock, .open = leds_drv_LED2_open, .ioctl = leds_drv_LED2_ioctl, .read = leds_drv_LED2_read, .write = leds_drv_LED2_write, .close = leds_drv_LED2_close
};

void leds_drv_init(const struct drv_model_cmn_s *drv) {
  int32_t gpio_fd, rc;
  struct gpio_setup_req_s gpio_setup_req;
  const struct drv_model_cmn_s *gpio_drv;

  if (!(gpio_drv = drv->dep("gpio"))) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = open(gpio_drv, "C", 3, 3u)) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  gpio_setup_req = {led0_gpio_pin_no};
  if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_PP, &gpio_setup_req, sizeof(gpio_setup_req))) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  gpio_setup_req = {led1_gpio_pin_no};
  if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_PP, &gpio_setup_req, sizeof(gpio_setup_req))) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  gpio_setup_req = {led2_gpio_pin_no};
  if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_PP, &gpio_setup_req, sizeof(gpio_setup_req))) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = close(gpio_drv, gpio_fd)) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  leds_drv_LED0_fops.owner = drv;
  leds_drv_LED1_fops.owner = drv;
  leds_drv_LED2_fops.owner = drv;

  LED0_lock = xSemaphoreCreateRecursiveMutex();
  LED1_lock = xSemaphoreCreateRecursiveMutex();
  LED2_lock = xSemaphoreCreateRecursiveMutex();

  drv->register_chardev("led0", &leds_drv_LED0_fops);
  drv->register_chardev("led1", &leds_drv_LED1_fops);
  drv->register_chardev("led2", &leds_drv_LED2_fops);

  drv_ptr = drv;
  return;

error:
  leds_drv_exit(drv);
  return;
}

void leds_drv_exit(const struct drv_model_cmn_s *drv) {
  int32_t gpio_fd, rc;
  struct gpio_setup_req_s gpio_setup_req;
  const struct drv_model_cmn_s *gpio_drv;

  if (!(gpio_drv = drv->dep("gpio"))) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = open(gpio_drv, "C", 3, 3u)) > 0) {
    gpio_setup_req = {led0_gpio_pin_no};
    if ((rc = ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_setup_req, sizeof(gpio_setup_req))) < 0) {
      leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    gpio_setup_req = {led1_gpio_pin_no};
    if ((rc = ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_setup_req, sizeof(gpio_setup_req))) < 0) {
      leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    gpio_setup_req = {led2_gpio_pin_no};
    if ((rc = ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_setup_req, sizeof(gpio_setup_req))) < 0) {
      leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((rc = close(gpio_drv, gpio_fd)) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Unregister char devices */
  drv->unregister_chardev("led0");
  drv->unregister_chardev("led1");
  drv->unregister_chardev("led2");

  /* Remove locks */
  vSemaphoreDelete(LED0_lock);
  vSemaphoreDelete(LED1_lock);
  vSemaphoreDelete(LED2_lock);

  /* Reset driver ptr */
  drv_ptr = nullptr;

error:
  return;
}

static int32_t leds_drv_LED0_open(int32_t flags, mode_t mode) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t leds_drv_LED0_ioctl(uint64_t req, const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (req) {
  case LEDS_BLINK: {
    int32_t rc, gpio_fd;
    const struct drv_model_cmn_s *gpio_drv;
    const struct led_blink_req_s *strobe_req = reinterpret_cast<const struct led_blink_req_s *>(buf);
    const struct gpio_strobe_req_s gpio_req {
      .pin = led0_gpio_pin_no, .up_ticks = strobe_req->up_ticks, .down_ticks = strobe_req->down_ticks, .n_times = strobe_req->n_times, .delay_fn = strobe_req->delay_fn
    };

    if (!(gpio_drv = drv_ptr->dep("gpio"))) {
      leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((gpio_fd = ::open(gpio_drv, "C", 3, 3u)) < 0) {
      leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_STROBE, &gpio_req, sizeof(gpio_req))) < 0) {
      // errno = ???
      leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    /* Close gpio/C file */
    if ((rc = ::close(gpio_drv, gpio_fd)) < 0) {
      leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;
  }

  return 0;
error:
  return -1;
}

static int32_t leds_drv_LED0_read(void *const buf, size_t) {
  int32_t gpio_fd, rc;
  struct gpio_setup_req_s gpio_setup_req;
  const struct drv_model_cmn_s *gpio_drv;
  const uint16_t port_mask = 0b0001000000000000;
  uint16_t temp;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(gpio_drv = drv_ptr->dep("gpio"))) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = open(gpio_drv, "C", 3, 3u)) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = read(gpio_drv, gpio_fd, &temp, sizeof(uint16_t))) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Close gpio/C file */
  if ((rc = ::close(gpio_drv, gpio_fd)) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  *reinterpret_cast<uint16_t *>(buf) = (port_mask & temp) >> led0_gpio_pin_no;
  return 0;
error:
  return -1;
};

static int32_t leds_drv_LED0_write(const void *buf, size_t) {
  int32_t gpio_fd, rc;
  struct gpio_setup_req_s gpio_setup_req;
  const struct drv_model_cmn_s *gpio_drv;
  const uint16_t port_mask = 0b0001000000000000;
  uint16_t temp;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(gpio_drv = drv_ptr->dep("gpio"))) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = ::open(gpio_drv, "C", 3, 3u)) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::read(gpio_drv, gpio_fd, &temp, sizeof(uint16_t))) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  temp &= ~(port_mask);
  temp |= (*reinterpret_cast<const uint16_t *>(buf) << led0_gpio_pin_no);

  if ((rc = write(gpio_drv, gpio_fd, &temp, sizeof(uint16_t))) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Close gpio/C file */
  if ((rc = ::close(gpio_drv, gpio_fd)) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
};

static int32_t leds_drv_LED0_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t leds_drv_LED1_open(int32_t flags, mode_t mode) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t leds_drv_LED1_ioctl(uint64_t req, const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (req) {
  case LEDS_BLINK: {
    int32_t rc, gpio_fd;
    const struct drv_model_cmn_s *gpio_drv;
    const struct led_blink_req_s *strobe_req = reinterpret_cast<const struct led_blink_req_s *>(buf);
    const struct gpio_strobe_req_s gpio_req {
      .pin = led1_gpio_pin_no, .up_ticks = strobe_req->up_ticks, .down_ticks = strobe_req->down_ticks, .n_times = strobe_req->n_times, .delay_fn = strobe_req->delay_fn
    };

    if (!(gpio_drv = drv_ptr->dep("gpio"))) {
      leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((gpio_fd = ::open(gpio_drv, "C", 3, 3u)) < 0) {
      leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_STROBE, &gpio_req, sizeof(gpio_req))) < 0) {
      // errno = ???
      leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    /* Close gpio/C file */
    if ((rc = ::close(gpio_drv, gpio_fd)) < 0) {
      leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;
  }

  return 0;
error:
  return -1;
}

static int32_t leds_drv_LED1_read(void *const buf, size_t) {
  int32_t gpio_fd, rc;
  struct gpio_setup_req_s gpio_setup_req;
  const struct drv_model_cmn_s *gpio_drv;
  const uint16_t port_mask = 0b0010000000000000;
  uint16_t temp;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(gpio_drv = drv_ptr->dep("gpio"))) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = open(gpio_drv, "C", 3, 3u)) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = read(gpio_drv, gpio_fd, &temp, sizeof(uint16_t))) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Close gpio/C file */
  if ((rc = ::close(gpio_drv, gpio_fd)) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  *reinterpret_cast<uint16_t *>(buf) = (port_mask & temp) >> led1_gpio_pin_no;
  return 0;
error:
  return -1;
};

static int32_t leds_drv_LED1_write(const void *buf, size_t) {
  int32_t gpio_fd, rc;
  struct gpio_setup_req_s gpio_setup_req;
  const struct drv_model_cmn_s *gpio_drv;
  const uint16_t port_mask = 0b0010000000000000;
  uint16_t temp;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(gpio_drv = drv_ptr->dep("gpio"))) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = open(gpio_drv, "C", 3, 3u)) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = read(gpio_drv, gpio_fd, &temp, sizeof(uint16_t))) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  temp &= ~(port_mask);
  temp |= (*reinterpret_cast<const uint16_t *>(buf) << led1_gpio_pin_no);

  if ((rc = write(gpio_drv, gpio_fd, &temp, sizeof(uint16_t))) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Close gpio/C file */
  if ((rc = ::close(gpio_drv, gpio_fd)) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
};

static int32_t leds_drv_LED1_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t leds_drv_LED2_open(int32_t flags, mode_t mode) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t leds_drv_LED2_ioctl(uint64_t req, const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (req) {
  case LEDS_BLINK: {
    int32_t rc, gpio_fd;
    const struct drv_model_cmn_s *gpio_drv;
    const struct led_blink_req_s *strobe_req = reinterpret_cast<const struct led_blink_req_s *>(buf);
    const struct gpio_strobe_req_s gpio_req {
      .pin = led2_gpio_pin_no, .up_ticks = strobe_req->up_ticks, .down_ticks = strobe_req->down_ticks, .n_times = strobe_req->n_times, .delay_fn = strobe_req->delay_fn
    };

    if (!(gpio_drv = drv_ptr->dep("gpio"))) {
      leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((gpio_fd = ::open(gpio_drv, "C", 3, 3u)) < 0) {
      leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_STROBE, &gpio_req, sizeof(gpio_req))) < 0) {
      // errno = ???
      leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    /* Close gpio/C file */
    if ((rc = ::close(gpio_drv, gpio_fd)) < 0) {
      leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;
  }

  return 0;
error:
  return -1;
}

static int32_t leds_drv_LED2_read(void *const buf, size_t) {
  int32_t gpio_fd, rc;
  struct gpio_setup_req_s gpio_setup_req;
  const struct drv_model_cmn_s *gpio_drv;
  const uint16_t port_mask = 0b0100000000000000;
  uint16_t temp;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(gpio_drv = drv_ptr->dep("gpio"))) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = open(gpio_drv, "C", 3, 3u)) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = read(gpio_drv, gpio_fd, &temp, sizeof(uint16_t))) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Close gpio/C file */
  if ((rc = ::close(gpio_drv, gpio_fd)) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  *reinterpret_cast<uint16_t *>(buf) = (port_mask & temp) >> led2_gpio_pin_no;
  return 0;
error:
  return -1;
};

static int32_t leds_drv_LED2_write(const void *buf, size_t) {
  int32_t gpio_fd, rc;
  struct gpio_setup_req_s gpio_setup_req;
  const struct drv_model_cmn_s *gpio_drv;
  const uint16_t port_mask = 0b0100000000000000;
  ;
  uint16_t temp;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(gpio_drv = drv_ptr->dep("gpio"))) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = open(gpio_drv, "C", 3, 3u)) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = read(gpio_drv, gpio_fd, &temp, sizeof(uint16_t))) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  temp &= ~(port_mask);
  temp |= (*reinterpret_cast<const uint16_t *>(buf) << led2_gpio_pin_no);

  if ((rc = write(gpio_drv, gpio_fd, &temp, sizeof(uint16_t))) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Close gpio/C file */
  if ((rc = ::close(gpio_drv, gpio_fd)) < 0) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
};

static int32_t leds_drv_LED2_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t leds_printf(const char *fmt, ...) {
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
    leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (strlen) {
    if ((usart_fd = ::open(usart, console_devstr, 3, 3u)) < 0) {
      leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, "[leds] : ", std::strlen("[leds] : "))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }
      leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, temp, std::strlen(temp))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::close(usart, usart_fd)) < 0) {
      leds_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
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
