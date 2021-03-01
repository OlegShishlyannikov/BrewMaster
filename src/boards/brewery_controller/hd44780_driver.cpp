#include <cstdarg>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "arch/mcu_tags.hpp"
#include "boards/board_tags.hpp"
#include "drivers/hd44780_driver.hpp"
#include "drivers/io/gpio_ioctl.hpp"
#include "drivers/io/hd44780_ioctl.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "fio/unistd.hpp"
#include "util/fonts/unicode_5x8.hpp"

const struct drv_model_cmn_s *drv_ptr;

// GLobal flags
extern bool debug_log_enabled;

/* HD44780 driver lock */
static xSemaphoreHandle hd44780_lock;
extern xQueueHandle events_worker_queue;

static constexpr uint32_t hd44780_cols = 20u, hd44780_rows = 4u;
static uint8_t hd44780_ll_displayparams = 0u;
static wchar_t hd44780_ll_buffer_w[hd44780_cols + 1u];
static char hd44780_ll_buffer[hd44780_cols + 1u];

/* Pins used in HD44780 operating */
static const uint8_t hd44780_data_pins[]{0u, 1u, 2u, 3u, 4u, 5u, 6u, 7u};
static const uint8_t hd44780_bl_en_pin{8u};
static const uint8_t hd44780_rs_pin{9u};
static const uint8_t hd44780_rw_pin{10u};
static const uint8_t hd44780_e_pin{11u};

enum hd44780_instr_e : uint32_t {
  // The rest should be left alone
  HD44780_CLEARDISPLAY = 0x01,
  HD44780_RETURNHOME = 0x02,
  HD44780_ENTRYMODESET = 0x04,
  HD44780_DISPLAYCONTROL = 0x08,
  HD44780_CURSORSHIFT = 0x10,
  HD44780_FUNCTIONSET = 0x20,
  HD44780_SETCGRAMADDR = 0x40,
  HD44780_SETDDRAMADDR = 0x80,

  HD44780_ENTRYRIGHT = 0x00,
  HD44780_ENTRYLEFT = 0x02,
  HD44780_ENTRYSHIFTINCREMENT = 0x01,
  HD44780_ENTRYSHIFTDECREMENT = 0x00,

  HD44780_DISPLAYON = 0x04,
  HD44780_DISPLAYOFF = 0x00,
  HD44780_CURSORON = 0x02,
  HD44780_CURSOROFF = 0x00,
  HD44780_BLINKON = 0x01,
  HD44780_BLINKOFF = 0x00,

  HD44780_DISPLAYMOVE = 0x08,
  HD44780_CURSORMOVE = 0x00,
  HD44780_MOVERIGHT = 0x04,
  HD44780_MOVELEFT = 0x00,

  HD44780_8BITMODE = 0x10,
  HD44780_4BITMODE = 0x00,
  HD44780_2LINE = 0x08,
  HD44780_1LINE = 0x00,
  HD44780_5x10DOTS = 0x04,
  HD44780_5x8DOTS = 0x00,
};

/* HD44780 driver low level functions forward reference */
static int32_t hd44780_ll_init(const struct drv_model_cmn_s *, int32_t);
static int32_t hd44780_ll_command(const struct drv_model_cmn_s *, int32_t, uint8_t);
static int32_t hd44780_ll_write(const struct drv_model_cmn_s *, int32_t, wchar_t);
static int32_t hd44780_ll_write(const struct drv_model_cmn_s *, int32_t, uint8_t);
static int32_t hd44780_ll_read(const struct drv_model_cmn_s *, int32_t, uint8_t *const);
static int32_t hd44780_ll_read_busy(const struct drv_model_cmn_s *, int32_t, uint8_t *const);
static int32_t hd44780_ll_read_addr(const struct drv_model_cmn_s *, int32_t, uint8_t *const);
static int32_t hd44780_ll_on(const struct drv_model_cmn_s *, int32_t);
static int32_t hd44780_ll_off(const struct drv_model_cmn_s *, int32_t);
static int32_t hd44780_ll_clear(const struct drv_model_cmn_s *, int32_t);
static int32_t hd44780_ll_return_home(const struct drv_model_cmn_s *, int32_t);
static int32_t hd44780_ll_enable_blinking(const struct drv_model_cmn_s *, int32_t);
static int32_t hd44780_ll_disable_blinking(const struct drv_model_cmn_s *, int32_t);
static int32_t hd44780_ll_enable_cursor(const struct drv_model_cmn_s *, int32_t);
static int32_t hd44780_ll_disable_cursor(const struct drv_model_cmn_s *, int32_t);
static int32_t hd44780_ll_scroll_left(const struct drv_model_cmn_s *, int32_t);
static int32_t hd44780_ll_scroll_right(const struct drv_model_cmn_s *, int32_t);
static int32_t hd44780_ll_set_left_to_right(const struct drv_model_cmn_s *, int32_t);
static int32_t hd44780_ll_set_right_to_left(const struct drv_model_cmn_s *, int32_t);
static int32_t hd44780_ll_enable_autoscroll(const struct drv_model_cmn_s *, int32_t);
static int32_t hd44780_ll_disable_autoscroll(const struct drv_model_cmn_s *, int32_t);
static int32_t hd44780_ll_create_char(const struct drv_model_cmn_s *, int32_t, uint8_t, const uint8_t *);
static int32_t hd44780_ll_set_cursor(const struct drv_model_cmn_s *, int32_t, uint8_t, uint8_t);
static int32_t hd44780_ll_puts(const struct drv_model_cmn_s *, int32_t, const wchar_t *);
static int32_t hd44780_ll_printf(const struct drv_model_cmn_s *, int32_t, const wchar_t *, ...);
static int32_t hd44780_ll_puts(const struct drv_model_cmn_s *, int32_t, const char *);
static int32_t hd44780_ll_printf(const struct drv_model_cmn_s *, int32_t, const char *, ...);
static int32_t hd44780_ll_send(const struct drv_model_cmn_s *, int32_t, uint8_t, uint8_t);

/* HD44780 driver functions */
static int32_t hd44780_drv_open(int32_t, mode_t);
static int32_t hd44780_drv_ioctl(uint64_t, const void *, size_t);
static int32_t hd44780_drv_read(void *const, size_t);
static int32_t hd44780_drv_write(const void *, size_t);
static int32_t hd44780_drv_close();
static int32_t hd44780_printf(const char *fmt, ...);

/* HD44780 helper functions */
static int32_t hd44780_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreTakeRecursive(hd44780_lock, portIO_MAX_DELAY)) != pdPASS) {
    // errno = ???

    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t hd44780_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreGiveRecursive(hd44780_lock)) != pdPASS) {
    // errno = ???
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

struct drv_ops_s hd44780_drv_ops {
  .init = hd44780_drv_init, .exit = hd44780_drv_exit
};

/* GPIO driver file operations secification */
struct file_ops_s hd44780_drv_fops {
  .flock = hd44780_flock, .funlock = hd44780_funlock, .open = hd44780_drv_open, .ioctl = hd44780_drv_ioctl, .read = hd44780_drv_read, .write = hd44780_drv_write, .close = hd44780_drv_close
};

void hd44780_drv_init(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *gpio;
  struct gpio_setup_req_s gpio_req;
  int32_t gpio_fd, rc;

  /* Get gpio driver from dependencies */
  if (!(gpio = drv->dep("gpio"))) {
    // errno = ENOENT
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Open gpio device */
  if ((gpio_fd = ::open(gpio, "C", 3, 2u)) < 0) {
    // errno = ???
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Set up pins */
  for (uint8_t pin : hd44780_data_pins) {
    gpio_req = {.pin = pin};
    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_OD, &gpio_req, sizeof(gpio_req))) < 0) {
      // errno = ???
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  for (uint8_t pin : {hd44780_bl_en_pin, hd44780_rs_pin, hd44780_rw_pin, hd44780_e_pin}) {
    gpio_req = {.pin = pin};
    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_PP, &gpio_req, sizeof(gpio_req))) < 0) {
      // errno = ???
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  /* Init procedure */
  if ((rc = hd44780_ll_init(gpio, gpio_fd)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Close gpio file */
  if ((rc = ::close(gpio, gpio_fd)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  hd44780_drv_fops.owner = drv;

  /* Init locks */
  hd44780_lock = xSemaphoreCreateRecursiveMutex();

  /* Register char device for each GPIO port */
  drv->register_chardev("hd44780_lcd", &hd44780_drv_fops);

  /* Initialize GPIO driver pointer */
  drv_ptr = drv;
  return;

  /* Something went wrong -- reset drv_ptr and exit */
error:
  hd44780_drv_exit(drv);
  return;
}

void hd44780_drv_exit(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *gpio;
  struct gpio_setup_req_s gpio_req;
  int32_t gpio_fd, rc;

  /* Get gpio driver from dependencies */
  if (!(gpio = drv->dep("gpio"))) {
    // errno = ENOENT
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Open gpio device */
  if ((gpio_fd = ::open(gpio, "C", 3, 2u)) < 0) {
    // errno = ???
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Set up pins */
  for (uint8_t pin : hd44780_data_pins) {
    gpio_req = {.pin = pin};
    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_req, sizeof(gpio_req))) < 0) {
      // errno = ???
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  for (uint8_t pin : {hd44780_bl_en_pin, hd44780_rs_pin, hd44780_rw_pin, hd44780_e_pin}) {
    gpio_req = {.pin = pin};
    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_req, sizeof(gpio_req))) < 0) {
      // errno = ???
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  /* Close gpio file */
  if ((rc = ::close(gpio, gpio_fd)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  hd44780_drv_fops.owner = nullptr;

  /* Init locks */
  vSemaphoreDelete(hd44780_lock);

  /* Register char device for each GPIO port */
  drv->unregister_chardev("hd44780_lcd");

  /* Initialize GPIO driver pointer */
  drv_ptr = nullptr;
  return;

error:
  return;
};

static int32_t hd44780_drv_open(int32_t oflags, mode_t mode) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t hd44780_drv_ioctl(uint64_t req, const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (req) {
  case HD44780_BL_ENABLE: {
    int32_t rc, gpio_fd;
    const struct drv_model_cmn_s *gpio_drv;
    const struct gpio_write_pin_req_s gpio_req { .pin = hd44780_bl_en_pin, .val = 1u };

    if (!(gpio_drv = drv_ptr->dep("gpio"))) {
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((gpio_fd = ::open(gpio_drv, "C", 3, 3u)) < 0) {
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_PIN_WRITE, &gpio_req, sizeof(gpio_req))) < 0) {
      if ((rc = ::close(gpio_drv, gpio_fd)) < 0) {
        hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    /* Close gpio/C file */
    if ((rc = ::close(gpio_drv, gpio_fd)) < 0) {
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case HD44780_BL_DISABLE: {
    int32_t rc, gpio_fd;
    const struct drv_model_cmn_s *gpio_drv;
    const struct gpio_write_pin_req_s gpio_req { .pin = hd44780_bl_en_pin, .val = 0u };

    if (!(gpio_drv = drv_ptr->dep("gpio"))) {
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((gpio_fd = ::open(gpio_drv, "C", 3, 3u)) < 0) {
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::ioctl(gpio_drv, gpio_fd, gpio_ioctl_cmd_e::GPIO_PIN_WRITE, &gpio_req, sizeof(gpio_req))) < 0) {
      if ((rc = ::close(gpio_drv, gpio_fd)) < 0) {
        hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    /* Close gpio/C file */
    if ((rc = ::close(gpio_drv, gpio_fd)) < 0) {
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case HD44780_CLEAR: {
    int32_t rc, gpio_fd;
    const struct drv_model_cmn_s *gpio_drv;
    const struct gpio_write_pin_req_s gpio_req { .pin = hd44780_bl_en_pin, .val = 0u };

    if (!(gpio_drv = drv_ptr->dep("gpio"))) {
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((gpio_fd = ::open(gpio_drv, "C", 3, 3u)) < 0) {
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = hd44780_ll_clear(gpio_drv, gpio_fd)) < 0) {
      if ((rc = ::close(gpio_drv, gpio_fd)) < 0) {
        hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    /* Close gpio/C file */
    if ((rc = ::close(gpio_drv, gpio_fd)) < 0) {
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case HD44780_SET_CURSOR: {
    int32_t rc, gpio_fd;
    const struct drv_model_cmn_s *gpio_drv;
    const struct gpio_write_pin_req_s gpio_req { .pin = hd44780_bl_en_pin, .val = 0u };
    const struct hd44780_cursor_pos_req_s *cp_req = static_cast<const struct hd44780_cursor_pos_req_s *>(buf);

    if (!(gpio_drv = drv_ptr->dep("gpio"))) {
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((gpio_fd = ::open(gpio_drv, "C", 3, 3u)) < 0) {
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = hd44780_ll_set_cursor(gpio_drv, gpio_fd, cp_req->col, cp_req->line)) < 0) {
      if ((rc = ::close(gpio_drv, gpio_fd)) < 0) {
        hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    /* Close gpio/C file */
    if ((rc = ::close(gpio_drv, gpio_fd)) < 0) {
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;
  }

  return 0;
error:
  return -1;
}

static int32_t hd44780_drv_read(void *const buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t hd44780_drv_write(const void *buf, size_t size) {
  int32_t rc, gpio_fd;
  const struct drv_model_cmn_s *gpio_drv;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!buf) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(gpio_drv = drv_ptr->dep("gpio"))) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = ::open(gpio_drv, "C", 3, 3u)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  for (uint32_t i = 0u; i < size; i++) {
    if ((rc = hd44780_ll_write(gpio_drv, gpio_fd, reinterpret_cast<const uint8_t *>(buf)[i])) < 0) {
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((rc = ::close(gpio_drv, gpio_fd)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t hd44780_drv_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static void dwt_init(void) { CoreDebug->DEMCR = CoreDebug->DEMCR | CoreDebug_DEMCR_TRCENA_Msk; }

static inline void dwt_delay_us(uint32_t us) {
  int32_t us_count_tick = us * (SystemCoreClock / 1000000ul);
  DWT->CYCCNT = 0;
  DWT->CTRL = DWT->CTRL | DWT_CTRL_CYCCNTENA_Msk;
  while (DWT->CYCCNT < us_count_tick)
    ;
  DWT->CTRL = DWT->CTRL & ~DWT_CTRL_CYCCNTENA_Msk;
}

static inline void dwt_delay_ms(uint32_t ms) {
  int32_t ms_count_tick = ms * (SystemCoreClock / 1000ul);
  DWT->CYCCNT = 0;
  DWT->CTRL = DWT->CTRL | DWT_CTRL_CYCCNTENA_Msk;
  while (DWT->CYCCNT < ms_count_tick)
    ;
  DWT->CTRL = DWT->CTRL & ~DWT_CTRL_CYCCNTENA_Msk;
}

/* HD44780 driver low level functions definition */
static int32_t hd44780_ll_cmd(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd, uint8_t cmd) { return hd44780_ll_send(gpio_drv, gpio_fd, cmd, 0u); }

static int32_t hd44780_ll_write(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd, wchar_t val) {
  int32_t rc;
  uint8_t hd44780_ll_ddram_addr;

  if ((rc = hd44780_ll_read_addr(gpio_drv, gpio_fd, &hd44780_ll_ddram_addr)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = hd44780_ll_create_char(gpio_drv, gpio_fd, 0u, (font5x8.bitmap + (((val - (val ? 31u : 0u)) * 8u))))) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = hd44780_ll_cmd(gpio_drv, gpio_fd, HD44780_SETDDRAMADDR | hd44780_ll_ddram_addr)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return hd44780_ll_send(gpio_drv, gpio_fd, 0u, 1u);
error:
  return -1;
}

static int32_t hd44780_ll_write(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd, uint8_t val) { return hd44780_ll_send(gpio_drv, gpio_fd, val, 1u); }

static int32_t hd44780_ll_read(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd, uint8_t *const buffer) {
  int32_t rc;
  uint16_t gpio_val;

  struct gpio_setup_req_s gpio_setup_req {};
  struct gpio_strobe_req_s gpio_req {
    .pin = hd44780_e_pin, .up_ticks = 10u, .down_ticks = 10u, .n_times = 1u, .delay_fn = dwt_delay_us
  };

  struct gpio_write_pin_req_s gpio_wp_req {
    .pin = hd44780_e_pin, .val = 1u
  };

  if (!buffer) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::read(gpio_drv, gpio_fd, &gpio_val, sizeof(gpio_val))) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Set d0 - d7 up (drain-closed)
  gpio_val |= (1u << hd44780_rw_pin);  // RW up (to read)
  gpio_val &= ~(1u << hd44780_rs_pin); // RS down (command)

  /* Set pins as inputs */
  for (uint8_t pin : hd44780_data_pins) {
    gpio_setup_req = {.pin = pin};
    if ((rc = ::ioctl(gpio_drv, gpio_fd, GPIO_SP_IPH, &gpio_setup_req, sizeof(gpio_setup_req))) < 0) {
      // errno = ???
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((rc = ::write(gpio_drv, gpio_fd, &gpio_val, sizeof(gpio_val))) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // E high
  if ((rc = ::ioctl(gpio_drv, gpio_fd, GPIO_PIN_WRITE, &gpio_wp_req, sizeof(gpio_wp_req))) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Read while E is high
  if ((rc = ::read(gpio_drv, gpio_fd, &gpio_val, sizeof(gpio_val))) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // E low
  gpio_wp_req.val = 0u;
  if ((rc = ::ioctl(gpio_drv, gpio_fd, GPIO_PIN_WRITE, &gpio_wp_req, sizeof(gpio_wp_req))) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Set pins back as outputs */
  for (uint8_t pin : hd44780_data_pins) {
    gpio_setup_req = {.pin = pin};
    if ((rc = ::ioctl(gpio_drv, gpio_fd, GPIO_SP_OD, &gpio_setup_req, sizeof(gpio_setup_req))) < 0) {
      // errno = ???
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  *buffer = gpio_val & 0xff;
  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_read_busy(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd, uint8_t *const buffer) {
  int32_t rc;
  uint8_t hd44780_ll_val;

  if (!buffer) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::hd44780_ll_read(gpio_drv, gpio_fd, &hd44780_ll_val)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  *buffer = (hd44780_ll_val >> 7u);
  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_read_addr(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd, uint8_t *const buffer) {
  int32_t rc;
  uint8_t hd44780_ll_val;

  if (!buffer) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::hd44780_ll_read(gpio_drv, gpio_fd, &hd44780_ll_val)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  *buffer = (hd44780_ll_val & 0x3f);
  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_send(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd, uint8_t val, uint8_t mode) {
  int32_t rc;
  uint16_t gpio_buf;
  struct gpio_strobe_req_s gpio_req {
    .pin = hd44780_e_pin, .up_ticks = 10u, .down_ticks = 10u, .n_times = 1u, .delay_fn = dwt_delay_us
  };

  if ((rc = ::read(gpio_drv, gpio_fd, &gpio_buf, sizeof(gpio_buf))) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (mode) {
    gpio_buf |= (1u << hd44780_rs_pin);
  } else {
    gpio_buf &= ~(1u << hd44780_rs_pin);
  }

  gpio_buf &= ~(1u << hd44780_rw_pin);
  gpio_buf &= ~0xff;
  gpio_buf |= val;
  if ((rc = ::write(gpio_drv, gpio_fd, &gpio_buf, sizeof(gpio_buf))) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(gpio_drv, gpio_fd, GPIO_STROBE, &gpio_req, sizeof(gpio_req))) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_init(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd) {
  int32_t rc;
  uint16_t gpio_buf;

  if ((rc = ::read(gpio_drv, gpio_fd, &gpio_buf, sizeof(gpio_buf))) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  dwt_init();
  dwt_delay_ms(15u);

  gpio_buf = 0x30;
  if ((rc = hd44780_ll_send(gpio_drv, gpio_fd, gpio_buf, 0u)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  dwt_delay_ms(5u);

  if ((rc = hd44780_ll_send(gpio_drv, gpio_fd, gpio_buf, 0u)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  dwt_delay_us(100u);

  if ((rc = hd44780_ll_send(gpio_drv, gpio_fd, gpio_buf, 0u)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  dwt_delay_us(100u);
  gpio_buf = 0x3c;
  if ((rc = hd44780_ll_send(gpio_drv, gpio_fd, gpio_buf, 0u)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  dwt_delay_us(100u);
  gpio_buf = 0x08;
  if ((rc = hd44780_ll_send(gpio_drv, gpio_fd, gpio_buf, 0u)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  dwt_delay_us(100u);
  gpio_buf = 0x01;
  if ((rc = hd44780_ll_send(gpio_drv, gpio_fd, gpio_buf, 0u)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  dwt_delay_us(100u);
  gpio_buf = 0x06;
  if ((rc = hd44780_ll_send(gpio_drv, gpio_fd, gpio_buf, 0u)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  dwt_delay_us(55u);
  if ((rc = hd44780_ll_clear(gpio_drv, gpio_fd)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  dwt_delay_us(55u);
  if ((rc = hd44780_ll_on(gpio_drv, gpio_fd)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_on(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd) {
  int32_t rc;
  hd44780_ll_displayparams |= HD44780_DISPLAYON;
  if ((rc = hd44780_ll_cmd(gpio_drv, gpio_fd, HD44780_DISPLAYCONTROL | hd44780_ll_displayparams)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_off(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd) {
  int32_t rc;
  hd44780_ll_displayparams &= ~HD44780_DISPLAYON;
  if ((rc = hd44780_ll_cmd(gpio_drv, gpio_fd, HD44780_DISPLAYCONTROL | hd44780_ll_displayparams)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_clear(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd) {
  int32_t rc;
  if ((rc = hd44780_ll_cmd(gpio_drv, gpio_fd, HD44780_CLEARDISPLAY)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  dwt_delay_ms(2u);
  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_return_home(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd) {
  int32_t rc;
  if ((rc = hd44780_ll_cmd(gpio_drv, gpio_fd, HD44780_RETURNHOME)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  dwt_delay_ms(2u);
  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_enable_blinking(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd) {
  int32_t rc;
  hd44780_ll_displayparams |= HD44780_BLINKON;
  if ((rc = hd44780_ll_cmd(gpio_drv, gpio_fd, HD44780_DISPLAYCONTROL | hd44780_ll_displayparams)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_disable_blinking(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd) {
  int32_t rc;
  hd44780_ll_displayparams &= ~HD44780_BLINKON;
  if ((rc = hd44780_ll_cmd(gpio_drv, gpio_fd, HD44780_DISPLAYCONTROL | hd44780_ll_displayparams)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_enable_cursor(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd) {
  int32_t rc;
  hd44780_ll_displayparams |= HD44780_CURSORON;
  if ((rc = hd44780_ll_cmd(gpio_drv, gpio_fd, HD44780_DISPLAYCONTROL | hd44780_ll_displayparams)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_disable_cursor(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd) {
  int32_t rc;
  hd44780_ll_displayparams &= ~HD44780_CURSORON;
  if ((rc = hd44780_ll_cmd(gpio_drv, gpio_fd, HD44780_DISPLAYCONTROL | hd44780_ll_displayparams)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_scroll_left(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd) {
  int32_t rc;
  if ((rc = hd44780_ll_cmd(gpio_drv, gpio_fd, HD44780_CURSORSHIFT | HD44780_DISPLAYMOVE | HD44780_MOVELEFT)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_scroll_right(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd) {
  int32_t rc;
  if ((rc = hd44780_ll_cmd(gpio_drv, gpio_fd, HD44780_CURSORSHIFT | HD44780_DISPLAYMOVE | HD44780_MOVERIGHT)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_set_left_to_right(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd) {
  int32_t rc;
  hd44780_ll_displayparams |= HD44780_ENTRYLEFT;
  if ((rc = hd44780_ll_cmd(gpio_drv, gpio_fd, HD44780_ENTRYMODESET | hd44780_ll_displayparams)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_set_right_to_left(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd) {
  int32_t rc;
  hd44780_ll_displayparams &= ~HD44780_ENTRYLEFT;
  if ((rc = hd44780_ll_cmd(gpio_drv, gpio_fd, HD44780_ENTRYMODESET | hd44780_ll_displayparams)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_enable_autoscroll(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd) {
  int32_t rc;
  hd44780_ll_displayparams |= HD44780_ENTRYSHIFTINCREMENT;
  if ((rc = hd44780_ll_cmd(gpio_drv, gpio_fd, HD44780_ENTRYMODESET | hd44780_ll_displayparams)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_disable_autoscroll(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd) {
  int32_t rc;
  hd44780_ll_displayparams &= ~HD44780_ENTRYSHIFTINCREMENT;
  if ((rc = hd44780_ll_cmd(gpio_drv, gpio_fd, HD44780_ENTRYMODESET | hd44780_ll_displayparams)) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_create_char(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd, uint8_t loc, const uint8_t *charmap) {
  int32_t rc;
  if ((rc = hd44780_ll_cmd(gpio_drv, gpio_fd, HD44780_SETCGRAMADDR | ((loc & 0x7u) << 3u))) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  for (uint32_t i = 0u; i < 8u; i++) {
    if ((rc = hd44780_ll_send(gpio_drv, gpio_fd, charmap[i], 1u)) < 0) {
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_set_cursor(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd, uint8_t col, uint8_t row) {
  static uint8_t offsets[] = {0x00u, 0x40u, 0x14u, 0x54u};
  int32_t rc;
  if ((rc = hd44780_ll_cmd(gpio_drv, gpio_fd, HD44780_SETDDRAMADDR | (col + offsets[row]))) < 0) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_puts(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd, const wchar_t *str) {
  int32_t rc;
  for (const wchar_t *it = str; *it; it++) {
    if ((rc = hd44780_ll_write(gpio_drv, gpio_fd, (wchar_t)*it)) < 0) {
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_puts(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd, const char *str) {
  int32_t rc;
  for (const char *it = str; *it; it++) {
    if ((rc = hd44780_ll_write(gpio_drv, gpio_fd, (uint8_t)*it)) < 0) {
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_printf(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd, const wchar_t *format, ...) {
  int32_t rc;
  va_list args;

  va_start(args, format);
  if ((rc = std::vswprintf(hd44780_ll_buffer_w, hd44780_cols + 1u, format, args)) < 0) {
    va_end(args);
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }
  va_end(args);

  if ((rc = hd44780_ll_puts(gpio_drv, gpio_fd, hd44780_ll_buffer))) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t hd44780_ll_printf(const struct drv_model_cmn_s *gpio_drv, int32_t gpio_fd, const char *format, ...) {
  int32_t rc;
  va_list args;

  va_start(args, format);
  if ((rc = std::vsnprintf(hd44780_ll_buffer, hd44780_cols + 1u, format, args)) < 0) {
    va_end(args);
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }
  va_end(args);

  if ((rc = hd44780_ll_puts(gpio_drv, gpio_fd, hd44780_ll_buffer))) {
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t hd44780_printf(const char *fmt, ...) {
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
    hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (strlen) {
    if ((usart_fd = ::open(usart, "usart1", 3, 3u)) < 0) {
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, "[hd44780] : ", std::strlen("[hd44780] : "))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, temp, std::strlen(temp))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::close(usart, usart_fd)) < 0) {
      hd44780_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
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
