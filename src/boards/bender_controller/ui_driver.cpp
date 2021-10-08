#include <cctype>
#include <cstdarg>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "arch/mcu_tags.hpp"
#include "boards/board_tags.hpp"
#include "drivers/io/74hc595_ioctl.hpp"
#include "drivers/io/buzzer_ioctl.hpp"
#include "drivers/io/gpio_ioctl.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "drivers/io/spi_ioctl.hpp"
#include "drivers/io/timer_ioctl.hpp"
#include "drivers/io/ui_ioctl.hpp"
#include "drivers/io/uln2003_ioctl.hpp"
#include "drivers/io/usart_ioctl.hpp"
#include "drivers/ui_driver.hpp"
#include "fio/unistd.hpp"
#include "sys/events_worker.hpp"

/* This pointer will be used in interrupt handlers and will be initialized in driver init function */
static const struct drv_model_cmn_s *drv_ptr;
extern bool debug_log_enabled;
extern xQueueHandle events_worker_queue;

/* Latch, clock and data pins used */
static constexpr uint8_t btn1_5_pin = 0u, btn6_9_pin = 1u, pedal_pin = 15u;
static constexpr const char *btn_pin_gpio_port_letter = "C";
static constexpr uint32_t ui_drv_buffer_size = 32u;
static constexpr const char *const console_devstr = "usart1";

static constexpr const uint16_t segment_digits[] = {0b01111110, 0b1100, 0b10110110, 0b10011110, 0b11001100, 0b11011010, 0b11111010, 0b1110, 0b11111110, 0b11011110, 0b0};
static constexpr const uint16_t segment_minus = 0b10000000;
static struct {
  struct {
    uint8_t first;
    uint8_t second;
  } up_left;

  struct {
    uint8_t first;
    uint8_t second;
    uint8_t third;
    uint8_t forth;
  } up_right;

  struct {
    uint8_t first;
    uint8_t second;
  } down_left;

  struct {
    uint8_t first;
    uint8_t second;
    uint8_t third;
    uint8_t forth;
  } down_right;
} fb;

/* 74HC595 lock & fifos */
static xSemaphoreHandle ui_lock;
extern xQueueHandle events_worker_queue;
static xQueueHandle ui_fifo;

static void (*btn_press_cbks[9u])(const void *, size_t){nullptr};
static void (*btn_release_cbks[9u])(const void *, size_t){nullptr};
static void (*pedal_press_cbk_ptr)(const void *, size_t){nullptr};
static void (*pedal_release_cbk_ptr)(const void *, size_t){nullptr};

// Printf to console
static int32_t ui_printf(const char *fmt, ...);

/* 74HC595 file IO functions forward reference */
static int32_t ui_drv_open(int32_t, mode_t);
static int32_t ui_drv_ioctl(uint64_t, const void *, size_t);
static int32_t ui_drv_read(void *const, size_t);
static int32_t ui_drv_write(const void *, size_t);
static int32_t ui_drv_close();

static void ui_bgr_task(const void *, size_t);
static void show_fb();
static void beep_short();
  
static void key1_press_callback(const void *, size_t);
static void key2_press_callback(const void *, size_t);
static void key3_press_callback(const void *, size_t);
static void key4_press_callback(const void *, size_t);
static void key5_press_callback(const void *, size_t);
static void key6_press_callback(const void *, size_t);
static void key7_press_callback(const void *, size_t);
static void key8_press_callback(const void *, size_t);
static void key9_press_callback(const void *, size_t);

static void key1_release_callback(const void *, size_t);
static void key2_release_callback(const void *, size_t);
static void key3_release_callback(const void *, size_t);
static void key4_release_callback(const void *, size_t);
static void key5_release_callback(const void *, size_t);
static void key6_release_callback(const void *, size_t);
static void key7_release_callback(const void *, size_t);
static void key8_release_callback(const void *, size_t);
static void key9_release_callback(const void *, size_t);

static void pedal_press_callback(const void *, size_t);
static void pedal_release_callback(const void *, size_t);

/* 74HC595 helper functions */
static int32_t ui_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreTakeRecursive(ui_lock, portMAX_DELAY)) != pdPASS) {
    // errno = ???
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t ui_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreGiveRecursive(ui_lock)) != pdPASS) {
    // errno = ???
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

struct drv_ops_s ui_drv_ops {
  .init = ui_drv_init, .exit = ui_drv_exit
};

/* 74HC595 driver file operations secification */
struct file_ops_s ui_drv_fops {
  .flock = ui_flock, .funlock = ui_funlock, .open = ui_drv_open, .ioctl = ui_drv_ioctl, .read = ui_drv_read, .write = ui_drv_write, .close = ui_drv_close
};

void ui_drv_init(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *gpio, *tim;
  int32_t tim_fd, gpio_fd, rc;
  struct gpio_setup_req_s gpio_req;
  struct timer_setup_req_s tim_req {
    .cnt_mode = timer_setup_cnt_mode_e::TIM_CNT_MODE_UP, .psc = 8000u, .period = 25u, .clk_div = timer_setup_clkdiv_e::TIM_CLKDIV1, .rep_cnt = 0u, .irq_priority = 5u
  };

  struct timer_cbk_req_s tim_cbk_req {
    .callback = ui_bgr_task
  };

  // Setup buttons GPIO
  if (!(gpio = drv->dep("gpio"))) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = ::open(gpio, btn_pin_gpio_port_letter, 3, 3u)) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  for (uint8_t pin : {btn1_5_pin, btn6_9_pin}) {
    gpio_req.pin = pin;
    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IPH, &gpio_req, sizeof(gpio_req))) < 0) {
      if ((rc = ::close(tim, tim_fd)) < 0) {
        ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((rc = ::close(gpio, gpio_fd)) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Setup timer
  if (!(tim = drv->dep("timer"))) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((tim_fd = ::open(tim, "TIM1", 3, 3u)) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(tim, tim_fd, timer_ioctl_cmd_e::TIMER_INIT, &tim_req, sizeof(tim_req))) < 0) {
    if ((rc = ::close(tim, tim_fd)) < 0) {
      ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(tim, tim_fd, timer_ioctl_cmd_e::TIMER_CBK_SET, &tim_cbk_req, sizeof(tim_cbk_req))) < 0) {
    if ((rc = ::close(tim, tim_fd)) < 0) {
      ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(tim, tim_fd, timer_ioctl_cmd_e::TIMER_START, nullptr, 0u)) < 0) {
    if ((rc = ::close(tim, tim_fd)) < 0) {
      ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(tim, tim_fd)) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  ui_drv_fops.owner = drv;

  /* Init locks */
  ui_lock = xSemaphoreCreateRecursiveMutex();
  ui_fifo = xQueueCreate(ui_drv_buffer_size, sizeof(char));
  xSemaphoreGive(ui_lock);

  /* Register char device for each GPIO port */
  drv->register_chardev("ui", &ui_drv_fops);

  /* Initialize GPIO driver pointer */
  drv_ptr = drv;
  return;

  /* Something went wrong -- reset drv_ptr and exit */
error:
  ui_drv_exit(drv);
  return;
}

void ui_drv_exit(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *gpio, *tim;
  int32_t tim_fd, gpio_fd, rc;
  struct gpio_setup_req_s gpio_req;

  // Setup buttons GPIO
  if (!(gpio = drv->dep("gpio"))) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = ::open(gpio, btn_pin_gpio_port_letter, 3, 3u)) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  for (uint8_t pin : {btn1_5_pin, btn6_9_pin}) {
    gpio_req.pin = pin;
    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_req, sizeof(gpio_req))) < 0) {
      if ((rc = ::close(tim, tim_fd)) < 0) {
        ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((rc = ::close(gpio, gpio_fd)) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Setup timer
  if (!(tim = drv->dep("timer"))) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((tim_fd = ::open(tim, "TIM1", 3, 3u)) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(tim, tim_fd, timer_ioctl_cmd_e::TIMER_STOP, nullptr, 0u)) < 0) {
    if ((rc = ::close(tim, tim_fd)) < 0) {
      ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(tim, tim_fd, timer_ioctl_cmd_e::TIMER_CBK_RESET, nullptr, 0u)) < 0) {
    if ((rc = ::close(tim, tim_fd)) < 0) {
      ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(tim, tim_fd, timer_ioctl_cmd_e::TIMER_DEINIT, nullptr, 0u)) < 0) {
    if ((rc = ::close(tim, tim_fd)) < 0) {
      ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(tim, tim_fd)) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  ui_drv_fops.owner = nullptr;

  /* Remove locks */
  vSemaphoreDelete(ui_lock);
  vQueueDelete(ui_fifo);

  /* Register char device for each GPIO port */
  drv->unregister_chardev("ui");

  /* Initialize GPIO driver pointer */
  drv_ptr = nullptr;
  return;
error:
  return;
}

/* 74HC595 file IO functions */
static int32_t ui_drv_open(int32_t, mode_t) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t ui_drv_ioctl(uint64_t req, const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (req) {
  case UI_DISP_LEFT_UP_SET_NUM ... UI_DISP_LEFT_DOWN_SET_NUM: {
    const struct ui_2d_disp_set_num_req_s *request = reinterpret_cast<const struct ui_2d_disp_set_num_req_s *>(buf);
    for (uint32_t i = 0u; i < sizeof(request->num_str) / sizeof(request->num_str[0]); i++) {
      if (std::isdigit(request->num_str[i])) {
        switch (i) {
        case 0u:
          if (req == UI_DISP_LEFT_UP_SET_NUM) {
            fb.up_left.first = request->num_str[i] - '0';
          } else {
            fb.down_left.first = request->num_str[i] - '0';
          }
          break;

        case 1u:
          if (req == UI_DISP_LEFT_UP_SET_NUM) {
            fb.up_left.second = request->num_str[i] - '0';
          } else {
            fb.down_left.second = request->num_str[i] - '0';
          }
          break;

        default:
          break;
        }
      } else {
        switch (i) {
        case 0u:
          fb.up_left.first = ' ';
          break;

        case 1u:
          fb.up_left.second = ' ';
          break;

        default:
          break;
        }
      }
    }
  } break;

  case UI_DISP_RIGHT_UP_SET_NUM ... UI_DISP_RIGHT_DOWN_SET_NUM: {
    const struct ui_4d_disp_set_num_req_s *request = reinterpret_cast<const struct ui_4d_disp_set_num_req_s *>(buf);
    if (request->is_negative) {
      if (req == UI_DISP_RIGHT_UP_SET_NUM) {
        fb.up_right.first = '-';
      } else {
        fb.down_right.first = '-';
      }
    } else {
      if (req == UI_DISP_RIGHT_UP_SET_NUM) {
        fb.up_right.first = ' ';
      } else {
        fb.down_right.first = ' ';
      }
    }

    for (uint32_t i = 0u; i < sizeof(request->num_str) / sizeof(request->num_str[0]); i++) {
      if (std::isdigit(request->num_str[i])) {
        switch (i) {
        case 0u:
          if (req == UI_DISP_RIGHT_UP_SET_NUM) {
            fb.up_right.second = request->num_str[i] - '0';
          } else {
            fb.down_right.second = request->num_str[i] - '0';
          }
          break;

        case 1u:
          if (req == UI_DISP_RIGHT_UP_SET_NUM) {
            fb.up_right.third = request->num_str[i] - '0';
          } else {
            fb.down_right.third = request->num_str[i] - '0';
          }
          break;

        case 2u:
          if (req == UI_DISP_RIGHT_UP_SET_NUM) {
            fb.up_right.forth = request->num_str[i] - '0';
          } else {
            fb.down_right.forth = request->num_str[i] - '0';
          }
          break;

        default:
          break;
        }
      } else {
        switch (i) {
        case 0u:
          if (req == UI_DISP_RIGHT_UP_SET_NUM) {
            fb.up_right.second = ' ';
          } else {
            fb.down_right.second = ' ';
          }
          break;

        case 1u:
          if (req == UI_DISP_RIGHT_UP_SET_NUM) {
            fb.up_right.third = ' ';
          } else {
            fb.down_right.third = ' ';
          }
          break;

        case 2u:
          if (req == UI_DISP_RIGHT_UP_SET_NUM) {
            fb.up_right.forth = ' ';
          } else {
            fb.down_right.forth = ' ';
          }
          break;

        default:
          break;
        }
      }
    }
  } break;

  case UI_DISP_LEFT_UP_RESET ... UI_DISP_LEFT_DOWN_RESET: {
    if (req == UI_DISP_LEFT_UP_RESET) {

      fb.up_left.first = ' ';
      fb.up_left.second = ' ';
    } else {

      fb.down_left.first = ' ';
      fb.down_left.second = ' ';
    }
  } break;

  case UI_DISP_RIGHT_UP_RESET ... UI_DISP_RIGHT_DOWN_RESET: {
    if (req == UI_DISP_RIGHT_UP_RESET) {

      fb.up_right.first = ' ';
      fb.up_right.second = ' ';
      fb.up_right.third = ' ';
      fb.up_right.forth = ' ';
    } else {

      fb.down_right.first = ' ';
      fb.down_right.second = ' ';
      fb.down_right.third = ' ';
      fb.down_right.forth = ' ';
    }
  } break;

  case UI_KEY_CBK_SET: {
    const struct ui_key_cbk_req_s *request = reinterpret_cast<const struct ui_key_cbk_req_s *>(buf);
    switch (request->type) {
    case UI_KEY_CBK_PRESS: {
      btn_press_cbks[request->key_no] = request->cbk;
    } break;

    case UI_KEY_CBK_RELEASE: {
      btn_release_cbks[request->key_no] = request->cbk;
    } break;

    default:
      break;
    }
  } break;

  case UI_KEY_CBK_RESET: {
    const struct ui_key_cbk_req_s *request = reinterpret_cast<const struct ui_key_cbk_req_s *>(buf);
    switch (request->type) {
    case UI_KEY_CBK_PRESS: {
      btn_press_cbks[request->key_no] = nullptr;
    } break;

    case UI_KEY_CBK_RELEASE: {
      btn_release_cbks[request->key_no] = nullptr;
    } break;

	default:
      break;
    }
  } break;

  case UI_PEDAL_CBK_SET: {
    const struct ui_pedal_cbk_req_s *request = reinterpret_cast<const struct ui_pedal_cbk_req_s *>(buf);
    switch (request->type) {
    case UI_KEY_CBK_PRESS: {
      pedal_press_cbk_ptr = request->cbk;
    } break;

    case UI_KEY_CBK_RELEASE: {
      pedal_release_cbk_ptr = request->cbk;
    } break;
    default:
      break;
    }
  } break;

  case UI_PEDAL_CBK_RESET: {
    const struct ui_pedal_cbk_req_s *request = reinterpret_cast<const struct ui_pedal_cbk_req_s *>(buf);
    switch (request->type) {
    case UI_KEY_CBK_PRESS: {
      pedal_press_cbk_ptr = nullptr;
    } break;
    case UI_KEY_CBK_RELEASE: {
      pedal_release_cbk_ptr = nullptr;
    } break;
    default:
      break;
    }
  } break;
  default:
    break;
  }

  return 0;
error:
  return -1;
}

static int32_t ui_drv_read(void *const, size_t) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t ui_drv_write(const void *buf, size_t size) {
  const struct drv_model_cmn_s *spi, *gpio;
  int32_t spi_fd, rc;
  struct spi_send_seq_req_s spi_seq_req;
  const uint16_t *data;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(spi = drv_ptr->dep("spi"))) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(gpio = drv_ptr->dep("gpio"))) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Select 74HC595 chip
  if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_SELECT, nullptr, 0u)) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Write sequence to 74HC595
  /* Assuming that buffer has data type and size of uint16 */
  spi_seq_req.seq = *reinterpret_cast<const uint16_t *>(buf);
  if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_SEND_SEQ, &spi_seq_req, sizeof(spi_seq_req))) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Unselect 74HC595 chip
  if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_UNSELECT, nullptr, 0u)) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t ui_drv_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t ui_printf(const char *fmt, ...) {
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
	
    if ((rc = ::write(usart, usart_fd, "[ui] : ", std::strlen("[ui] : "))) < 0) {
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

static void ui_bgr_task(const void *daat, size_t size) {
  const struct drv_model_cmn_s *ic74hc595, *uln2003, *gpio;
  int32_t rc, gpio_fd, uln2003_fd, ic74hc595_fd;
  uint16_t ic74hc595_val_to_write, zero = 0u;
  static uint8_t current_uln2003_ch_num = 1u;
  uint16_t gpio_val;
  static bool key1_pressed = false, key2_pressed = false, key3_pressed = false, key4_pressed = false, key5_pressed = false, key6_pressed = false, key7_pressed = false, key8_pressed = false,
	key9_pressed = false, pedal_pressed = false;
  static uint32_t key1_press_time = 0u, key2_press_time = 0u, key3_press_time = 0u, key4_press_time = 0u, key5_press_time = 0u, key6_press_time = 0u, key7_press_time = 0u, key8_press_time = 0u,
	key9_press_time = 0u, pedal_press_time = 0u;
  static constexpr const uint32_t key_press_time_threshold = 2u, pedal_press_time_threshold = 10u;
  static struct uln2003_ch_on_off_req_s uln2003_ch_req = {.ch_num = current_uln2003_ch_num};

  // Show framebuffer and read buttons gpio meanwhile
  // Check dependencies
  if (!(gpio = drv_ptr->dep("gpio"))) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(uln2003 = drv_ptr->dep("uln2003"))) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(ic74hc595 = drv_ptr->dep("ic74hc595"))) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // First channel of ULN2003 ON, read buttons, draw on display
  if ((uln2003_fd = ::open(uln2003, "uln2003", 3u, 2)) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Switch ULN2003 channel
  if ((rc = ::ioctl(uln2003, uln2003_fd, uln2003_ioctl_cmd_e::ULN2003_CHANNEL_OFF, &uln2003_ch_req, sizeof(uln2003_ch_req))) < 0) {
    if ((uln2003_fd = ::close(uln2003, uln2003_fd)) < 0) {
      ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  current_uln2003_ch_num == 6u ? current_uln2003_ch_num = 1u : current_uln2003_ch_num++;
  uln2003_ch_req.ch_num = current_uln2003_ch_num;

  if ((ic74hc595_fd = ::open(ic74hc595, "ic74hc595", 3u, 2)) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::write(ic74hc595, ic74hc595_fd, &zero, sizeof(zero))) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((ic74hc595_fd = ::close(ic74hc595, ic74hc595_fd)) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(uln2003, uln2003_fd, uln2003_ioctl_cmd_e::ULN2003_CHANNEL_ON, &uln2003_ch_req, sizeof(uln2003_ch_req))) < 0) {
    if ((uln2003_fd = ::close(uln2003, uln2003_fd)) < 0) {
      ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(uln2003, uln2003_fd)) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((gpio_fd = ::open(gpio, btn_pin_gpio_port_letter, 3u, 2)) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::read(gpio, gpio_fd, &gpio_val, sizeof(gpio_val))) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(gpio, gpio_fd)) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (uln2003_ch_req.ch_num) {
  case 1u: {
    ic74hc595_val_to_write = (fb.up_right.forth != '-' ? (fb.up_right.forth != ' ' ? segment_digits[fb.up_right.forth] : 0x00) : segment_minus) |
                             (fb.down_right.forth != '-' ? (fb.down_right.forth != ' ' ? (segment_digits[fb.down_right.forth] << 8u) : 0x00) : (segment_minus << 8u));
  } break;

  case 2u: {
    ic74hc595_val_to_write = (fb.up_right.third != '-' ? (fb.up_right.third != ' ' ? segment_digits[fb.up_right.third] : 0x00) : segment_minus) |
                             (fb.down_right.third != '-' ? (fb.down_right.third != ' ' ? (segment_digits[fb.down_right.third] << 8u) : 0x00) : (segment_minus << 8u));
  } break;

  case 3u: {
    ic74hc595_val_to_write = (fb.up_right.second != '-' ? (fb.up_right.second != ' ' ? segment_digits[fb.up_right.second] : 0x00) : segment_minus) |
                             (fb.down_right.second != '-' ? (fb.down_right.second != ' ' ? (segment_digits[fb.down_right.second] << 8u) : 0x00) : (segment_minus << 8u));
  } break;

  case 4u: {
    ic74hc595_val_to_write = (fb.up_right.first != '-' ? (fb.up_right.first != ' ' ? segment_digits[fb.up_right.first] : 0x00) : segment_minus) |
                             (fb.down_right.first != '-' ? (fb.down_right.first != ' ' ? (segment_digits[fb.down_right.first] << 8u) : 0x00) : (segment_minus << 8u));
  } break;

  case 5u: {
    ic74hc595_val_to_write = (fb.up_left.second != '-' ? (fb.up_left.second != ' ' ? segment_digits[fb.up_left.second] : 0x00) : segment_minus) |
                             (fb.down_left.second != '-' ? (fb.down_left.second != ' ' ? (segment_digits[fb.down_left.second] << 8u) : 0x00) : (segment_minus << 8u));
  } break;

  case 6u: {
    ic74hc595_val_to_write = (fb.up_left.first != '-' ? (fb.up_left.first != ' ' ? segment_digits[fb.up_left.first] : 0x00) : segment_minus) |
                             (fb.down_left.first != '-' ? (fb.down_left.first != ' ' ? (segment_digits[fb.down_left.first] << 8u) : 0x00) : (segment_minus << 8u));
  } break;
  default:
    break;
  }

  if ((ic74hc595_fd = ::open(ic74hc595, "ic74hc595", 3u, 2)) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::write(ic74hc595, ic74hc595_fd, &ic74hc595_val_to_write, sizeof(ic74hc595_val_to_write))) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((ic74hc595_fd = ::close(ic74hc595, ic74hc595_fd)) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (uln2003_ch_req.ch_num) {
  case 2u: {
    if (!key5_pressed && !((gpio_val >> btn1_5_pin) & 0x01u) && (!key5_pressed ? key5_press_time++ : key5_press_time) >= key_press_time_threshold) {
      key5_pressed = !((gpio_val >> btn1_5_pin) & 0x01u);
      key5_press_time = 0u;
      key5_press_callback(nullptr, 0u);
    } else if (key5_pressed && !(!((gpio_val >> btn1_5_pin) & 0x01u)) && (key5_pressed ? key5_press_time++ : key5_press_time) >= key_press_time_threshold) {
      key5_pressed = !((gpio_val >> btn1_5_pin) & 0x01u);
      key5_press_time = 0u;
      key5_release_callback(nullptr, 0u);
    }
  } break;

  case 3u: {
    if (!key4_pressed && !((gpio_val >> btn1_5_pin) & 0x01u) && (!key4_pressed ? key4_press_time++ : key4_press_time) >= key_press_time_threshold) {
      key4_pressed = !((gpio_val >> btn1_5_pin) & 0x01u);
      key4_press_time = 0u;
      key4_press_callback(nullptr, 0u);
    } else if (key4_pressed && !(!((gpio_val >> btn1_5_pin) & 0x01u)) && (key4_pressed ? key4_press_time++ : key4_press_time) >= key_press_time_threshold) {
      key4_pressed = !((gpio_val >> btn1_5_pin) & 0x01u);
      key4_press_time = 0u;
      key4_release_callback(nullptr, 0u);
    } else if (key4_pressed && !((gpio_val >> btn1_5_pin) & 0x01u) && (key4_pressed ? key4_press_time++ : key4_press_time) >= key_press_time_threshold * 2u) {
      key4_press_time = 0u;
      key4_press_callback(nullptr, 0u);
    }

    if (!key9_pressed && !((gpio_val >> btn6_9_pin) & 0x01u) && (!key9_pressed ? key9_press_time++ : key9_press_time) >= key_press_time_threshold) {
      key9_pressed = !((gpio_val >> btn6_9_pin) & 0x01u);
      key9_press_time = 0u;
      key9_press_callback(nullptr, 0u);
    } else if (key9_pressed && !(!((gpio_val >> btn6_9_pin) & 0x01u)) && (key9_pressed ? key9_press_time++ : key9_press_time) >= key_press_time_threshold) {
      key9_pressed = !((gpio_val >> btn6_9_pin) & 0x01u);
      key9_press_time = 0u;
      key9_release_callback(nullptr, 0u);
    }
  } break;

  case 4u: {
    if (!key3_pressed && !((gpio_val >> btn1_5_pin) & 0x01u) && (!key3_pressed ? key3_press_time++ : key3_press_time) >= key_press_time_threshold) {
      key3_pressed = !((gpio_val >> btn1_5_pin) & 0x01u);
      key3_press_time = 0u;
      key3_press_callback(nullptr, 0u);
    } else if (key3_pressed && !(!((gpio_val >> btn1_5_pin) & 0x01u)) && (key3_pressed ? key3_press_time++ : key3_press_time) >= key_press_time_threshold) {
      key3_pressed = !((gpio_val >> btn1_5_pin) & 0x01u);
      key3_press_time = 0u;
      key3_release_callback(nullptr, 0u);
    } else if (key3_pressed && !((gpio_val >> btn1_5_pin) & 0x01u) && (key3_pressed ? key3_press_time++ : key3_press_time) >= key_press_time_threshold * 2u) {
      key3_press_time = 0u;
      key3_press_callback(nullptr, 0u);
    }

    if (!key8_pressed && !((gpio_val >> btn6_9_pin) & 0x01u) && (!key8_pressed ? key8_press_time++ : key8_press_time) >= key_press_time_threshold) {
      key8_pressed = !((gpio_val >> btn6_9_pin) & 0x01u);
      key8_press_time = 0u;
      key8_press_callback(nullptr, 0u);
    } else if (key8_pressed && !(!((gpio_val >> btn6_9_pin) & 0x01u)) && (key8_pressed ? key8_press_time++ : key8_press_time) >= key_press_time_threshold) {
      key8_pressed = !((gpio_val >> btn6_9_pin) & 0x01u);
      key8_press_time = 0u;
      key8_release_callback(nullptr, 0u);
    }
  } break;

  case 5u: {
    if (!key2_pressed && !((gpio_val >> btn1_5_pin) & 0x01u) && (!key2_pressed ? key2_press_time++ : key2_press_time) >= key_press_time_threshold) {
      key2_pressed = !((gpio_val >> btn1_5_pin) & 0x01u);
      key2_press_time = 0u;
      key2_press_callback(nullptr, 0u);
    } else if (key2_pressed && !(!((gpio_val >> btn1_5_pin) & 0x01u)) && (key2_pressed ? key2_press_time++ : key2_press_time) >= key_press_time_threshold) {
      key2_pressed = !((gpio_val >> btn1_5_pin) & 0x01u);
      key2_press_time = 0u;
      key2_release_callback(nullptr, 0u);
    }

    if (!key7_pressed && !((gpio_val >> btn6_9_pin) & 0x01u) && (!key7_pressed ? key7_press_time++ : key7_press_time) >= key_press_time_threshold) {
      key7_pressed = !((gpio_val >> btn6_9_pin) & 0x01u);
      key7_press_time = 0u;
      key7_press_callback(nullptr, 0u);
    } else if (key7_pressed && !(!((gpio_val >> btn6_9_pin) & 0x01u)) && (key7_pressed ? key7_press_time++ : key7_press_time) >= key_press_time_threshold) {
      key7_pressed = !((gpio_val >> btn6_9_pin) & 0x01u);
      key7_press_time = 0u;
      key7_release_callback(nullptr, 0u);
    }
  } break;

  case 6u: {
    if (!key1_pressed && !((gpio_val >> btn1_5_pin) & 0x01u) && (!key1_pressed ? key1_press_time++ : key1_press_time) >= key_press_time_threshold) {
      key1_pressed = !((gpio_val >> btn1_5_pin) & 0x01u);
      key1_press_time = 0u;
      key1_press_callback(nullptr, 0u);
    } else if (key1_pressed && !(!((gpio_val >> btn1_5_pin) & 0x01u)) && (key1_pressed ? key1_press_time++ : key1_press_time) >= key_press_time_threshold) {
      key1_pressed = !((gpio_val >> btn1_5_pin) & 0x01u);
      key1_press_time = 0u;
      key1_release_callback(nullptr, 0u);
    }

    if (!key6_pressed && !((gpio_val >> btn6_9_pin) & 0x01u) && (!key6_pressed ? key6_press_time++ : key6_press_time) >= key_press_time_threshold) {
      key6_pressed = !((gpio_val >> btn6_9_pin) & 0x01u);
      key6_press_time = 0u;
      key6_press_callback(nullptr, 0u);
    } else if (key6_pressed && !(!((gpio_val >> btn6_9_pin) & 0x01u)) && (key6_pressed ? key6_press_time++ : key6_press_time) >= key_press_time_threshold) {
      key6_pressed = !((gpio_val >> btn6_9_pin) & 0x01u);
      key6_press_time = 0u;
      key6_release_callback(nullptr, 0u);
    }

  } break;

  case 1u:
  default:
    break;
  }

  if (!pedal_pressed && !((gpio_val >> pedal_pin) & 0x01u) && (!pedal_pressed ? pedal_press_time++ : pedal_press_time) >= pedal_press_time_threshold) {
    pedal_pressed = !((gpio_val >> pedal_pin) & 0x01u);
    pedal_press_time = 0u;
    pedal_press_callback(nullptr, 0u);
  } else if (pedal_pressed && !(!((gpio_val >> pedal_pin) & 0x01u)) && (pedal_pressed ? pedal_press_time++ : pedal_press_time) > pedal_press_time_threshold) {
    pedal_pressed = !((gpio_val >> pedal_pin) & 0x01u);
    pedal_press_time = 0u;
    pedal_release_callback(nullptr, 0u);
  }

  return;
error:
  return;
}

static void beep_short() {
  const struct drv_model_cmn_s *buzzer;
  int32_t rc, buzzer_fd;

  struct buzzer_beep_req_s req {
    .delay_fn = vTaskDelay, .up = 1u, .down = 0u, .n = 1u
  };

  if (!(buzzer = drv_ptr->dep("buzzer"))) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((buzzer_fd = ::open(buzzer, "buzzer0", 3, 3u)) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(buzzer, buzzer_fd, buzzer_drv_ioctl_cmd_e::BUZZER_BEEP, &req, sizeof(req))) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(buzzer, buzzer_fd)) < 0) {
    ui_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

exit:
  return;
error:
  return;
}

static void key1_press_callback(const void *data, size_t size) {
  if (btn_press_cbks[0]) {
    btn_press_cbks[0](data, size);
  }
}

static void key2_press_callback(const void *data, size_t size) {
  if (btn_press_cbks[1u]) {
    btn_press_cbks[1u](data, size);
  }
}

static void key3_press_callback(const void *data, size_t size) {
  if (btn_press_cbks[2u]) {
    btn_press_cbks[2u](data, size);
  }
}

static void key4_press_callback(const void *data, size_t size) {
  if (btn_press_cbks[3u]) {
    btn_press_cbks[3u](data, size);
  }
}

static void key5_press_callback(const void *data, size_t size) {
  if (btn_press_cbks[4u]) {
    btn_press_cbks[4u](data, size);
  }
}

static void key6_press_callback(const void *data, size_t size) {
  if (btn_press_cbks[5u]) {
    btn_press_cbks[5u](data, size);
  }
}

static void key7_press_callback(const void *data, size_t size) {
  if (btn_press_cbks[6u]) {
    btn_press_cbks[6u](data, size);
  }
}

static void key8_press_callback(const void *data, size_t size) {
  if (btn_press_cbks[7u]) {
    btn_press_cbks[7u](data, size);
  }
}

static void key9_press_callback(const void *data, size_t size) {
  if (btn_press_cbks[8u]) {
    btn_press_cbks[8u](data, size);
  }
}

static void key1_release_callback(const void *data, size_t size) {
  if (btn_release_cbks[0u]) {
    btn_release_cbks[0u](data, size);
  }
}

static void key2_release_callback(const void *data, size_t size) {
  if (btn_release_cbks[1u]) {
    btn_release_cbks[1u](data, size);
  }
}

static void key3_release_callback(const void *data, size_t size) {
  if (btn_release_cbks[2u]) {
    btn_release_cbks[2u](data, size);
  }
}

static void key4_release_callback(const void *data, size_t size) {
  if (btn_release_cbks[3u]) {
    btn_release_cbks[3u](data, size);
  }
}

static void key5_release_callback(const void *data, size_t size) {
  if (btn_release_cbks[4u]) {
    btn_release_cbks[4u](data, size);
  }
}

static void key6_release_callback(const void *data, size_t size) {
  if (btn_release_cbks[5u]) {
    btn_release_cbks[5u](data, size);
  }
}

static void key7_release_callback(const void *data, size_t size) {
  if (btn_release_cbks[6u]) {
    btn_release_cbks[6u](data, size);
  }
}

static void key8_release_callback(const void *data, size_t size) {
  if (btn_release_cbks[7u]) {
    btn_release_cbks[7u](data, size);
  }
}

static void key9_release_callback(const void *data, size_t size) {
  if (btn_release_cbks[8u]) {
    btn_release_cbks[8u](data, size);
  }
}

static void pedal_press_callback(const void *data, size_t size) {
  if (pedal_press_cbk_ptr) {
    pedal_press_cbk_ptr(data, size);
  }
}

static void pedal_release_callback(const void *data, size_t size) {
  if (pedal_release_cbk_ptr) {
    pedal_release_cbk_ptr(data, size);
  }
}
