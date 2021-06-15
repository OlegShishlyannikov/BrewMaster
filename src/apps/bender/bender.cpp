#include <cstdarg>
#include <malloc.h>

#include "apps/bender/bender.hpp"
#include "apps/shell/vt100.hpp"
#include "lfs/lfs.h"
#include "sys/system.hpp"
#include "util/hash/sha256.hpp"
#include "json/json.h"

extern struct sys_impl_s &sys;
extern bool debug_log_enabled;
static bool task_running;
static xSemaphoreHandle state_sem;

static void beep_short();
static void beep_long();

static void relay_load_on(int32_t);
static void relay_load_off(int32_t);
static void relay_load_switch(int32_t);
static bool get_relay_state(int32_t);

static void up_key_press_cbk(const void *, size_t);
static void down_key_press_cbk(const void *, size_t);
static void set_key_press_cbk(const void *, size_t);
static void forward_key_press_cbk(const void *, size_t);
static void backward_key_press_cbk(const void *, size_t);
static void reset_key_press_cbk(const void *, size_t);
static void setup_key_press_cbk(const void *, size_t);

static void up_key_release_cbk(const void *, size_t);
static void down_key_release_cbk(const void *, size_t);
static void set_key_release_cbk(const void *, size_t);
static void forward_key_release_cbk(const void *, size_t);
static void backward_key_release_cbk(const void *, size_t);
static void reset_key_release_cbk(const void *, size_t);
static void pedal_press_cbk(const void *, size_t);
static void pedal_release_cbk(const void *, size_t);

static void forward_key_press();
static void forward_key_release();
static void backward_key_press();
static void backward_key_release();

static constexpr const int32_t max_angle = 179, min_angle = -179, static_return_error_abs_val = 3;
static int32_t current_angle = 0, absolute_angle = 0, set_angle = 0, relative_angle_correction = 0, target_angle = 0, return_angle = 0;

static void display_angles();
static void clear_whole_screen();
static void set_key_cbks();
static int32_t read_angle();

static enum bender_app_state_e : int32_t {
  BENDER_APP_STATE_UNKNOWN = -1,
  BENDER_APP_STATE_IDLE = 0,
  BENDER_APP_STATE_RETURNING_TO_ZERO,
  BENDER_APP_STATE_RETURNING_TO_ZERO_FROM_NEGATIVE,
  BENDER_APP_STATE_RETURNING_TO_ZERO_FROM_POSITIVE,
  BENDER_APP_STATE_BENDING,
  BENDER_APP_STATE_BENDING_TO_NEGATIVE,
  BENDER_APP_STATE_BENDING_TO_POSITIVE,
  BENDER_APP_STATE_BENDING_RETURNING_FROM_NEGATIVE,
  BENDER_APP_STATE_BENDING_RETURNING_FROM_POSITIVE
} bender_app_state = BENDER_APP_STATE_IDLE;
static bender_app_state_e bender_app_prev_state = BENDER_APP_STATE_UNKNOWN;

void bender_app_s::entry(void *args) {
  int32_t rc, zsp3806g2e_fd, ui_fd, gpio_fd;
  uint16_t gpio_val, angle_raw;

  task_running = true;
  beep_long();

  clear_whole_screen();
  display_angles();
  set_key_cbks();

  state_sem = xSemaphoreCreateMutex();

loop:
  while (task_running) {

    // Measure current angle first
    angle_raw = read_angle();
    absolute_angle = static_cast<int32_t>((static_cast<double>(angle_raw) / (static_cast<double>(UINT16_MAX) / 2.0f)) * 360.0f - 180.0f);

    if (absolute_angle - relative_angle_correction < min_angle) {
      current_angle = max_angle + (absolute_angle - relative_angle_correction - min_angle);
    } else if (absolute_angle - relative_angle_correction > max_angle) {
      current_angle = min_angle + (absolute_angle - relative_angle_correction - max_angle);
    } else {
      current_angle = absolute_angle - relative_angle_correction;
    }

    display_angles();

    switch (bender_app_state) {
    case BENDER_APP_STATE_RETURNING_TO_ZERO: {
      if (current_angle < 0) {

        xSemaphoreTake(state_sem, portMAX_DELAY);
        bender_app_prev_state = bender_app_state;
        bender_app_state = BENDER_APP_STATE_RETURNING_TO_ZERO_FROM_NEGATIVE;
        xSemaphoreGive(state_sem);
      } else if (current_angle > 0) {

        xSemaphoreTake(state_sem, portMAX_DELAY);
        bender_app_prev_state = bender_app_state;
        bender_app_state = BENDER_APP_STATE_RETURNING_TO_ZERO_FROM_POSITIVE;
        xSemaphoreGive(state_sem);
      } else {

        xSemaphoreTake(state_sem, portMAX_DELAY);
        bender_app_prev_state = bender_app_state;
        bender_app_state = BENDER_APP_STATE_IDLE;
        xSemaphoreGive(state_sem);
      }
    } break;

    case BENDER_APP_STATE_RETURNING_TO_ZERO_FROM_NEGATIVE: {
      // If current position less than zero -- emulate backward_key_press_cbk
      if (current_angle + static_return_error_abs_val) {
        backward_key_press();
      } else if (!(current_angle + static_return_error_abs_val)) {
        backward_key_release();

        xSemaphoreTake(state_sem, portMAX_DELAY);
        bender_app_prev_state = bender_app_state;
        bender_app_state = BENDER_APP_STATE_IDLE;
        xSemaphoreGive(state_sem);
      }
    } break;

    case BENDER_APP_STATE_RETURNING_TO_ZERO_FROM_POSITIVE: {
      // If current position less than zero -- emulate backward_key_press_cbk
      if (current_angle - static_return_error_abs_val) {
        forward_key_press();
      } else if (!(current_angle - static_return_error_abs_val)) {
        forward_key_release();

        xSemaphoreTake(state_sem, portMAX_DELAY);
        bender_app_prev_state = bender_app_state;
        bender_app_state = BENDER_APP_STATE_IDLE;
        xSemaphoreGive(state_sem);
      }
    } break;

    case BENDER_APP_STATE_BENDING: {
      // If current angle is positive
      if (set_angle > 0) {

        xSemaphoreTake(state_sem, portMAX_DELAY);
        bender_app_prev_state = bender_app_state;
        bender_app_state = BENDER_APP_STATE_BENDING_TO_POSITIVE;
        xSemaphoreGive(state_sem);

        // Target angle isn't in range
        if (target_angle > max_angle) {
          // Target angle exeeds max range -- target angle changes sign
          target_angle = min_angle + (target_angle - max_angle);
        }

        // If it is negative
      } else if (set_angle < 0) {

        xSemaphoreTake(state_sem, portMAX_DELAY);
        bender_app_prev_state = bender_app_state;
        bender_app_state = BENDER_APP_STATE_BENDING_TO_NEGATIVE;
        xSemaphoreGive(state_sem);

        if (target_angle < min_angle) {
          target_angle = max_angle + (target_angle - min_angle);
        }
      } else if (set_angle == 0) {
        xSemaphoreTake(state_sem, portMAX_DELAY);
        bender_app_prev_state = bender_app_state;
        bender_app_state = BENDER_APP_STATE_IDLE;
        xSemaphoreGive(state_sem);
      }
    } break;

    case BENDER_APP_STATE_BENDING_TO_NEGATIVE: {
      if (current_angle != target_angle) {
        forward_key_press();
      } else if (current_angle == target_angle) {
        forward_key_release();

        xSemaphoreTake(state_sem, portMAX_DELAY);
        bender_app_prev_state = bender_app_state;
        bender_app_state = BENDER_APP_STATE_BENDING_RETURNING_FROM_NEGATIVE;
        xSemaphoreGive(state_sem);
      }
    } break;

    case BENDER_APP_STATE_BENDING_TO_POSITIVE: {
      if (current_angle != target_angle) {
        backward_key_press();
      } else if (current_angle == target_angle) {
        backward_key_release();

        xSemaphoreTake(state_sem, portMAX_DELAY);
        bender_app_prev_state = bender_app_state;
        bender_app_state = BENDER_APP_STATE_BENDING_RETURNING_FROM_POSITIVE;
        xSemaphoreGive(state_sem);
      }
    } break;

    case BENDER_APP_STATE_BENDING_RETURNING_FROM_NEGATIVE: {
      // If current position less than zero -- emulate backward_key_press_cbk
      if ((current_angle + static_return_error_abs_val) != return_angle) {
        backward_key_press();
      } else if ((current_angle + static_return_error_abs_val) == return_angle) {
        backward_key_release();

        xSemaphoreTake(state_sem, portMAX_DELAY);
        bender_app_prev_state = bender_app_state;
        bender_app_state = BENDER_APP_STATE_IDLE;
        xSemaphoreGive(state_sem);
      }
    } break;

    case BENDER_APP_STATE_BENDING_RETURNING_FROM_POSITIVE: {
      // If current position less than zero -- emulate backward_key_press_cbk
      if ((current_angle - static_return_error_abs_val) != return_angle) {
        forward_key_press();
      } else if ((current_angle - static_return_error_abs_val) == return_angle) {
        forward_key_release();

        xSemaphoreTake(state_sem, portMAX_DELAY);
        bender_app_prev_state = bender_app_state;
        bender_app_state = BENDER_APP_STATE_IDLE;
        xSemaphoreGive(state_sem);
      }
    } break;

    case BENDER_APP_STATE_IDLE:
    default:
      break;
    }

    vTaskDelay(2u);
  }

exit:
  if ((rc = ::open(&sys, "rcc/rcc0", 3, 2u)) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  }

  if ((rc = ::ioctl(&sys, "rcc/rcc0", rcc_drv_ioctl_cmd_e::RCC_REBOOT, nullptr, 0u)) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  }

  if ((rc = ::close(&sys, "rcc/rcc0")) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  }

  vTaskDelete(nullptr);
}

int32_t bender_app_s::bender_dbg_printfmt(const char *fmt, ...) {
  int32_t rc, usart_fd, strlen;
  static char *temp;
  const struct drv_model_cmn_s *usart;

  std::va_list arg;
  va_start(arg, fmt);
  size_t bufsz = std::vsnprintf(nullptr, 0u, fmt, arg);
  temp = static_cast<char *>(std::calloc(bufsz, sizeof(char)));
  strlen = std::vsprintf(temp, fmt, arg);
  va_end(arg);

  if (!debug_log_enabled) {
    goto exit;
  }

  if (!(usart = sys.drv("usart"))) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (strlen) {
    if ((usart_fd = ::open(usart, "usart1", 3, 3u)) < 0) {
      bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, "[bender] : ", std::strlen("[bender] : "))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }
      bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, temp, std::strlen(temp))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::close(usart, usart_fd)) < 0) {
      bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

exit:
  std::free(temp);
  return strlen;
error:

  std::free(temp);
  return -1;
}

int32_t bender_app_s::dbg_printfmt(const char *fmt, ...) {
  int32_t rc, usart_fd, strlen;
  static char *temp;
  const struct drv_model_cmn_s *usart;

  std::va_list arg;
  va_start(arg, fmt);
  size_t bufsz = std::vsnprintf(nullptr, 0u, fmt, arg);
  temp = static_cast<char *>(std::calloc(bufsz, sizeof(char)));
  strlen = std::vsprintf(temp, fmt, arg);
  va_end(arg);

  if (!(usart = sys.drv("usart"))) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (strlen) {
    if ((usart_fd = ::open(usart, "usart1", 3, 3u)) < 0) {
      bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, temp, std::strlen(temp))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::close(usart, usart_fd)) < 0) {
      bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  std::free(temp);
  return strlen;
error:

  std::free(temp);
  return -1;
}

int32_t bender_app_s::bender_dbg_nprint(void *ptr, size_t n) {
  int32_t rc, usart_fd;
  const struct drv_model_cmn_s *usart = sys.drv("usart");

  if ((usart_fd = ::open(usart, "usart1", 3, 3u)) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::write(usart, usart_fd, "[bender] : ", std::strlen("[bender] : "))) < 0) {
    if ((rc = ::close(usart, usart_fd)) < 0) {
      bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::write(usart, usart_fd, ptr, n)) < 0) {
    if ((rc = ::close(usart, usart_fd)) < 0) {
      bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(usart, usart_fd)) < 0) {
    if ((rc = ::close(usart, usart_fd)) < 0) {
      bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return n;
error:
  return -1;
}

static void beep_short() {
  int32_t rc, fd;
  struct buzzer_beep_req_s req {
    .delay_fn = vTaskDelay, .up = 1u, .down = 0u, .n = 1u
  };

  if ((fd = ::open(&sys, "buzzer/buzzer0", 3, 3u)) < 0) {
    goto error;
  }

  if ((rc = ::ioctl(&sys, "buzzer/buzzer0", buzzer_drv_ioctl_cmd_e::BUZZER_BEEP, &req, sizeof(req))) < 0) {
    goto error;
  }

  if ((rc = ::close(&sys, "buzzer/buzzer0")) < 0) {
    goto error;
  }

exit:
  return;
error:
  return;
}

static void beep_long() {
  int32_t rc, fd;
  struct buzzer_beep_req_s req {
    .delay_fn = vTaskDelay, .up = 50u, .down = 0u, .n = 1u
  };

  if ((fd = ::open(&sys, "buzzer/buzzer0", 3, 3u)) < 0) {
    goto error;
  }

  if ((rc = ::ioctl(&sys, "buzzer/buzzer0", buzzer_drv_ioctl_cmd_e::BUZZER_BEEP, &req, sizeof(req))) < 0) {
    goto error;
  }

  if ((rc = ::close(&sys, "buzzer/buzzer0")) < 0) {
    goto error;
  }

exit:
  return;
error:
  return;
}

static void relay_load_on(int32_t n) {
  int32_t rc, fd;
  char path[32u];
  enum relay_load_state_e state;
  std::snprintf(path, sizeof(path), "relay_load/relayload%i", n);
  struct relay_load_get_state_req_s req {
    .state = &state
  };

  if ((fd = ::open(&sys, path, 3, 3u)) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, path, relay_load_ioctl_cmd_e::RELAY_LOAD_ON, nullptr, 0u)) < 0) {
    if ((rc = ::close(&sys, path)) < 0) {
      bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, path)) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return;
error:
  return;
}

static void relay_load_off(int32_t n) {
  int32_t rc, fd;
  char path[32u];
  enum relay_load_state_e state;
  std::snprintf(path, sizeof(path), "relay_load/relayload%i", n);
  struct relay_load_get_state_req_s req {
    .state = &state
  };

  if ((fd = ::open(&sys, path, 3, 3u)) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, path, relay_load_ioctl_cmd_e::RELAY_LOAD_OFF, nullptr, 0u)) < 0) {
    if ((rc = ::close(&sys, path)) < 0) {
      bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, path)) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return;
error:
  return;
}

static void relay_load_switch(int32_t n) {
  int32_t rc, fd;
  char path[32u];
  enum relay_load_state_e state;
  std::snprintf(path, sizeof(path), "relay_load/relayload%i", n);
  struct relay_load_get_state_req_s req {
    .state = &state
  };

  if ((fd = ::open(&sys, path, 3, 3u)) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, path, relay_load_ioctl_cmd_e::RELAY_LOAD_GET_STATE, &req, sizeof(req))) < 0) {
    if ((rc = ::close(&sys, path)) < 0) {
      bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (state == relay_load_state_e::RELAY_LOAD_STATE_ON) {
    relay_load_off(n);
  } else if (state == relay_load_state_e::RELAY_LOAD_STATE_OFF) {
    relay_load_on(n);
  }

  if ((rc = ::close(&sys, path)) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

error:
  return;
}

static bool get_relay_state(int32_t n) {
  int32_t rc, fd;
  char path[32u];
  enum relay_load_state_e state;
  std::snprintf(path, sizeof(path), "relay_load/relayload%i", n);
  struct relay_load_get_state_req_s req {
    .state = &state
  };

  if ((fd = ::open(&sys, path, 3, 3u)) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, path, relay_load_ioctl_cmd_e::RELAY_LOAD_GET_STATE, &req, sizeof(req))) < 0) {
    if ((rc = ::close(&sys, path)) < 0) {
      bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, path)) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (state == relay_load_state_e::RELAY_LOAD_STATE_ON) {
    return true;
  } else if (state == relay_load_state_e::RELAY_LOAD_STATE_OFF) {
    return false;
  }

error:
  return false;
}

static void up_key_press_cbk(const void *data, size_t size) {
  if (set_angle < max_angle) {
    set_angle++;
    beep_short();
  }

  display_angles();
}

static void up_key_release_cbk(const void *data, size_t size) {}

static void down_key_press_cbk(const void *data, size_t size) {
  if (set_angle > min_angle) {
    set_angle--;
    beep_short();
  }

  display_angles();
}

static void down_key_release_cbk(const void *data, size_t size) {}

static void set_key_press_cbk(const void *data, size_t size) {
  if (bender_app_state == BENDER_APP_STATE_IDLE) {
    target_angle = current_angle + set_angle;

    // If the prevous state hasn't been returning to origin angle from bending
    if (bender_app_prev_state != BENDER_APP_STATE_BENDING_RETURNING_FROM_NEGATIVE && bender_app_prev_state != BENDER_APP_STATE_BENDING_RETURNING_FROM_POSITIVE) {
      return_angle = current_angle;
    }

    xSemaphoreTake(state_sem, portMAX_DELAY);
    bender_app_state = BENDER_APP_STATE_BENDING;
    xSemaphoreGive(state_sem);
    beep_short();
  }
}

static void set_key_release_cbk(const void *data, size_t size) { beep_short(); }

static void forward_key_press() {
  switch (bender_app_state) {

    // If it has been invoked from state machine -- do nothing to don't bother the bending process
  case BENDER_APP_STATE_RETURNING_TO_ZERO ... BENDER_APP_STATE_BENDING_RETURNING_FROM_POSITIVE: {
    relay_load_on(0u);
  } break;

    // If it has been onvoked manually from IDLE state
  case BENDER_APP_STATE_IDLE:
  default:
    break;
  }
}

static void forward_key_press_cbk(const void *data, size_t size) {
  switch (bender_app_state) {

    // If it has been onvoked manually from IDLE state
  case BENDER_APP_STATE_IDLE: {
    if (!get_relay_state(0u) && !get_relay_state(1u)) {
      relay_load_on(0u);
      bender_app_prev_state = BENDER_APP_STATE_IDLE;
      beep_short();
    }
  } break;

    // If it has been invoked from state machine -- do nothing to don't bother the bending process
  case BENDER_APP_STATE_RETURNING_TO_ZERO ... BENDER_APP_STATE_BENDING_RETURNING_FROM_POSITIVE:
  default:
    break;
  }
}

static void forward_key_release() {
  switch (bender_app_state) {

    // If it has been invoked from state machine -- do nothing to don't bother the bending process
  case BENDER_APP_STATE_RETURNING_TO_ZERO ... BENDER_APP_STATE_BENDING_RETURNING_FROM_POSITIVE: {
    relay_load_off(0u);
    vTaskDelay(10);
    relay_load_on(1u);
    vTaskDelay(42);
    relay_load_off(1u);
  } break;

    // If it has been onvoked manually from IDLE state
  case BENDER_APP_STATE_IDLE:
  default:
    break;
  }
}

static void forward_key_release_cbk(const void *data, size_t size) {
  switch (bender_app_state) {

    // If it has been onvoked manually from IDLE state
  case BENDER_APP_STATE_IDLE: {
    relay_load_off(0u);
    vTaskDelay(10);
    relay_load_on(1u);
    vTaskDelay(42);
    relay_load_off(1u);
  } break;

    // If it has been invoked from state machine -- do nothing to don't bother the bending process
  case BENDER_APP_STATE_RETURNING_TO_ZERO ... BENDER_APP_STATE_BENDING_RETURNING_FROM_POSITIVE:
  default:
    break;
  }
}

static void backward_key_press() {
  switch (bender_app_state) {

    // If it has been invoked from state machine -- do nothing to don't bother the bending process
  case BENDER_APP_STATE_RETURNING_TO_ZERO ... BENDER_APP_STATE_BENDING_RETURNING_FROM_POSITIVE: {
    relay_load_on(1u);
  } break;

    // If it has been onvoked manually from IDLE state
  case BENDER_APP_STATE_IDLE:
  default:
    break;
  }
}

static void backward_key_press_cbk(const void *data, size_t size) {
  switch (bender_app_state) {

    // If it has been onvoked manually from IDLE state
  case BENDER_APP_STATE_IDLE: {
    if (!get_relay_state(0u) && !get_relay_state(1u)) {
      relay_load_on(1u);
      bender_app_prev_state = BENDER_APP_STATE_IDLE;
      beep_short();
    }
  } break;

    // If it has been invoked from state machine -- do nothing to don't bother the bending process
  case BENDER_APP_STATE_RETURNING_TO_ZERO ... BENDER_APP_STATE_BENDING_RETURNING_FROM_POSITIVE:
  default:
    break;
  }
}

static void backward_key_release() {
  switch (bender_app_state) {

    // If it has been invoked from state machine -- do nothing to don't bother the bending process
  case BENDER_APP_STATE_RETURNING_TO_ZERO ... BENDER_APP_STATE_BENDING_RETURNING_FROM_POSITIVE: {
    relay_load_off(1u);
    vTaskDelay(10);
    relay_load_on(0u);
    vTaskDelay(50);
    relay_load_off(0u);
  } break;

    // If it has been onvoked manually from IDLE state
  case BENDER_APP_STATE_IDLE: {
  } break;
  default:
    break;
  }
}

static void backward_key_release_cbk(const void *data, size_t size) {
  switch (bender_app_state) {

    // If it has been onvoked manually from IDLE state
  case BENDER_APP_STATE_IDLE: {
    relay_load_off(1u);
    vTaskDelay(10);
    relay_load_on(0u);
    vTaskDelay(50);
    relay_load_off(0u);
  } break;

    // If it has been invoked from state machine -- do nothing to don't bother the bending process
  case BENDER_APP_STATE_RETURNING_TO_ZERO ... BENDER_APP_STATE_BENDING_RETURNING_FROM_POSITIVE:
  default:
    break;
  }
}

static void reset_key_press_cbk(const void *data, size_t size) {
  if (bender_app_state == BENDER_APP_STATE_IDLE) {
    xSemaphoreTake(state_sem, portMAX_DELAY);
    bender_app_state = BENDER_APP_STATE_RETURNING_TO_ZERO;
    xSemaphoreGive(state_sem);
    beep_short();
  }
}

static void reset_key_release_cbk(const void *data, size_t size) { beep_short(); }

static void setup_key_press_cbk(const void *, size_t) {
  switch (bender_app_state) {
  case BENDER_APP_STATE_IDLE: {
    relative_angle_correction = absolute_angle;
    beep_short();
  } break;

  case BENDER_APP_STATE_RETURNING_TO_ZERO ... BENDER_APP_STATE_BENDING_RETURNING_FROM_POSITIVE:
  default:
    break;
  }
}

static void pedal_press_cbk(const void *data, size_t size) { set_key_press_cbk(data, size); }
static void pedal_release_cbk(const void *data, size_t size) { set_key_release_cbk(data, size); }

static void clear_whole_screen() {
  int32_t ui_fd, rc;

  if ((ui_fd = ::open(&sys, "ui/ui", 3, 3u)) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((rc = ::ioctl(&sys, "ui/ui", ui_ioctl_cmd_e::UI_DISP_LEFT_DOWN_RESET, nullptr, 0u)) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((rc = ::ioctl(&sys, "ui/ui", ui_ioctl_cmd_e::UI_DISP_LEFT_UP_RESET, nullptr, 0u)) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((rc = ::ioctl(&sys, "ui/ui", ui_ioctl_cmd_e::UI_DISP_RIGHT_DOWN_RESET, nullptr, 0u)) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((rc = ::ioctl(&sys, "ui/ui", ui_ioctl_cmd_e::UI_DISP_RIGHT_UP_RESET, nullptr, 0u)) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((ui_fd = ::close(&sys, "ui/ui")) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

exit:
  return;
}

static void display_angles() {
  int32_t rc, ui_fd;
  char current_angle_buffer[4u];
  char set_angle_buffer[4u];
  struct ui_4d_disp_set_num_req_s ui_4d_req;

  if ((ui_fd = ::open(&sys, "ui/ui", 3, 3u)) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  std::sprintf(current_angle_buffer, "%3d", std::abs(current_angle));
  std::memcpy(ui_4d_req.num_str, current_angle_buffer, sizeof(current_angle_buffer));
  ui_4d_req.is_negative = current_angle < 0;
  if ((rc = ::ioctl(&sys, "ui/ui", ui_ioctl_cmd_e::UI_DISP_RIGHT_UP_SET_NUM, &ui_4d_req, sizeof(ui_4d_req))) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  std::sprintf(set_angle_buffer, "%3d", std::abs(set_angle));
  std::memcpy(ui_4d_req.num_str, set_angle_buffer, sizeof(set_angle_buffer));
  ui_4d_req.is_negative = set_angle < 0;
  if ((rc = ::ioctl(&sys, "ui/ui", ui_ioctl_cmd_e::UI_DISP_RIGHT_DOWN_SET_NUM, &ui_4d_req, sizeof(ui_4d_req))) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((ui_fd = ::close(&sys, "ui/ui")) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

exit:
  return;
}

static void set_key_cbks() {
  int32_t rc, ui_fd;
  struct ui_key_cbk_req_s ui_req;
  struct ui_pedal_cbk_req_s ui_pedal_req;

  if ((ui_fd = ::open(&sys, "ui/ui", 3, 3u)) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  ui_req = {.key_no = ui_key_number_e::UI_KEY8, .type = ui_key_cbk_type_e::UI_KEY_CBK_PRESS, .cbk = forward_key_press_cbk};
  if ((rc = ::ioctl(&sys, "ui/ui", ui_ioctl_cmd_e::UI_KEY_CBK_SET, &ui_req, sizeof(ui_req))) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  ui_req = {.key_no = ui_key_number_e::UI_KEY8, .type = ui_key_cbk_type_e::UI_KEY_CBK_RELEASE, .cbk = forward_key_release_cbk};
  if ((rc = ::ioctl(&sys, "ui/ui", ui_ioctl_cmd_e::UI_KEY_CBK_SET, &ui_req, sizeof(ui_req))) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  ui_req = {.key_no = ui_key_number_e::UI_KEY9, .type = ui_key_cbk_type_e::UI_KEY_CBK_PRESS, .cbk = backward_key_press_cbk};
  if ((rc = ::ioctl(&sys, "ui/ui", ui_ioctl_cmd_e::UI_KEY_CBK_SET, &ui_req, sizeof(ui_req))) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  ui_req = {.key_no = ui_key_number_e::UI_KEY9, .type = ui_key_cbk_type_e::UI_KEY_CBK_RELEASE, .cbk = backward_key_release_cbk};
  if ((rc = ::ioctl(&sys, "ui/ui", ui_ioctl_cmd_e::UI_KEY_CBK_SET, &ui_req, sizeof(ui_req))) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  ui_req = {.key_no = ui_key_number_e::UI_KEY3, .type = ui_key_cbk_type_e::UI_KEY_CBK_PRESS, .cbk = up_key_press_cbk};
  if ((rc = ::ioctl(&sys, "ui/ui", ui_ioctl_cmd_e::UI_KEY_CBK_SET, &ui_req, sizeof(ui_req))) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  ui_req = {.key_no = ui_key_number_e::UI_KEY4, .type = ui_key_cbk_type_e::UI_KEY_CBK_PRESS, .cbk = down_key_press_cbk};
  if ((rc = ::ioctl(&sys, "ui/ui", ui_ioctl_cmd_e::UI_KEY_CBK_SET, &ui_req, sizeof(ui_req))) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  ui_req = {.key_no = ui_key_number_e::UI_KEY1, .type = ui_key_cbk_type_e::UI_KEY_CBK_PRESS, .cbk = reset_key_press_cbk};
  if ((rc = ::ioctl(&sys, "ui/ui", ui_ioctl_cmd_e::UI_KEY_CBK_SET, &ui_req, sizeof(ui_req))) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  ui_req = {.key_no = ui_key_number_e::UI_KEY2, .type = ui_key_cbk_type_e::UI_KEY_CBK_PRESS, .cbk = set_key_press_cbk};
  if ((rc = ::ioctl(&sys, "ui/ui", ui_ioctl_cmd_e::UI_KEY_CBK_SET, &ui_req, sizeof(ui_req))) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  ui_req = {.key_no = ui_key_number_e::UI_KEY6, .type = ui_key_cbk_type_e::UI_KEY_CBK_PRESS, .cbk = setup_key_press_cbk};
  if ((rc = ::ioctl(&sys, "ui/ui", ui_ioctl_cmd_e::UI_KEY_CBK_SET, &ui_req, sizeof(ui_req))) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  ui_pedal_req = {.type = ui_key_cbk_type_e::UI_KEY_CBK_PRESS, .cbk = pedal_press_cbk};
  if ((rc = ::ioctl(&sys, "ui/ui", ui_ioctl_cmd_e::UI_PEDAL_CBK_SET, &ui_pedal_req, sizeof(ui_pedal_req))) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  ui_pedal_req = {.type = ui_key_cbk_type_e::UI_KEY_CBK_RELEASE, .cbk = pedal_release_cbk};
  if ((rc = ::ioctl(&sys, "ui/ui", ui_ioctl_cmd_e::UI_PEDAL_CBK_SET, &ui_pedal_req, sizeof(ui_pedal_req))) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

  if ((ui_fd = ::close(&sys, "ui/ui")) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto exit;
  }

exit:
  return;
}

static int32_t read_angle() {
  int32_t zsp3806g2e_fd, rc, angle;
  if ((zsp3806g2e_fd = ::open(&sys, "zsp3806g2e/zsp3806g2e", 3, 3u)) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::read(&sys, "zsp3806g2e/zsp3806g2e", &angle, sizeof(angle))) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, "zsp3806g2e/zsp3806g2e")) < 0) {
    bender_app_s::bender_dbg_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return angle;
error:
  return min_angle - 1;
}
