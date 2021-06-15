#include <cstdarg>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "arch/mcu_tags.hpp"
#include "boards/board_tags.hpp"
#include "drivers/io/gpio_ioctl.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "drivers/io/timer_ioctl.hpp"
#include "drivers/timer_driver.hpp"
#include "fio/unistd.hpp"
#include "sys/events_worker.hpp"

static const struct drv_model_cmn_s *drv_ptr;
extern bool debug_log_enabled;
extern xQueueHandle events_worker_queue;

/* TIM driver lock */
static xSemaphoreHandle TIM1_lock;

static constexpr const uint16_t timer_cnt_modes[] = {TIM_CounterMode_Up, TIM_CounterMode_Down, TIM_CounterMode_CenterAligned1, TIM_CounterMode_CenterAligned2, TIM_CounterMode_CenterAligned3};
static constexpr const uint16_t timer_clk_divs[] = {TIM_CKD_DIV1, TIM_CKD_DIV2, TIM_CKD_DIV4};

static int32_t timer_printf(const char *fmt, ...);
static int32_t timer_drv_TIM1_open(int32_t, mode_t);
static int32_t timer_drv_TIM1_ioctl(uint64_t, const void *, size_t);
static int32_t timer_drv_TIM1_read(void *const, size_t);
static int32_t timer_drv_TIM1_write(const void *, size_t);
static int32_t timer_drv_TIM1_close();

static void TIM1_irq_handler();
static void (*TIM1_update_callback)(const void *, size_t);

/* GPIOA helper functions */
static int32_t TIM1_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreTakeRecursive(TIM1_lock, portIO_MAX_DELAY)) != pdPASS) {
    // errno = ???
    timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t TIM1_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreGiveRecursive(TIM1_lock)) != pdPASS) {
    // errno = ???
    timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

/* TIMER driver operations secification */
struct drv_ops_s timer_drv_ops {
  .init = timer_drv_init, .exit = timer_drv_exit
};

/* TIMER driver file operations secification */
struct file_ops_s timer_drv_TIM1_fops {
  .flock = TIM1_flock, .funlock = TIM1_funlock, .open = timer_drv_TIM1_open, .ioctl = timer_drv_TIM1_ioctl, .read = timer_drv_TIM1_read, .write = timer_drv_TIM1_write, .close = timer_drv_TIM1_close
};

void timer_drv_init(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *rcc;
  struct rcc_drv_req_s rcc_req;
  int32_t rcc_fd, rc;

  /* Get rcc driver from dependencies */
  if (!(rcc = drv->dep("rcc"))) {
    // errno = ENOENT
    timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Open RCC device */
  if ((rcc_fd = ::open(rcc, "rcc0", 3, 2u)) < 0) {
    // errno = ???
    timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Enable GPIOA clocking */
  rcc_req = {rcc_bus_e::APB2, rcc_apb2_periph_e::RCC_TIM1};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_ENABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Close rcc file */
  if ((rc = ::close(rcc, rcc_fd)) < 0) {
    timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  timer_drv_TIM1_fops.owner = drv;

  /* Init locks */
  TIM1_lock = xSemaphoreCreateRecursiveMutex();

  /* Register char device for each GPIO port */
  drv->register_chardev("TIM1", &timer_drv_TIM1_fops);

  /* Initialize GPIO driver pointer */
  drv_ptr = drv;
  return;

  /* Something went wrong -- reset drv_ptr and exit */
error:
  timer_drv_exit(drv);
  return;
}

void timer_drv_exit(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *rcc;
  struct rcc_drv_req_s rcc_req;
  int32_t rcc_fd, rc;

  /* Get rcc driver from dependencies */
  if (!(rcc = drv->dep("rcc"))) {
    // errno = ENOENT
    timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Open RCC device */
  if ((rcc_fd = ::open(rcc, "rcc0", 3, 2u)) < 0) {
    // errno = ???
    timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Disable GPIOA clocking */
  rcc_req = {rcc_bus_e::APB2, rcc_apb2_periph_e::RCC_TIM1};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_DISABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Close rcc file */
  if ((rc = ::close(rcc, rcc_fd)) < 0) {
    timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Unregister char devices */
  drv->unregister_chardev("TIM1");

  /* Remove locks */
  vSemaphoreDelete(TIM1_lock);

  /* Reset driver ptr */
  drv_ptr = nullptr;

error:
  return;
}

static int32_t timer_drv_TIM1_open(int32_t oflags, mode_t mode) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t timer_drv_TIM1_ioctl(uint64_t req, const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (req) {
  case TIMER_INIT: {
    const struct timer_setup_req_s *req = reinterpret_cast<const struct timer_setup_req_s *>(buf);
    TIM_TimeBaseInitTypeDef timer;
    NVIC_InitTypeDef nvic;
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_CounterMode = timer_cnt_modes[req->cnt_mode];
    timer.TIM_Prescaler = req->psc;
    timer.TIM_Period = req->period;
    timer.TIM_ClockDivision = timer_clk_divs[req->clk_div];
    timer.TIM_RepetitionCounter = req->rep_cnt;
    TIM_TimeBaseInit(TIM1, &timer);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    g_pfnVectorsRam[TIM1_UP_TIM10_IRQHandler_Vector] = TIM1_irq_handler;

    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, req->irq_priority);
    NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  } break;

  case TIMER_DEINIT: {
    TIM_Cmd(TIM1, DISABLE);
    TIM_DeInit(TIM1);
    TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
    NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);

    g_pfnVectorsRam[TIM1_UP_TIM10_IRQHandler_Vector] = nullptr;

  } break;

  case TIMER_START: {
    TIM_Cmd(TIM1, ENABLE);
  } break;

  case TIMER_STOP: {
    TIM_Cmd(TIM1, DISABLE);
  } break;

  case TIMER_CBK_SET: {
    const struct timer_cbk_req_s *req = reinterpret_cast<const struct timer_cbk_req_s *>(buf);
    TIM1_update_callback = req->callback;
  } break;

  case TIMER_CBK_RESET: {
    const struct timer_cbk_req_s *req = reinterpret_cast<const struct timer_cbk_req_s *>(buf);
    TIM1_update_callback = nullptr;
  } break;

  default:
    break;
  }

  return 0;
error:
  return -1;
}

static int32_t timer_drv_TIM1_read(void *const, size_t) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t timer_drv_TIM1_write(const void *, size_t) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t timer_drv_TIM1_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t timer_printf(const char *fmt, ...) {
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
    timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (strlen) {
    if ((usart_fd = ::open(usart, "usart1", 3, 3u)) < 0) {
      timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, "[timer] : ", std::strlen("[timer] : "))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }
      timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, temp, std::strlen(temp))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::close(usart, usart_fd)) < 0) {
      timer_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
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

static void TIM1_irq_handler() {
  BaseType_t rc;
  portBASE_TYPE hp_task_woken;

  if (TIM_GetITStatus(TIM1, TIM_IT_Update)) {
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    if (TIM1_update_callback) {
      typename events_worker_s::event_s event{.handler = TIM1_update_callback, .data = nullptr, .size = 0u};

      if ((rc = xQueueSendToBackFromISR(events_worker_queue, &event, &hp_task_woken)) != pdPASS) {
        goto exit;
      }

      if (hp_task_woken == pdTRUE) {
        taskYIELD();
      }
    }
  }

exit:
  return;
}
