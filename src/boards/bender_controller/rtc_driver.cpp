#include <cstdarg>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "arch/mcu_tags.hpp"
#include "boards/board_tags.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "drivers/io/rtc_ioctl.hpp"
#include "drivers/io/usart_ioctl.hpp"
#include "drivers/rtc_driver.hpp"
#include "fio/unistd.hpp"
#include "sys/events_worker.hpp"

static const struct drv_model_cmn_s *drv_ptr;

/* RTC lock */
static xSemaphoreHandle rtc_lock;
extern xQueueHandle events_worker_queue;

static constexpr const char *console_devstr = "usart1";

static bool rtc_gettime(struct rtc_s *const);  /* Get time */
static bool rtc_settime(const struct rtc_s *); /* Set time */
extern bool debug_log_enabled;

static constexpr uint32_t first_year = 0u;
static constexpr uint32_t first_day = 0u;
static constexpr const uint8_t days_in_month[] = {31u, 29u, 31u, 30u, 31u, 30u, 31u, 31u, 30u, 31u, 30u, 31u};

static void (*RTC_second_callback)(const void *, size_t){nullptr};
static void (*RTC_minute_callback)(const void *, size_t){nullptr};
static void (*RTC_hour_callback)(const void *, size_t){nullptr};
static void (*RTC_day_callback)(const void *, size_t){nullptr};

static void RTC_irq_handler();

/* BSP dependent file operations functions -- forward reference */
static int32_t rtc_drv_open(int32_t, mode_t);
static int32_t rtc_drv_ioctl(uint64_t, const void *, size_t);
static int32_t rtc_drv_read(void *const, size_t);
static int32_t rtc_drv_write(const void *, size_t);
static int32_t rtc_drv_close();

/* Helper functions */
static int32_t rtc_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  BaseType_t rc;
  if (!(rc = xSemaphoreTakeRecursive(rtc_lock, portIO_MAX_DELAY))) {
    // errno = ???
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t rtc_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  BaseType_t rc;
  if (!(rc = xSemaphoreGiveRecursive(rtc_lock))) {
    // errno = ???
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t rtc_drv_printf(const char *fmt, ...);

struct drv_ops_s rtc_drv_ops {
  .init = rtc_drv_init, .exit = rtc_drv_exit
};

/* Driver file operations specification */
struct file_ops_s rtc_drv_fops {
  .flock = rtc_flock, .funlock = rtc_funlock, .open = rtc_drv_open, .ioctl = rtc_drv_ioctl, .read = rtc_drv_read, .write = rtc_drv_write, .close = rtc_drv_close
};

void rtc_drv_init(const struct drv_model_cmn_s *drv) {
  volatile uint16_t i;
  int32_t rc, rcc_fd;
  const struct drv_model_cmn_s *rcc;
  struct rcc_drv_req_s rcc_req;

  /* Get rcc driver from dependencies */
  if (!(rcc = drv->dep("rcc"))) {
    // errno = ENOENT
    rtc_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Open RCC device */
  if ((rcc_fd = ::open(rcc, "rcc0", 3, 2u)) < 0) {
    // errno = ???
    rtc_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Enable BKP clocking */
  rcc_req = {rcc_bus_e::APB1, rcc_apb1_periph_e::RCC_BKP};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_ENABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    rtc_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Enable PWR clocking */
  rcc_req = {rcc_bus_e::APB1, rcc_apb1_periph_e::RCC_PWR};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_ENABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    rtc_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Close rcc file */
  if ((rc = ::close(rcc, rcc_fd)) < 0) {
    rtc_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Allow access to BKP Domain */
  PWR_BackupAccessCmd(ENABLE);

  /* Reset Backup Domain */
  BKP_DeInit();

  /* Disable LSE */
  RCC_LSEConfig(RCC_LSE_OFF);
  RCC_LSICmd(ENABLE);

  /* Wait till LSI is ready */
  while (!RCC_GetFlagStatus(RCC_FLAG_LSIRDY))
    ;

  /* Select LSI as RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

  /* Enable RTC Clock */
  RCC_RTCCLKCmd(ENABLE);

  /* Wait for RTC registers synchronization */
  RTC_WaitForSynchro();

  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();

  /* Set RTC prescaler: set RTC period to 1sec */
  RTC_SetPrescaler(39999u); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */

  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();

  /* Set initial value */
  RTC_SetCounter(0u);

  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();
  BKP_WriteBackupRegister(BKP_DR1, 0xa5a5);

  /* Lock access to BKP Domain */
  PWR_BackupAccessCmd(DISABLE);

  rtc_drv_fops.owner = drv;
  rtc_lock = xSemaphoreCreateRecursiveMutex();
  drv->register_chardev("rtc0", &rtc_drv_fops);

  drv_ptr = drv;
  return;

error:
  rtc_drv_exit(drv);
  return;
}

void rtc_drv_exit(const struct drv_model_cmn_s *drv) {
  int32_t rc, rcc_fd;
  const struct drv_model_cmn_s *rcc;
  struct rcc_drv_req_s rcc_req;

  BKP_DeInit();
  PWR_DeInit();
  RCC_LSEConfig(RCC_LSE_OFF);
  RCC_LSICmd(DISABLE);
  RCC_RTCCLKCmd(DISABLE);

  /* Get rcc driver from dependencies */
  if (!(rcc = drv->dep("rcc"))) {
    // errno = ENOENT
    rtc_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Open RCC device */
  if ((rcc_fd = ::open(rcc, "rcc0", 3, 2u)) < 0) {
    // errno = ???
    rtc_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Enable BKP clocking */
  rcc_req = {rcc_bus_e::APB1, rcc_apb1_periph_e::RCC_BKP};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_DISABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    rtc_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Enable PWR clocking */
  rcc_req = {rcc_bus_e::APB1, rcc_apb1_periph_e::RCC_PWR};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_DISABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    rtc_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Close rcc file */
  if ((rc = ::close(rcc, rcc_fd)) < 0) {
    rtc_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  drv->unregister_chardev("rcc0");
  vSemaphoreDelete(rtc_lock);
  drv_ptr = nullptr;
  return;

error:
  vSemaphoreDelete(rtc_lock);
  drv_ptr = nullptr;
  return;
}

static int32_t rtc_drv_printf(const char *fmt, ...) {
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
    rtc_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (strlen) {
    if ((usart_fd = ::open(usart, console_devstr, 3, 3u)) < 0) {
      rtc_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, "[rtc] : ", std::strlen("[rtc] : "))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        rtc_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }
      rtc_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, temp, std::strlen(temp))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        rtc_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      rtc_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::close(usart, usart_fd)) < 0) {
      rtc_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
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

static int32_t rtc_drv_open(int32_t oflags, mode_t mode) {
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t rtc_drv_ioctl(uint64_t req, const void *buf, size_t size) {
  int32_t rc;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  switch (req) {
  case RTC_ON_SECOND_SET: {
    const struct rtc_callback_req_s *req = static_cast<const struct rtc_callback_req_s *>(buf);
    RTC_second_callback = req->callback;
    rc = 0;
  } break;

  case RTC_ON_MINUTE_SET: {
    const struct rtc_callback_req_s *req = static_cast<const struct rtc_callback_req_s *>(buf);
    RTC_minute_callback = req->callback;
    rc = 0;
  } break;

  case RTC_ON_HOUR_SET: {
    const struct rtc_callback_req_s *req = static_cast<const struct rtc_callback_req_s *>(buf);
    RTC_hour_callback = req->callback;
    rc = 0;
  } break;

  case RTC_ON_DAY_SET: {
    const struct rtc_callback_req_s *req = static_cast<const struct rtc_callback_req_s *>(buf);
    RTC_day_callback = req->callback;
    rc = 0;
  } break;

  case RTC_ON_SECOND_RESET: {
    RTC_second_callback = nullptr;
    rc = 0;
  } break;

  case RTC_ON_MINUTE_RESET: {
    RTC_minute_callback = nullptr;
    rc = 0;
  } break;

  case RTC_ON_HOUR_RESET: {
    RTC_hour_callback = nullptr;
    rc = 0;
  } break;

  case RTC_ON_DAY_RESET: {
    RTC_day_callback = nullptr;
    rc = 0;
  } break;

  case RTC_IRQ_ENABLE: {
    const struct rtc_irq_req_s *req = static_cast<const struct rtc_irq_req_s *>(buf);
    g_pfnVectorsRam[RTC_IRQHandler_Vector] = RTC_irq_handler;
    RTC_ITConfig(RTC_IT_SEC, ENABLE);
    NVIC_SetPriority(RTC_IRQn, req->priority);
    NVIC_EnableIRQ(RTC_IRQn);
    rc = 0;
  } break;

  case RTC_IRQ_DISABLE: {
    RTC_ITConfig(RTC_IT_SEC, DISABLE);
    NVIC_DisableIRQ(RTC_IRQn);
    g_pfnVectorsRam[RTC_IRQHandler_Vector] = nullptr;
    rc = 0;
  } break;

  case RTC_SET_TIME: {
    const struct rtc_set_time_req_s *req = static_cast<const struct rtc_set_time_req_s *>(buf);
    if (!rtc_settime(req->rtc)) {
      rtc_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    rc = 0;
  } break;

  case RTC_GET_TIME: {
    const struct rtc_get_time_req_s *req = static_cast<const struct rtc_get_time_req_s *>(buf);
    if (!rtc_gettime(req->rtc)) {
      rtc_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    rc = 0;
  } break;
  default:
    rc = -1;
    break;
  }

  return rc;
error:
  return -1;
}

static int32_t rtc_drv_read(void *const buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t rtc_drv_write(const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t rtc_drv_close() {
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

/*******************************************************************************
 * Function Name  : counter_to_struct
 * Description    : populates time-struct based on counter-value
 * Input          : - counter-value (unit seconds, 0 -> 1.1.2000 00:00:00),
 *                  - Pointer to time-struct
 * Output         : time-struct gets populated, DST not taken into account here
 * Return         : none
 *  Based on code from Peter Dannegger found in the mikrocontroller.net forum.
 *******************************************************************************/
static void counter_to_struct(uint32_t sec, struct rtc_s *t) {
  t->sec = sec % 60;
  sec /= 60;
  t->min = sec % 60;
  sec /= 60;
  t->hour = sec % 24;
}

/*******************************************************************************
 * Function Name  : struct_to_counter
 * Description    : calculates second-counter from populated time-struct
 * Input          : Pointer to time-struct
 * Output         : none
 * Return         : counter-value (unit seconds, 0 -> 1.1.2000 00:00:00),
 *  Based on code from "LalaDumm" found in the mikrocontroller.net forum.
 *******************************************************************************/
static uint32_t struct_to_counter(const struct rtc_s *t) { return static_cast<uint32_t>(t->hour) * 3600u + static_cast<uint32_t>(t->min) * 60u + t->sec; }

/*******************************************************************************
 * Function Name  : rtc_gettime
 * Description    : populates structure from HW-RTC, takes DST into account
 * Input          : None
 * Output         : time-struct gets modified
 * Return         : always true/not used
 *******************************************************************************/
static bool rtc_gettime(struct rtc_s *const rtc) {
  uint32_t t;

  while ((t = RTC_GetCounter()) != RTC_GetCounter())
    ;
  counter_to_struct(t, rtc); // get non DST time
  return true;
}

/*******************************************************************************
 * Function Name  : my_rtc_set_cntr
 * Description    : sets the hardware-counter
 * Input          : new counter-value
 * Output         : None
 * Return         : None
 *******************************************************************************/
static void my_rtc_set_cntr(uint32_t cnt) {
  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();
  /* Change the current time */
  RTC_SetCounter(cnt);
  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();
}

/*******************************************************************************
 * Function Name  : rtc_settime
 * Description    : sets HW-RTC with values from time-struct, takes DST into
 *                  account, HW-RTC always running in non-DST time
 * Input          : None
 * Output         : None
 * Return         : not used
 *******************************************************************************/
static bool rtc_settime(const struct rtc_s *rtc) {
  uint32_t cnt;
  struct rtc_s ts;

  cnt = struct_to_counter(rtc); // non-DST counter-value
  counter_to_struct(cnt, &ts);  // normalize struct (for weekday)

  PWR_BackupAccessCmd(ENABLE);
  my_rtc_set_cntr(cnt);
  PWR_BackupAccessCmd(DISABLE);

  return true;
}

static void RTC_irq_handler() {
  BaseType_t rc;
  portBASE_TYPE hp_task_woken;

  if (RTC_GetITStatus(RTC_IT_SEC)) {
    RTC_ClearITPendingBit(RTC_IT_SEC);

    if (RTC_second_callback) {
      typename events_worker_s::event_s event{.handler = RTC_second_callback, .data = nullptr, .size = 0u};
      if ((rc = xQueueSendToBackFromISR(events_worker_queue, &event, &hp_task_woken)) != pdPASS) {
        goto exit;
      }

      if (hp_task_woken == pdTRUE) {
        taskYIELD();
      }
    }

    return;
  } else if (RTC_GetITStatus(RTC_IT_ALR) || RTC_GetITStatus(RTC_IT_OW)) {
    goto exit;
  }

exit:
  RTC_ClearITPendingBit(RTC_IT_SEC | RTC_IT_ALR | RTC_IT_OW);
  return;
}
