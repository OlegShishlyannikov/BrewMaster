#include <cstdarg>

#include "FreeRTOS.h"
#include "fio/unistd.hpp"
#include "semphr.h"
#include "task.h"

#include "drivers/io/rcc_ioctl.hpp"
#include "drivers/io/usart_ioctl.hpp"

#include "apps/brewery/brewery.hpp"
#include "apps/shell/shell.hpp"

extern struct sys_impl_s &sys;
extern bool debug_log_enabled;

static int32_t init_clock();
static int32_t init_console();
static int32_t init_printfmt(const char *fmt, ...);

void init_task_code(void *args) {
  int32_t rc;
  struct app_s **apps;
  size_t app_num;
  struct app_s *shell_app, *brewery_app;
  xTaskHandle brewery_task_handle, shell_task_handle;

  if ((rc = init_clock()) < 0) {
    goto error_state;
  }

  if ((rc = init_console()) < 0) {
    goto error_state;
  }

  apps = sys.apps();
  app_num = sys.apps_num();

  shell_app = sys.app("shell");
  if (!(shell_app)) {
    goto error_state;
  }

  brewery_app = sys.app("brewery");
  if (!(brewery_app)) {
    goto error_state;
  }

  xTaskCreate(shell_app->entry, shell_app->name, configMINIMAL_STACK_SIZE * 6u, args, 5u, &shell_task_handle);
  xTaskCreate(brewery_app->entry, brewery_app->name, configMINIMAL_STACK_SIZE * 24u, args, 8u, &brewery_task_handle);

  while (true) {
    vTaskDelay(100u);
  }

error_state:
  vTaskDelete(nullptr);
}

static int32_t init_printfmt(const char *fmt, ...) {
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

  if (!(usart = sys.drv("usart"))) {
    goto error;
  }

  if ((usart_fd = ::open(usart, "usart1", 3, 3u)) < 0) {
    goto error;
  }

  if ((rc = ::write(usart, usart_fd, temp, strlen)) < 0) {
    goto error;
  }

  if ((rc = ::close(usart, usart_fd)) < 0) {
    goto error;
  }

 exit:
  free(temp);
  return strlen;
error:

  free(temp);
  return -1;
}

static int32_t init_clock() {
  int32_t rc, rcc_fd;
  /* Set SYSCLK frequency */
  if ((rcc_fd = ::open(&sys, "rcc/rcc0", 3, 3u)) < 0) {
    goto error;
  }

  if ((rc = ::ioctl(&sys, "rcc/rcc0", rcc_drv_ioctl_cmd_e::RCC_SET_SYSCLK_72MHZ, nullptr, 0u)) < 0) {
    goto error;
  }

  if ((rc = ::close(&sys, "rcc/rcc0")) < 0) {
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t init_console() {
  int32_t rc, usart_fd;

  struct usart_setup_req_s usart_req {
    .baudrate = 115200u, .irq_priority = 5u
  };

  /* Init USART driver */
  if ((usart_fd = ::open(&sys, "usart/usart1", 3, 3u)) < 0) {
    goto error;
  }

  if ((rc = ::ioctl(&sys, "usart/usart1", usart_ioctl_cmd_e::USART_INIT, &usart_req, sizeof(usart_req))) < 0) {
    goto error;
  }

  if ((rc = ::close(&sys, "usart/usart1")) < 0) {
    goto error;
  }

  return rc;
error:
  return -1;
}
