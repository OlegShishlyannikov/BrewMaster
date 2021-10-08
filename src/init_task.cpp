#include <cstdarg>

#include "FreeRTOS.h"
#include "fio/unistd.hpp"
#include "semphr.h"
#include "task.h"

#include "drivers/io/rcc_ioctl.hpp"
#include "drivers/io/usart_ioctl.hpp"

#include "apps/bender/bender.hpp"
#include "apps/shell/shell.hpp"

extern struct sys_impl_s &sys;
extern bool debug_log_enabled;

static constexpr const char *console_driver_name = "usart";
static constexpr const char *console_device_name = "usart1";
static constexpr const char *console_device_path = "usart/usart1";

static int32_t init_clock();
static int32_t init_console();
static int32_t init_printfmt(const char *fmt, ...);

void init_task_code(void *args) {
  int32_t rc;
  struct app_s **apps;
  size_t app_num;
  struct app_s *shell_app, *bender_app, *modbus_app;
  xTaskHandle bender_task_handle, shell_task_handle, modbus_task_handle;

  if ((rc = init_clock()) < 0) {
	init_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error_state;
  }

  // if ((rc = init_console()) < 0) {
    // goto error_state;
  // }

  apps = sys.apps();
  app_num = sys.apps_num();

  // shell_app = sys.app("shell");
  // if (!(shell_app)) {
  //   goto error_state;
  // }

  bender_app = sys.app("bender");
  if (!(bender_app)) {
    goto error_state;
  }

  // xTaskCreate(shell_app->entry, shell_app->name, configMINIMAL_STACK_SIZE * 6u, args, 5u, &shell_task_handle);
  xTaskCreate(bender_app->entry, bender_app->name, configMINIMAL_STACK_SIZE * 32u, args, 8u, &bender_task_handle);
  // xTaskCreate(modbus_app->entry, modbus_app->name, configMINIMAL_STACK_SIZE * 6u, args, 5u, &modbus_task_handle);
  
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

  if (!(usart = sys.drv(console_driver_name))) {
    goto error;
  }

  if ((usart_fd = ::open(usart, console_device_name, 3, 3u)) < 0) {
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
	init_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, "rcc/rcc0", rcc_drv_ioctl_cmd_e::RCC_SET_SYSCLK_72MHZ, nullptr, 0u)) < 0) {
	init_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, "rcc/rcc0")) < 0) {
	init_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t init_console() {
  int32_t rc, usart_fd;

  struct usart_setup_req_s usart_req {
    .baudrate = 115200u, .irq_priority = 5u, .hw_flow_ctrl = usart_hw_flow_ctrl_e::NONE, .mode = usart_mode_e::RXTX, .parity = usart_parity_e::NO, .sb = usart_stop_bits_e::SB_1,
    .wl = usart_word_len_e::WL_8B
  };

  /* Init USART driver */
  if ((usart_fd = ::open(&sys, console_device_path, 3, 3u)) < 0) {
	init_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, console_device_path, usart_ioctl_cmd_e::USART_INIT, &usart_req, sizeof(usart_req))) < 0) {
	init_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, console_device_path)) < 0) {
	init_printfmt("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}
