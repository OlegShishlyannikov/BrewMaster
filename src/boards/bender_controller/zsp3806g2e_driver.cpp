#include <cstdarg>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "arch/mcu_tags.hpp"
#include "boards/board_tags.hpp"
#include "drivers/io/gpio_ioctl.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "drivers/io/spi_ioctl.hpp"
#include "drivers/io/usart_ioctl.hpp"
#include "drivers/zsp380g2e_driver.hpp"
#include "fio/unistd.hpp"
#include "sys/events_worker.hpp"

static const struct drv_model_cmn_s *drv_ptr;
extern xQueueHandle events_worker_queue;

// Global flags
extern bool debug_log_enabled;

/* Button0 lock */
static xSemaphoreHandle zsp3806g2e_lock;
