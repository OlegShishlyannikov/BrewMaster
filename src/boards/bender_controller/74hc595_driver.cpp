#include <cstdarg>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "arch/mcu_tags.hpp"
#include "boards/board_tags.hpp"
#include "drivers/74hc595_driver.hpp"
#include "drivers/io/74hc595_ioctl.hpp"
#include "drivers/io/gpio_ioctl.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "drivers/io/spi_ioctl.hpp"
#include "drivers/io/usart_ioctl.hpp"
#include "fio/unistd.hpp"
#include "sys/events_worker.hpp"

/* This pointer will be used in interrupt handlers and will be initialized in driver init function */
static const struct drv_model_cmn_s *drv_ptr;
extern bool debug_log_enabled;

/* Latch, clock and data pins used */
static constexpr uint8_t ic74hc595_sh_cp_no = 5u, ic74hc595_st_cp_no = 6u, ic74hc595_ds_no = 7u;
static constexpr const char *ic74hc595_drv_spi_devstr = "spi1";
static constexpr const char *ic74hc595_drv_console_devstr = "usart1";

/* 74HC595 lock & fifos */
static xSemaphoreHandle ic74hc595_lock;
extern xQueueHandle events_worker_queue;

// Printf to console
static int32_t ic74hc595_printf(const char *fmt, ...);

/* 74HC595 file IO functions forward reference */
static int32_t ic74hc595_drv_open(int32_t, mode_t);
static int32_t ic74hc595_drv_ioctl(uint64_t, const void *, size_t);
static int32_t ic74hc595_drv_read(void *const, size_t);
static int32_t ic74hc595_drv_write(const void *, size_t);
static int32_t ic74hc595_drv_close();

/* 74HC595 helper functions */
static int32_t ic74hc595_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreTakeRecursive(ic74hc595_lock, portMAX_DELAY)) != pdPASS) {
    // errno = ???
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t ic74hc595_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreGiveRecursive(ic74hc595_lock)) != pdPASS) {
    // errno = ???
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

struct drv_ops_s ic74hc595_drv_ops {
  .init = ic74hc595_drv_init, .exit = ic74hc595_drv_exit
};

/* 74HC595 driver file operations secification */
struct file_ops_s ic74hc595_drv_fops {
  .flock = ic74hc595_flock, .funlock = ic74hc595_funlock, .open = ic74hc595_drv_open, .ioctl = ic74hc595_drv_ioctl, .read = ic74hc595_drv_read, .write = ic74hc595_drv_write,
  .close = ic74hc595_drv_close
};

void ic74hc595_drv_init(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *spi;
  int32_t spi_fd, rc;
  struct spi_setup_req_s spi_setup_req {
    .irq_priority = 6u, .bdr_psc = SPI_BDRPSC_32, .chpa = SPI_CPHA_2EDGE, .cpol = SPI_CPOL_LOW, .datasize = SPI_DATASIZE_16B, .direction = SPI_DIR_1L_TXD, .endianess = SPI_FIRST_BIT_MSB,
    .mode = SPI_MASTER
  };

  struct spi_send_seq_req_s spi_seq_req;

  if (!(spi = drv->dep("spi"))) {
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((spi_fd = ::open(spi, ic74hc595_drv_spi_devstr, 3, 3u)) < 0) {
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_INIT, &spi_setup_req, sizeof(spi_setup_req))) < 0) {
    if ((rc = ::close(spi, spi_fd)) < 0) {
      ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(spi, spi_fd)) < 0) {
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  ic74hc595_drv_fops.owner = drv;

  /* Init locks */
  ic74hc595_lock = xSemaphoreCreateRecursiveMutex();
  xSemaphoreGive(ic74hc595_lock);

  /* Register char device for each GPIO port */
  drv->register_chardev("ic74hc595", &ic74hc595_drv_fops);

  /* Initialize GPIO driver pointer */
  drv_ptr = drv;
  return;

  /* Something went wrong -- reset drv_ptr and exit */
error:
  ic74hc595_drv_exit(drv);
  return;
}

void ic74hc595_drv_exit(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *spi;
  int32_t rc, spi_fd;

  if (!(spi = drv->dep("spi"))) {
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((spi_fd = ::open(spi, ic74hc595_drv_spi_devstr, 3, 3u)) < 0) {
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_DEINIT, nullptr, 0u)) < 0) {
    if ((rc = ::close(spi, spi_fd)) < 0) {
      ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(spi, spi_fd)) < 0) {
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  ic74hc595_drv_fops.owner = nullptr;

  /* Remove locks */
  vSemaphoreDelete(ic74hc595_lock);

  /* Register char device for each GPIO port */
  drv->unregister_chardev("ic74hc595");

  /* Initialize GPIO driver pointer */
  drv_ptr = nullptr;
  return;
error:
  return;
}

/* 74HC595 file IO functions */
static int32_t ic74hc595_drv_open(int32_t, mode_t) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t ic74hc595_drv_ioctl(uint64_t req, const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t ic74hc595_drv_read(void *const, size_t) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t ic74hc595_drv_write(const void *buf, size_t size) {
  const struct drv_model_cmn_s *spi, *gpio;
  int32_t spi_fd, rc;
  struct spi_send_seq_req_s spi_seq_req;
  const uint16_t *data;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(spi = drv_ptr->dep("spi"))) {
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(gpio = drv_ptr->dep("gpio"))) {
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((spi_fd = ::open(spi, ic74hc595_drv_spi_devstr, 2, 3u)) < 0) {
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Select 74HC595 chip
  if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_SELECT, nullptr, 0u)) < 0) {
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Write sequence to 74HC595
  /* Assuming that buffer has data type and size of uint16 */
  spi_seq_req.seq = *reinterpret_cast<const uint16_t *>(buf);
  if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_SEND_SEQ, &spi_seq_req, sizeof(spi_seq_req))) < 0) {
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Unselect 74HC595 chip
  if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_UNSELECT, nullptr, 0u)) < 0) {
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(spi, spi_fd)) < 0) {
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t ic74hc595_drv_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ic74hc595_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t ic74hc595_printf(const char *fmt, ...) {
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
    if ((usart_fd = ::open(usart, ic74hc595_drv_console_devstr, 3, 3u)) < 0) {
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, "[74hc595] : ", std::strlen("[74hc595] : "))) < 0) {
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
