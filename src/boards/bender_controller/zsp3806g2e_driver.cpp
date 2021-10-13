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
#include "drivers/io/zsp3806g2e_ioctl.hpp"
#include "drivers/zsp3806g2e_driver.hpp"
#include "fio/unistd.hpp"
#include "sys/events_worker.hpp"

/* This pointer will be used in interrupt handlers and will be initialized in driver init function */
static const struct drv_model_cmn_s *drv_ptr;
extern bool debug_log_enabled;
static constexpr const char *console_devstr = "usart2";

/* Latch, clock and data pins used */
// static constexpr uint8_t zsp3806g2e_miso_pin = 5u, zsp3806g2e_cs_pin = 6u, zsp3806g2e_data_pin = 7u;

/* ZSP3806G2E lock & fifos */
static xSemaphoreHandle zsp3806g2e_lock;
extern xQueueHandle events_worker_queue;

// Printf to console
static int32_t zsp3806g2e_printf(const char *fmt, ...);

/* ZSP3806G2E file IO functions forward reference */
static int32_t zsp3806g2e_drv_open(int32_t, mode_t);
static int32_t zsp3806g2e_drv_ioctl(uint64_t, const void *, size_t);
static int32_t zsp3806g2e_drv_read(void *const, size_t);
static int32_t zsp3806g2e_drv_write(const void *, size_t);
static int32_t zsp3806g2e_drv_close();

/* ZSP3806G2E helper functions */
static int32_t zsp3806g2e_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreTakeRecursive(zsp3806g2e_lock, portMAX_DELAY)) != pdPASS) {
    // errno = ???
    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t zsp3806g2e_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreGiveRecursive(zsp3806g2e_lock)) != pdPASS) {
    // errno = ???
    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

struct drv_ops_s zsp3806g2e_drv_ops {
  .init = zsp3806g2e_drv_init, .exit = zsp3806g2e_drv_exit
};

/* ZSP3806G2E driver file operations secification */
struct file_ops_s zsp3806g2e_drv_fops {
  .flock = zsp3806g2e_flock, .funlock = zsp3806g2e_funlock, .open = zsp3806g2e_drv_open, .ioctl = zsp3806g2e_drv_ioctl, .read = zsp3806g2e_drv_read, .write = zsp3806g2e_drv_write,
  .close = zsp3806g2e_drv_close
};

void zsp3806g2e_drv_init(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *spi;
  int32_t spi_fd, rc;
  struct spi_setup_req_s spi_setup_req {
    .irq_priority = 6u, .bdr_psc = SPI_BDRPSC_256, .chpa = SPI_CPHA_1EDGE, .cpol = SPI_CPOL_LOW, .datasize = SPI_DATASIZE_16B, .direction = SPI_DIR_2L_FD, .endianess = SPI_FIRST_BIT_MSB,
    .mode = SPI_MASTER
  };

  if (!(spi = drv->dep("spi"))) {
    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((spi_fd = ::open(spi, "spi3", 3, 3u)) < 0) {
    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_INIT, &spi_setup_req, sizeof(spi_setup_req))) < 0) {
    if ((rc = ::close(spi, spi_fd)) < 0) {
      zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(spi, spi_fd)) < 0) {
    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  zsp3806g2e_drv_fops.owner = drv;

  /* Init locks */
  zsp3806g2e_lock = xSemaphoreCreateRecursiveMutex();
  xSemaphoreGive(zsp3806g2e_lock);

  /* Register char device for each GPIO port */
  drv->register_chardev("zsp3806g2e", &zsp3806g2e_drv_fops);

  /* Initialize GPIO driver pointer */
  drv_ptr = drv;
  return;

  /* Something went wrong -- reset drv_ptr and exit */
error:
  zsp3806g2e_drv_exit(drv);
  return;
}

void zsp3806g2e_drv_exit(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *spi;
  int32_t rc, spi_fd;

  if (!(spi = drv->dep("spi"))) {
    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((spi_fd = ::open(spi, "spi3", 3, 3u)) < 0) {
    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_DEINIT, nullptr, 0u)) < 0) {
    if ((rc = ::close(spi, spi_fd)) < 0) {
      zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(spi, spi_fd)) < 0) {
    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  zsp3806g2e_drv_fops.owner = nullptr;

  /* Remove locks */
  vSemaphoreDelete(zsp3806g2e_lock);

  /* Register char device for each GPIO port */
  drv->unregister_chardev("zsp3806g2e");

  /* Initialize GPIO driver pointer */
  drv_ptr = nullptr;
  return;
error:
  return;
}

/* ZSP3806G2E file IO functions */
static int32_t zsp3806g2e_drv_open(int32_t, mode_t) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t zsp3806g2e_drv_ioctl(uint64_t req, const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t zsp3806g2e_drv_read(void *const buf, size_t size) {
  const struct drv_model_cmn_s *spi;
  int32_t spi_fd, rc;
  uint16_t recvd;
  struct spi_send_seq_req_s spi_seq_req {
    .seq = 0xffff
  };

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(spi = drv_ptr->dep("spi"))) {
    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((spi_fd = ::open(spi, "spi3", 3, 3u)) < 0) {
    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Select
  if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_SELECT, nullptr, 0u)) < 0) {
    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Read
  if ((rc = ::read(spi, spi_fd, &recvd, sizeof(recvd))) < 0) {
    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Unselect
  if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_UNSELECT, nullptr, 0u)) < 0) {
    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(spi, spi_fd)) < 0) {
    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  *reinterpret_cast<uint16_t *const>(buf) = recvd;
  return 0;
error:
  return -1;
}

static int32_t zsp3806g2e_drv_write(const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t zsp3806g2e_drv_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    zsp3806g2e_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t zsp3806g2e_printf(const char *fmt, ...) {
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

    if ((rc = ::write(usart, usart_fd, "[usart] : ", std::strlen("[usart] : "))) < 0) {
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
