#include <cstdarg>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "arch/mcu_tags.hpp"
#include "boards/board_tags.hpp"
#include "drivers/io/gpio_event.hpp"
#include "drivers/io/gpio_ioctl.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "drivers/io/spi_event.hpp"
#include "drivers/io/spi_ioctl.hpp"
#include "drivers/spi_driver.hpp"
#include "fio/unistd.hpp"
#include "sys/events_worker.hpp"

static const struct drv_model_cmn_s *drv_ptr;
extern xQueueHandle events_worker_queue;
extern bool debug_log_enabled;
static SPI_TypeDef *spi_periph[]{SPI1, SPI2, SPI3};

// SPI device FIFOs
static xQueueHandle SPI_fifos[sizeof(spi_periph) / sizeof(spi_periph[0u])]{xQueueHandle()};

/* SPI locks */
static xSemaphoreHandle SPI_locks[sizeof(spi_periph) / sizeof(spi_periph[0u])]{xSemaphoreHandle()};
static xSemaphoreHandle SPI_dma_locks[sizeof(spi_periph) / sizeof(spi_periph[0u])]{xSemaphoreHandle()};

static int32_t spi_drv_printf(const char *fmt, ...);
static int32_t spi_drv_SPI1_open(int32_t, mode_t);
static int32_t spi_drv_SPI1_ioctl(uint64_t, const void *, size_t);
static int32_t spi_drv_SPI1_read(void *const, size_t);
static int32_t spi_drv_SPI1_write(const void *, size_t);
static int32_t spi_drv_SPI1_close();

static int32_t spi_drv_SPI3_open(int32_t, mode_t);
static int32_t spi_drv_SPI3_ioctl(uint64_t, const void *, size_t);
static int32_t spi_drv_SPI3_read(void *const, size_t);
static int32_t spi_drv_SPI3_write(const void *, size_t);
static int32_t spi_drv_SPI3_close();

static void SPI_irq_handler();
static void SPI1_dma_irq_handler();

static void (*SPI_on_recv_callbacks[sizeof(spi_periph) / sizeof(spi_periph[0u])])(const void *, size_t){nullptr};
static void (*SPI_on_send_callbacks[sizeof(spi_periph) / sizeof(spi_periph[0u])])(const void *, size_t){nullptr};

// DMA1_Channel3 -- SPI1_TX mapped
static DMA_Channel_TypeDef *SPI1_dma_channel = DMA1_Channel3;
static constexpr uint32_t SPI1_dma_irq_vector_num = DMA1_Channel3_IRQHandler_Vector, SPI1_irq_vector_num = SPI1_IRQHandler_Vector, SPI3_irq_vector_num = SPI3_IRQHandler_Vector;
static const IRQn_Type SPI1_dma_irq_num = DMA1_Channel3_IRQn, SPI1_irq_num = SPI1_IRQn, SPI3_irq_num = SPI3_IRQn;

// SPI1 pins
static constexpr const uint8_t SPI1_mosi_pin_no = 7u, SPI1_sck_pin_no = 5u;
static constexpr const uint8_t SPI1_miso_pin_no = 6u, SPI1_nss_pin_no = 4u;

// SPI3 pins
static constexpr const uint8_t SPI3_mosi_pin_no = 5u, SPI3_sck_pin_no = 3u;
static constexpr const uint8_t SPI3_miso_pin_no = 4u, SPI3_nss_pin_no = 2u;

// SPI FIFO sizes
static constexpr const uint32_t SPI1_drv_rxd_buffer_size = 32u;
static constexpr const uint32_t SPI3_drv_rxd_buffer_size = 32u;

// SPI setup declarations
static const uint16_t spi_bdr_pscs[]{SPI_BaudRatePrescaler_2,  SPI_BaudRatePrescaler_4,  SPI_BaudRatePrescaler_8,   SPI_BaudRatePrescaler_16,
                                     SPI_BaudRatePrescaler_32, SPI_BaudRatePrescaler_64, SPI_BaudRatePrescaler_128, SPI_BaudRatePrescaler_256};

static const uint16_t spi_cpha[]{SPI_CPHA_1Edge, SPI_CPHA_2Edge};
static const uint16_t spi_cpol[]{SPI_CPOL_Low, SPI_CPOL_High};
static const uint16_t spi_datasize[]{SPI_DataSize_8b, SPI_DataSize_16b};
static const uint16_t spi_dir[]{SPI_Direction_2Lines_FullDuplex, SPI_Direction_2Lines_RxOnly, SPI_Direction_1Line_Rx, SPI_Direction_1Line_Tx};
static const uint16_t spi_endianess[]{SPI_FirstBit_MSB, SPI_FirstBit_LSB};
static const uint16_t spi_mode[]{SPI_Mode_Master, SPI_Mode_Slave};

/* SPI1 helper functions */
static int32_t SPI1_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreTakeRecursive(SPI_locks[0u], portIO_MAX_DELAY)) != pdPASS) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t SPI1_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreGiveRecursive(SPI_locks[0u])) != pdPASS) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

/* SPI3 helper functions */
static int32_t SPI3_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if (!(rc = xSemaphoreTakeRecursive(SPI_locks[2u], portIO_MAX_DELAY))) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t SPI3_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if (!(rc = xSemaphoreGiveRecursive(SPI_locks[2u]))) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

struct drv_ops_s spi_drv_ops {
  .init = spi_drv_init, .exit = spi_drv_exit
};

/* SPI driver file operations secification */
struct file_ops_s spi_drv_SPI1_fops {
  .flock = SPI1_flock, .funlock = SPI1_funlock, .open = spi_drv_SPI1_open, .ioctl = spi_drv_SPI1_ioctl, .read = spi_drv_SPI1_read, .write = spi_drv_SPI1_write, .close = spi_drv_SPI1_close
};

struct file_ops_s spi_drv_SPI3_fops {
  .flock = SPI3_flock, .funlock = SPI3_funlock, .open = spi_drv_SPI3_open, .ioctl = spi_drv_SPI3_ioctl, .read = spi_drv_SPI3_read, .write = spi_drv_SPI3_write, .close = spi_drv_SPI3_close
};

void spi_drv_init(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *rcc, *gpio;
  struct rcc_drv_req_s rcc_req;
  struct gpio_setup_req_s gpio_req;
  struct gpio_write_pin_req_s gpio_pin_write_req;
  int32_t rcc_fd, gpio_fd, rc;

  /* Get rcc driver from dependencies */
  if (!(rcc = drv->dep("rcc"))) {
    // errno = ENOENT
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Open RCC device */
  if ((rcc_fd = ::open(rcc, "rcc0", 3, 2u)) < 0) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Enable SPI1 clocking */
  rcc_req = {rcc_bus_e::APB2, rcc_apb2_periph_e::RCC_SPI1};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_ENABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Enable SPI3 clocking */
  rcc_req = {rcc_bus_e::APB1, rcc_apb1_periph_e::RCC_SPI3};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_ENABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Enable DMA1 clocking */
  rcc_req = {rcc_bus_e::AHB, rcc_ahb_periph_e::RCC_DMA1};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_ENABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Close rcc file */
  if ((rc = ::close(rcc, rcc_fd)) < 0) {
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Get rcc driver from dependencies */
  if (!(gpio = drv->dep("gpio"))) {
    // errno = ENOENT
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Open GPIOA device file */
  if ((gpio_fd = ::open(gpio, "A", 3, 2u)) < 0) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Initialize SPI1 MOSI and SCK pins
  for (uint8_t pin : {SPI1_mosi_pin_no, SPI1_sck_pin_no}) {
    gpio_req = {.pin = pin};
    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_ALT_PP, &gpio_req, sizeof(gpio_req))) < 0) {
      // errno = ???
      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  // Initialize SPI1 NSS pin
  gpio_req = {.pin = SPI1_nss_pin_no};
  if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_PP, &gpio_req, sizeof(gpio_req))) < 0) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // SPI1 NCS up
  gpio_pin_write_req = {.pin = SPI1_nss_pin_no, .val = 1u};
  if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_PIN_WRITE, &gpio_pin_write_req, sizeof(gpio_pin_write_req))) < 0) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Initialize MISO pin
  gpio_req = {.pin = SPI1_miso_pin_no};
  if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IPH, &gpio_req, sizeof(gpio_req))) < 0) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Close GPIOA file */
  if ((rc = ::close(gpio, gpio_fd)) < 0) {
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Open GPIOB device file */
  if ((gpio_fd = ::open(gpio, "B", 3, 2u)) < 0) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Initialize SPI3 MOSI and SCK pins
  for (uint8_t pin : {SPI3_mosi_pin_no, SPI3_sck_pin_no}) {
    gpio_req = {.pin = pin};
    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_ALT_PP, &gpio_req, sizeof(gpio_req))) < 0) {
      // errno = ???
      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  // Initialize SPI3 MISO pin
  gpio_req = {.pin = SPI3_miso_pin_no};
  if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IPH, &gpio_req, sizeof(gpio_req))) < 0) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Initialize SPI3 NSS pin
  gpio_req = {.pin = SPI3_nss_pin_no};
  if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_PP, &gpio_req, sizeof(gpio_req))) < 0) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // SPI3 NCS up
  gpio_pin_write_req = {.pin = SPI3_nss_pin_no, .val = 1u};
  if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_PIN_WRITE, &gpio_pin_write_req, sizeof(gpio_pin_write_req))) < 0) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Close GPIOB file */
  if ((rc = ::close(gpio, gpio_fd)) < 0) {
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  spi_drv_SPI1_fops.owner = drv;
  spi_drv_SPI3_fops.owner = drv;

  /* Init SPI1 locks */
  SPI_locks[0u] = xSemaphoreCreateRecursiveMutex();
  SPI_dma_locks[0u] = xSemaphoreCreateBinary();

  /* Init SPI3 lock */
  SPI_locks[2u] = xSemaphoreCreateRecursiveMutex();

  // Init SPI FIFOs
  SPI_fifos[0u] = xQueueCreate(SPI1_drv_rxd_buffer_size, sizeof(uint16_t));
  SPI_fifos[2u] = xQueueCreate(SPI3_drv_rxd_buffer_size, sizeof(uint16_t));

  /* Register char device for each GPIO port */
  drv->register_chardev("spi1", &spi_drv_SPI1_fops);
  drv->register_chardev("spi3", &spi_drv_SPI3_fops);

  /* Initialize GPIO driver pointer */
  drv_ptr = drv;
  return;

  /* Something went wrong -- reset drv_ptr and exit */
error:
  spi_drv_exit(drv);
  return;
}

void spi_drv_exit(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *rcc, *gpio;
  struct rcc_drv_req_s rcc_req;
  struct gpio_setup_req_s gpio_req;
  int32_t rcc_fd, gpio_fd, rc;

  /* Get rcc driver from dependencies */
  if (!(rcc = drv->dep("rcc"))) {
    // errno = ENOENT
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Open RCC device */
  if ((rcc_fd = ::open(rcc, "rcc0", 3, 2u)) < 0) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Disable SPI1 clocking */
  rcc_req = {rcc_bus_e::APB2, rcc_apb2_periph_e::RCC_SPI1};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_DISABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Disable SPI3 clocking */
  rcc_req = {rcc_bus_e::APB1, rcc_apb1_periph_e::RCC_SPI3};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_DISABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Close rcc file */
  if ((rc = ::close(rcc, rcc_fd)) < 0) {
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Get rcc driver from dependencies */
  if (!(gpio = drv->dep("gpio"))) {
    // errno = ENOENT
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Open GPIOA device file */
  if ((gpio_fd = ::open(gpio, "A", 3, 2u)) < 0) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Initialize SPI1 MOSI and SCK pins
  for (uint8_t pin : {SPI1_mosi_pin_no, SPI1_sck_pin_no}) {
    gpio_req = {.pin = pin};
    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_req, sizeof(gpio_req))) < 0) {
      // errno = ???
      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  // Restore default setting SPI1 NSS pin
  gpio_req = {.pin = SPI1_nss_pin_no};
  if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_req, sizeof(gpio_req))) < 0) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Restore default setting MISO pin
  gpio_req = {.pin = SPI1_miso_pin_no};
  if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_req, sizeof(gpio_req))) < 0) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Close GPIOA file */
  if ((rc = ::close(gpio, gpio_fd)) < 0) {
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Open GPIOB device file */
  if ((gpio_fd = ::open(gpio, "B", 3, 2u)) < 0) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Restore default settings SPI3 MOSI and SCK pins
  for (uint8_t pin : {SPI3_mosi_pin_no, SPI3_sck_pin_no}) {
    gpio_req = {.pin = pin};
    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_req, sizeof(gpio_req))) < 0) {
      // errno = ???
      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  // Restore default setting on SPI3 MISO pin
  gpio_req = {.pin = SPI3_miso_pin_no};
  if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_req, sizeof(gpio_req))) < 0) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Restore default setting SPI3 NSS pin
  gpio_req = {.pin = SPI3_nss_pin_no};
  if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_req, sizeof(gpio_req))) < 0) {
    // errno = ???
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Close GPIOB file */
  if ((rc = ::close(gpio, gpio_fd)) < 0) {
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  spi_drv_SPI1_fops.owner = nullptr;
  spi_drv_SPI3_fops.owner = nullptr;

  /* Remove SPI1 locks */
  vSemaphoreDelete(SPI_locks[0u]);
  vSemaphoreDelete(SPI_dma_locks[0u]);

  /* Remove SPI3 lock */
  vSemaphoreDelete(SPI_locks[2u]);

  // Remove SPI FIFOs
  vQueueDelete(SPI_fifos[0u]);
  vQueueDelete(SPI_fifos[2u]);

  /* Register char device for each GPIO port */
  drv->unregister_chardev("spi1");
  drv->unregister_chardev("spi3");

  drv_ptr = nullptr;
  return;
error:
  return;
};

static int32_t spi_drv_SPI1_open(int32_t oflags, mode_t mode) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t spi_drv_SPI1_ioctl(uint64_t req, const void *buf, size_t size) {
  int32_t rc;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (req) {
  case SPI_INIT: {
    const struct spi_setup_req_s *spi_req = reinterpret_cast<const struct spi_setup_req_s *>(buf);
    SPI_InitTypeDef spi;
    SPI_StructInit(&spi);
    spi.SPI_BaudRatePrescaler = spi_bdr_pscs[spi_req->bdr_psc];
    spi.SPI_CPHA = spi_cpha[spi_req->chpa];
    spi.SPI_CPOL = spi_cpol[spi_req->cpol];
    spi.SPI_DataSize = spi_datasize[spi_req->datasize];
    spi.SPI_Direction = spi_dir[spi_req->direction];
    spi.SPI_FirstBit = spi_endianess[spi_req->endianess];
    spi.SPI_Mode = spi_mode[spi_req->mode];
    spi.SPI_NSS = SPI_NSS_Soft;
    SPI_Init(spi_periph[0u], &spi);

    g_pfnVectorsRam[SPI1_dma_irq_vector_num] = SPI1_dma_irq_handler;
    g_pfnVectorsRam[SPI1_irq_vector_num] = SPI_irq_handler;

    SPI_I2S_ITConfig(spi_periph[0u], SPI_I2S_IT_RXNE, ENABLE);
    DMA_ITConfig(SPI1_dma_channel, DMA_IT_TC, ENABLE);
    SPI_Cmd(spi_periph[0u], ENABLE);

    NVIC_SetPriority(SPI1_irq_num, spi_req->irq_priority);
    NVIC_EnableIRQ(SPI1_irq_num);
    NVIC_SetPriority(SPI1_dma_irq_num, spi_req->irq_priority);
    NVIC_EnableIRQ(SPI1_dma_irq_num);
  } break;

  case SPI_DEINIT: {
    SPI_Cmd(spi_periph[0u], DISABLE);
    SPI_I2S_DeInit(spi_periph[0u]);
    SPI_I2S_ITConfig(spi_periph[0u], SPI_I2S_IT_RXNE, DISABLE);
    DMA_ITConfig(SPI1_dma_channel, DMA_IT_TC, DISABLE);
    NVIC_DisableIRQ(SPI1_irq_num);
    NVIC_DisableIRQ(SPI1_dma_irq_num);

    g_pfnVectorsRam[SPI1_dma_irq_vector_num] = nullptr;
    g_pfnVectorsRam[SPI1_irq_vector_num] = nullptr;
  } break;

  case SPI_ON_RECV: {
    const struct spi_callback_req_s *spi_req = reinterpret_cast<const struct spi_callback_req_s *>(buf);
    SPI_on_recv_callbacks[0u] = spi_req->callback;
  } break;

  case SPI_ON_SEND: {
    const struct spi_callback_req_s *spi_req = reinterpret_cast<const struct spi_callback_req_s *>(buf);
    SPI_on_send_callbacks[0u] = spi_req->callback;
  } break;

  case SPI_SELECT: {
    const struct drv_model_cmn_s *gpio;
    int32_t gpio_fd;
    const struct gpio_write_pin_req_s gpio_req { .pin = SPI1_nss_pin_no, .val = 0u };

    /* Get gpio driver from dependencies */
    if (!(gpio = drv_ptr->dep("gpio"))) {
      // errno = ENOENT
      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    /* Open GPIOA device file */
    if ((gpio_fd = ::open(gpio, "A", 3, 2u)) < 0) {
      // errno = ???
      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_PIN_WRITE, &gpio_req, sizeof(gpio_req))) < 0) {
      if ((rc = ::close(gpio, gpio_fd)) < 0) {
        spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    /* Close GPIOA file */
    if ((rc = ::close(gpio, gpio_fd)) < 0) {
      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case SPI_UNSELECT: {
    const struct drv_model_cmn_s *gpio;
    int32_t gpio_fd;
    const struct gpio_write_pin_req_s gpio_req { .pin = SPI1_nss_pin_no, .val = 1u };

    /* Get gpio driver from dependencies */
    if (!(gpio = drv_ptr->dep("gpio"))) {
      // errno = ENOENT
      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    /* Open GPIOA device file */
    if ((gpio_fd = ::open(gpio, "A", 3, 2u)) < 0) {
      // errno = ???
      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_PIN_WRITE, &gpio_req, sizeof(gpio_req))) < 0) {
      if ((rc = ::close(gpio, gpio_fd)) < 0) {
        spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    /* Close GPIOA file */
    if ((rc = ::close(gpio, gpio_fd)) < 0) {
      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case SPI_SEND_SEQ: {
    const struct spi_send_seq_req_s *spi_seq_req = reinterpret_cast<const struct spi_send_seq_req_s *>(buf);
    while (SPI_I2S_GetFlagStatus(spi_periph[0u], SPI_I2S_FLAG_BSY))
      ;
    SPI_I2S_SendData(spi_periph[0u], spi_seq_req->seq);
  } break;
  }

  return 0;
error:
  return -1;
}

static int32_t spi_drv_SPI1_read(void *const buf, size_t size) {
  BaseType_t rc;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!buf) {
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (size % sizeof(uint16_t)) {
    uint16_t from_queue[size / sizeof(uint16_t) + 1u];
    for (size_t n = 0u; n < sizeof(from_queue) / sizeof(uint16_t); n++) {
      if ((rc = xQueueReceive(SPI_fifos[0u], &from_queue[n], portIO_MAX_DELAY)) != pdPASS) {
        spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      *reinterpret_cast<uint8_t *>(buf) = from_queue[n];
    }
  } else {
    for (size_t n = 0u; n < size / sizeof(uint16_t); n++) {
      if ((rc = xQueueReceive(SPI_fifos[0u], reinterpret_cast<uint16_t *const>(buf) + n, portIO_MAX_DELAY)) != pdPASS) {
        spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }
    }
  }

  return 0;
error:
  return -1;
}

static int32_t spi_drv_SPI1_write(const void *buf, size_t size) {
  BaseType_t rc;
  DMA_InitTypeDef dma;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  DMA_Cmd(DMA1_Channel4, DISABLE);
  DMA_StructInit(&dma);
  dma.DMA_PeripheralBaseAddr = reinterpret_cast<size_t>(&spi_periph[0u]->DR);
  dma.DMA_MemoryBaseAddr = reinterpret_cast<const size_t>(buf);
  dma.DMA_BufferSize = size;
  dma.DMA_DIR = DMA_DIR_PeripheralDST;
  dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
  dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_Init(SPI1_dma_channel, &dma);
  DMA_ITConfig(SPI1_dma_channel, DMA_IT_TC, ENABLE);
  SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
  DMA_Cmd(SPI1_dma_channel, ENABLE);

  // Take lock (if lock hadn't been taken after timeout -- stop transaction)
  if ((rc = xSemaphoreTake(SPI_dma_locks[0u], portIO_MAX_DELAY))) {

    DMA_DeInit(SPI1_dma_channel);
    DMA_Cmd(SPI1_dma_channel, DISABLE);
  }

  return 0;
error:
  return -1;
}

static int32_t spi_drv_SPI1_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t spi_drv_SPI3_open(int32_t oflags, mode_t mode) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t spi_drv_SPI3_ioctl(uint64_t req, const void *buf, size_t size) {
  int32_t rc;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (req) {

  case SPI_INIT: {
    const struct spi_setup_req_s *spi_req = reinterpret_cast<const struct spi_setup_req_s *>(buf);
    SPI_InitTypeDef spi;
    SPI_StructInit(&spi);
    spi.SPI_BaudRatePrescaler = spi_bdr_pscs[spi_req->bdr_psc];
    spi.SPI_CPHA = spi_cpha[spi_req->chpa];
    spi.SPI_CPOL = spi_cpol[spi_req->cpol];
    spi.SPI_DataSize = spi_datasize[spi_req->datasize];
    spi.SPI_Direction = spi_dir[spi_req->direction];
    spi.SPI_FirstBit = spi_endianess[spi_req->endianess];
    spi.SPI_Mode = spi_mode[spi_req->mode];
    spi.SPI_NSS = SPI_NSS_Soft;
    SPI_Init(spi_periph[2u], &spi);

    // g_pfnVectorsRam[SPI3_irq_vector_num] = SPI_irq_handler;
    // SPI_I2S_ITConfig(spi_periph[2u], SPI_I2S_IT_RXNE, ENABLE);
    SPI_Cmd(spi_periph[2u], ENABLE);

    // NVIC_SetPriority(SPI3_irq_num, spi_req->irq_priority);
    // NVIC_EnableIRQ(SPI3_irq_num);
  } break;

  case SPI_DEINIT: {
    SPI_Cmd(spi_periph[2u], DISABLE);
    SPI_I2S_DeInit(spi_periph[2u]);
    SPI_I2S_ITConfig(spi_periph[2u], SPI_I2S_IT_RXNE, DISABLE);
    // NVIC_DisableIRQ(SPI3_irq_num);
    // g_pfnVectorsRam[SPI3_irq_vector_num] = nullptr;
  } break;

  case SPI_ON_RECV: {
    const struct spi_callback_req_s *spi_req = reinterpret_cast<const struct spi_callback_req_s *>(buf);
    SPI_on_recv_callbacks[0u] = spi_req->callback;
  } break;

  case SPI_ON_SEND: {
    const struct spi_callback_req_s *spi_req = reinterpret_cast<const struct spi_callback_req_s *>(buf);
    SPI_on_send_callbacks[0u] = spi_req->callback;
  } break;

  case SPI_SELECT: {
    const struct drv_model_cmn_s *gpio;
    int32_t gpio_fd;
    const struct gpio_write_pin_req_s gpio_req { .pin = SPI3_nss_pin_no, .val = 0u };

    /* Get gpio driver from dependencies */
    if (!(gpio = drv_ptr->dep("gpio"))) {
      // errno = ENOENT
      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    /* Open GPIOA device file */
    if ((gpio_fd = ::open(gpio, "B", 3, 2u)) < 0) {
      // errno = ???
      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_PIN_WRITE, &gpio_req, sizeof(gpio_req))) < 0) {
      if ((rc = ::close(gpio, gpio_fd)) < 0) {
        spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    /* Close GPIOA file */
    if ((rc = ::close(gpio, gpio_fd)) < 0) {
      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case SPI_UNSELECT: {
    const struct drv_model_cmn_s *gpio;
    int32_t gpio_fd;
    const struct gpio_write_pin_req_s gpio_req { .pin = SPI3_nss_pin_no, .val = 1u };

    /* Get gpio driver from dependencies */
    if (!(gpio = drv_ptr->dep("gpio"))) {
      // errno = ENOENT
      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    /* Open GPIOA device file */
    if ((gpio_fd = ::open(gpio, "B", 3, 2u)) < 0) {
      // errno = ???
      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_PIN_WRITE, &gpio_req, sizeof(gpio_req))) < 0) {
      if ((rc = ::close(gpio, gpio_fd)) < 0) {
        spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    /* Close GPIOA file */
    if ((rc = ::close(gpio, gpio_fd)) < 0) {
      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case SPI_SEND_SEQ: {
    const struct spi_send_seq_req_s *spi_seq_req = reinterpret_cast<const struct spi_send_seq_req_s *>(buf);
    while (SPI_I2S_GetFlagStatus(spi_periph[2u], SPI_I2S_FLAG_BSY))
      ;
    SPI_I2S_SendData(spi_periph[2u], spi_seq_req->seq);
  } break;
  }

  return 0;
error:
  return -1;
}

static int32_t spi_drv_SPI3_read(void *const buf, size_t size) {
  BaseType_t rc;
  struct spi_send_seq_req_s spi_seq_req {
    .seq = 0xffff
  };
  uint16_t recvd;
  
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  while (SPI_I2S_GetFlagStatus(spi_periph[2u], SPI_I2S_FLAG_BSY))
    ;
  SPI_I2S_SendData(spi_periph[2u], spi_seq_req.seq);
  while(!SPI_I2S_GetFlagStatus(spi_periph[2u], SPI_I2S_FLAG_RXNE))
    ;
  
  recvd = SPI_I2S_ReceiveData(SPI3);  
  *reinterpret_cast<uint16_t *>(buf)  = recvd;
  
  // if (size % sizeof(uint16_t)) {
  //   uint16_t from_queue[size / sizeof(uint16_t) + 1u];
  //   for (size_t n = 0u; n < sizeof(from_queue) / sizeof(uint16_t); n++) {
  //     if ((rc = xQueueReceive(SPI_fifos[2u], &from_queue[n], portIO_MAX_DELAY)) != pdPASS) {
  //       spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  //       goto error;
  //     }

  //     *reinterpret_cast<uint8_t *>(buf) = from_queue[n];
  //   }
  // } else {
  //   for (size_t n = 0u; n < size / sizeof(uint16_t); n++) {
  //     if ((rc = xQueueReceive(SPI_fifos[2u], reinterpret_cast<uint16_t *const>(buf) + n, portIO_MAX_DELAY)) != pdPASS) {
  //       spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
  //       goto error;
  //     }
  //   }
  // }

  return 0;
error:
  return -1;
}

static int32_t spi_drv_SPI3_write(const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  for (uint32_t i = 0u; i < size; i++) {
    while (SPI_I2S_GetFlagStatus(spi_periph[2u], SPI_I2S_FLAG_BSY))
      ;
    SPI_I2S_SendData(spi_periph[2u], reinterpret_cast<const uint16_t *>(buf)[i]);
	
  }

  return 0;
error:
  return -1;
};

static int32_t spi_drv_SPI3_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static void SPI_irq_handler() {
  BaseType_t rc;
  portBASE_TYPE hp_task_woken;
  uint16_t *buf;

  for (uint32_t i = 0u; i < sizeof(spi_periph) / sizeof(spi_periph[0u]); i++) {
    if (SPI_I2S_GetITStatus(spi_periph[i], SPI_I2S_IT_RXNE)) {
      uint16_t recvd = SPI_I2S_ReceiveData(spi_periph[i]);

      if (SPI_on_recv_callbacks[i]) {
        buf = reinterpret_cast<uint16_t *>(malloc(sizeof(uint16_t)));
        *buf = SPI_I2S_ReceiveData(spi_periph[i]);
        typename events_worker_s::event_s event{.handler = SPI_on_recv_callbacks[i], .data = buf, .size = sizeof(*buf)};

        if ((rc = xQueueSendToBackFromISR(events_worker_queue, &event, &hp_task_woken)) != pdPASS) {
          goto exit;
        }

        if (hp_task_woken == pdTRUE) {
          taskYIELD();
        }
      }

      if ((rc = xQueueSendToBackFromISR(SPI_fifos[i], &recvd, &hp_task_woken)) != pdPASS) {
        goto exit;
      }
    } else if (SPI_I2S_GetITStatus(spi_periph[i], SPI_I2S_IT_TXE)) {
      if (SPI_on_send_callbacks[i]) {
        buf = reinterpret_cast<uint16_t *>(malloc(sizeof(uint16_t)));
        *buf = spi_periph[i]->DR;
        typename events_worker_s::event_s event{.handler = SPI_on_send_callbacks[i], .data = buf, .size = sizeof(*buf)};

        if ((rc = xQueueSendToBackFromISR(events_worker_queue, &event, &hp_task_woken)) != pdPASS) {
          goto exit;
        }

        if (hp_task_woken == pdTRUE) {
          taskYIELD();
        }
      }
    }
  }
exit:
  return;
}

static void SPI1_dma_irq_handler() {
  BaseType_t rc;
  portBASE_TYPE hp_task_woken;
  if (DMA_GetITStatus(DMA1_IT_TC3)) {

    DMA_ClearITPendingBit(SPI1_dma_irq_num);
    DMA_ClearFlag(DMA1_IT_GL3 | DMA1_IT_HT3 | DMA1_IT_TE3 | DMA1_IT_TC3);
    if ((rc = xSemaphoreGiveFromISR(SPI_dma_locks[0u], &hp_task_woken)) != pdPASS) {
      goto clear_irq_bits;
    }

    if (hp_task_woken == pdTRUE) {
      taskYIELD();
    }
  }

  return;
clear_irq_bits:
  DMA_ClearITPendingBit(SPI1_dma_irq_num);
  DMA_ClearFlag(DMA1_IT_GL3 | DMA1_IT_HT3 | DMA1_IT_TE3 | DMA1_IT_TC3);
}

static int32_t spi_drv_printf(const char *fmt, ...) {
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
    spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (strlen) {
    if ((usart_fd = ::open(usart, "usart1", 3, 3u)) < 0) {
      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, "[spi] : ", std::strlen("[spi] : "))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }
      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, temp, std::strlen(temp))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::close(usart, usart_fd)) < 0) {
      spi_drv_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
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
