#include <cstdarg>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "arch/mcu_tags.hpp"
#include "boards/board_tags.hpp"
#include "drivers/io/gpio_ioctl.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "drivers/io/usart_ioctl.hpp"
#include "drivers/usart_driver.hpp"
#include "fio/unistd.hpp"
#include "sys/events_worker.hpp"

/* This pointer will be used in interrupt handlers and will be initialized in driver init function */
static const struct drv_model_cmn_s *drv_ptr;
extern bool debug_log_enabled;

/* RX & TX pins */
static constexpr uint8_t USART1_rx_pin_no = 7u, USART1_tx_pin_no = 6u;
static constexpr const char *usart_gpio_port_letter = "B";
/* Buffer sizes */
static constexpr uint32_t USART1_drv_rxd_buffer_size = 32u;
static constexpr uint32_t USART1_drv_txd_buffer_size = 32u;

/* USART1 lock */
static xSemaphoreHandle USART1_lock, USART1_dma_lock;
extern xQueueHandle events_worker_queue;
static xQueueHandle USART1_fifo;

// Printf to console
static int32_t usart_printf(const char *fmt, ...);

/* USART1 file IO functions forward reference */
static int32_t usart_drv_USART1_open(int32_t, mode_t);
static int32_t usart_drv_USART1_ioctl(uint64_t, const void *, size_t);
static int32_t usart_drv_USART1_read(void *const, size_t);
static int32_t usart_drv_USART1_write(const void *, size_t);
static int32_t usart_drv_USART1_close();

static void USART1_irq_handler();
static void USART1_dma_irq_handler();

static void (*USART1_on_send_callback)(const void *, size_t);
static void (*USART1_on_recv_callback)(const void *, size_t);

/* USART1 helper functions */
static int32_t USART1_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreTakeRecursive(USART1_lock, portIO_MAX_DELAY)) != pdPASS) {
    // errno = ???
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t USART1_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreGiveRecursive(USART1_lock)) != pdPASS) {
    // errno = ???
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

struct drv_ops_s usart_drv_ops {
  .init = usart_drv_init, .exit = usart_drv_exit
};

/* USART driver file operations secification */
struct file_ops_s usart_drv_USART1_fops {
  .flock = USART1_flock, .funlock = USART1_funlock, .open = usart_drv_USART1_open, .ioctl = usart_drv_USART1_ioctl, .read = usart_drv_USART1_read, .write = usart_drv_USART1_write,
  .close = usart_drv_USART1_close
};

void usart_drv_init(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *rcc, *gpio;
  struct rcc_drv_req_s rcc_req;
  struct gpio_setup_req_s gpio_req;
  int32_t rcc_fd, gpio_fd, rc;

  /* Get rcc driver from dependencies */
  if (!(rcc = drv->dep("rcc"))) {
    // errno = ENOENT
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Open RCC device */
  if ((rcc_fd = ::open(rcc, "rcc0", 3, 2u)) < 0) {
    // errno = ???
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Enable GPIOC clocking */
  rcc_req = {rcc_bus_e::APB2, rcc_apb2_periph_e::RCC_USART1};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_ENABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Enable DMA1 clocking */
  rcc_req = {rcc_bus_e::AHB, rcc_ahb_periph_e::RCC_DMA1};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_ENABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Close rcc file */
  if ((rc = ::close(rcc, rcc_fd)) < 0) {
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Get rcc driver from dependencies */
  if (!(gpio = drv->dep("gpio"))) {
    // errno = ENOENT
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Open GPIO device */
  if ((gpio_fd = ::open(gpio, usart_gpio_port_letter, 3, 2u)) < 0) {
    // errno = ???
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  gpio_req = {.pin = USART1_tx_pin_no};
  if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_ALT_PP, &gpio_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  gpio_req = {.pin = USART1_rx_pin_no};
  if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IPH, &gpio_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Close rcc file */
  if ((rc = ::close(gpio, gpio_fd)) < 0) {
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  usart_drv_USART1_fops.owner = drv;

  /* Init locks */
  USART1_lock = xSemaphoreCreateRecursiveMutex();
  USART1_dma_lock = xSemaphoreCreateBinary();
  USART1_fifo = xQueueCreate(USART1_drv_rxd_buffer_size, sizeof(char));

  xSemaphoreGive(USART1_dma_lock);

  /* Register char device for each GPIO port */
  drv->register_chardev("usart1", &usart_drv_USART1_fops);

  /* Initialize GPIO driver pointer */
  drv_ptr = drv;
  return;

  /* Something went wrong -- reset drv_ptr and exit */
error:
  usart_drv_exit(drv);
  return;
}

void usart_drv_exit(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *rcc, *gpio;
  struct rcc_drv_req_s rcc_req;
  struct gpio_setup_req_s gpio_req;
  int32_t rcc_fd, gpio_fd, rc;

  /* Get rcc driver from dependencies */
  if (!(rcc = drv->dep("rcc"))) {
    // errno = ENOENT
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Open RCC device */
  if ((rcc_fd = ::open(rcc, "rcc0", 3, 2u)) < 0) {
    // errno = ???
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Disable USART clocking */
  rcc_req = {rcc_bus_e::APB2, rcc_apb2_periph_e::RCC_USART1};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_DISABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Disable DMA1 clocking */
  rcc_req = {rcc_bus_e::AHB, rcc_ahb_periph_e::RCC_DMA1};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_DISABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Close rcc file */
  if ((rc = ::close(rcc, rcc_fd)) < 0) {
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Get rcc driver from dependencies */
  if (!(gpio = drv->dep("gpio"))) {
    // errno = ENOENT
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Open GPIO device */
  if ((gpio_fd = ::open(gpio, usart_gpio_port_letter, 3, 2u)) < 0) {
    // errno = ???
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  gpio_req = {.pin = USART1_tx_pin_no};
  if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  gpio_req = {.pin = USART1_rx_pin_no};
  if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  /* Close gpio file */
  if ((rc = ::close(gpio, gpio_fd)) < 0) {
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  usart_drv_USART1_fops.owner = nullptr;

  /* Remove locks */
  vSemaphoreDelete(USART1_lock);
  vSemaphoreDelete(USART1_dma_lock);
  vQueueDelete(USART1_fifo);

  /* Register char device for each GPIO port */
  drv->unregister_chardev("usart1");

  /* Initialize GPIO driver pointer */
  drv_ptr = nullptr;
  return;
error:
  return;
}

static int32_t usart_drv_USART1_open(int32_t, mode_t) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t usart_drv_USART1_ioctl(uint64_t req, const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (req) {

  case USART_INIT: {
    const struct usart_setup_req_s *usart_req = reinterpret_cast<const struct usart_setup_req_s *>(buf);
    GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

    USART_InitTypeDef usart;
    usart.USART_BaudRate = usart_req->baudrate;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &usart);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
    USART_Cmd(USART1, ENABLE);

    g_pfnVectorsRam[DMA1_Channel4_IRQHandler_Vector] = USART1_dma_irq_handler;
    g_pfnVectorsRam[USART1_IRQHandler_Vector] = USART1_irq_handler;

    NVIC_SetPriority(USART1_IRQn, usart_req->irq_priority);
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(DMA1_Channel4_IRQn, usart_req->irq_priority);
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);

  } break;

  case USART_DEINIT: {
    const struct usart_setup_req_s *usart_req = reinterpret_cast<const struct usart_setup_req_s *>(buf);
    USART_InitTypeDef usart;
	GPIO_PinRemapConfig(GPIO_Remap_USART1, DISABLE);
    USART_Cmd(USART1, DISABLE);
    USART_DeInit(USART1);
    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, DISABLE);
    NVIC_DisableIRQ(USART1_IRQn);
    NVIC_DisableIRQ(DMA1_Channel4_IRQn);

    g_pfnVectorsRam[DMA1_Channel4_IRQHandler_Vector] = nullptr;
    g_pfnVectorsRam[USART1_IRQHandler_Vector] = nullptr;
  } break;

  case USART_ON_RECV: {
    const struct usart_callback_req_s *usart_req = reinterpret_cast<const struct usart_callback_req_s *>(buf);
    USART1_on_recv_callback = usart_req->callback;
  } break;

  case USART_ON_SEND: {
    const struct usart_callback_req_s *usart_req = reinterpret_cast<const struct usart_callback_req_s *>(buf);
    USART1_on_send_callback = usart_req->callback;
  } break;
  }

  return 0;
error:
  return -1;
}

static int32_t usart_drv_USART1_read(void *const buf, size_t size) {
  BaseType_t rc;

  for (size_t nbyte = 0u; nbyte < size; nbyte++) {
    if ((rc = xQueueReceive(USART1_fifo, reinterpret_cast<char *const>(buf) + nbyte, portIO_MAX_DELAY)) != pdPASS) {
      usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  return 0;
error:
  return -1;
}

static int32_t usart_drv_USART1_write(const void *buf, size_t size) {
  // BaseType_t rc;
  // DMA_InitTypeDef dma;

  // DMA_StructInit(&dma);
  // dma.DMA_PeripheralBaseAddr = reinterpret_cast<size_t>(&USART1->DR);
  // dma.DMA_MemoryBaseAddr = reinterpret_cast<const size_t>(buf);
  // dma.DMA_BufferSize = size;
  // dma.DMA_DIR = DMA_DIR_PeripheralDST;
  // dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  // dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  // dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
  // dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;

  // if ((rc = xSemaphoreTake(USART1_dma_lock, portIO_MAX_DELAY * 20)) != pdPASS) {
  //   DMA_DeInit(DMA1_Channel4);
  //   DMA_Cmd(DMA1_Channel4, DISABLE);
  //   usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);goto error;
  // }

  // DMA_DeInit(DMA1_Channel4);
  // DMA_Cmd(DMA1_Channel4, DISABLE);
  // DMA_Init(DMA1_Channel4, &dma);
  // DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
  // USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
  // DMA_Cmd(DMA1_Channel4, ENABLE);

  for (uint32_t i = 0u; i < size; i++) {
    while (!USART_GetFlagStatus(USART1, USART_FLAG_TC))
      ;
    USART_SendData(USART1, static_cast<const char *>(buf)[i]);
  }

  return size;
error:
  return -1;
}

static int32_t usart_drv_USART1_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static void USART1_irq_handler() {
  BaseType_t rc;
  portBASE_TYPE hp_task_woken;
  char *buf, recvd;

  if (USART_GetITStatus(USART1, USART_IT_RXNE)) {
    recvd = USART_ReceiveData(USART1);
    if (USART1_on_recv_callback) {
      buf = reinterpret_cast<char *>(malloc(sizeof(char)));
      *buf = recvd;
      typename events_worker_s::event_s event{.handler = USART1_on_recv_callback, .data = buf, .size = sizeof(char)};

      if ((rc = xQueueSendToBackFromISR(events_worker_queue, &event, &hp_task_woken)) != pdPASS) {
        goto exit;
      }

      if (hp_task_woken == pdTRUE) {
        taskYIELD();
      }
    }

    if ((rc = xQueueSendToBackFromISR(USART1_fifo, &recvd, &hp_task_woken)) != pdPASS) {
      goto exit;
    }
  } else if (USART_GetITStatus(USART1, USART_IT_TC)) {
    if (USART1_on_send_callback) {
      buf = reinterpret_cast<char *>(malloc(sizeof(char)));
      *buf = USART1->DR;
      typename events_worker_s::event_s event{.handler = USART1_on_send_callback, .data = buf, .size = sizeof(char)};

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

static void USART1_dma_irq_handler() {
  BaseType_t rc;
  portBASE_TYPE hp_task_woken;
  if (DMA_GetITStatus(DMA1_IT_TC4)) {

    DMA_ClearITPendingBit(DMA1_Channel4_IRQn);
    DMA_ClearFlag(DMA1_IT_GL4 | DMA1_IT_HT4 | DMA1_IT_TE4 | DMA1_IT_TC4);
    if ((rc = xSemaphoreGiveFromISR(USART1_dma_lock, &hp_task_woken)) != pdPASS) {
      goto clear_irq_bits;
    }

    if (hp_task_woken == pdTRUE) {
      taskYIELD();
    }
  }

  return;
clear_irq_bits:
  DMA_ClearITPendingBit(DMA1_Channel4_IRQn);
  DMA_ClearFlag(DMA1_IT_GL4 | DMA1_IT_HT4 | DMA1_IT_TE4 | DMA1_IT_TC4);
}

static int32_t usart_printf(const char *fmt, ...) {
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
    if ((usart_fd = ::open(usart, "usart1", 3, 3u)) < 0) {
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
