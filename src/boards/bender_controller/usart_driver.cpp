#include <cstdarg>
#include <vector>

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
extern xQueueHandle events_worker_queue;

/* RX & TX pins */
enum usart_device_e : uint32_t { USARTDEV1 = 0u, USARTDEV2, USARTDEV3, USARTDEV_MAX };
static constexpr const char *usart_gpio_port_letters[] = {"A", "A", "B"};
static constexpr const uint8_t usart_tx_pins[USARTDEV_MAX] = {9u, 2u, 10u};
static constexpr const uint8_t usart_rx_pins[USARTDEV_MAX] = {10u, 3u, 11u};
static constexpr enum isr_vectors_e usart_vectors[USARTDEV_MAX] = {USART1_IRQHandler_Vector, USART2_IRQHandler_Vector, USART3_IRQHandler_Vector};
static constexpr enum IRQn usart_irqs[USARTDEV_MAX] = {USART1_IRQn, USART2_IRQn, USART3_IRQn};

static constexpr const uint16_t usart_hw_low_ctrl[] = {USART_HardwareFlowControl_None, USART_HardwareFlowControl_RTS, USART_HardwareFlowControl_CTS, USART_HardwareFlowControl_RTS_CTS};
static constexpr const uint16_t usart_mode[] = {USART_Mode_Rx, USART_Mode_Tx, USART_Mode_Rx | USART_Mode_Tx};
static constexpr const uint16_t usart_parity[] = {USART_Parity_No, USART_Parity_Even, USART_Parity_Odd};
static constexpr const uint16_t usart_stop_bits[] = {USART_StopBits_0_5, USART_StopBits_1, USART_StopBits_1_5, USART_StopBits_2};
static constexpr const uint16_t usart_word_len[] = {USART_WordLength_8b, USART_WordLength_9b};

static USART_TypeDef *usart_devices[] = {USART1, USART2, USART3};
static constexpr const char *console_devstr = "usart1";
static constexpr const char *usart_dev_names[] = {"usart1", "usart2", "usart3"};

/* Buffer sizes */
static constexpr uint32_t usart_drv_rxd_buffer_size = 32u;
static constexpr uint32_t usart_drv_txd_buffer_size = 32u;

/* USART locks */
// static xSemaphoreHandle USART1_lock, USART1_dma_lock;
static xSemaphoreHandle usart_locks[USARTDEV_MAX], usart_dma_locks[USARTDEV_MAX];

/* USART fifos */
// static xQueueHandle USART1_fifo;
static xQueueHandle usart_fifos[USARTDEV_MAX];

// Printf to console
static int32_t usart_printf(const char *fmt, ...);

/* USART file IO functions forward reference */
template <enum usart_device_e device> static int32_t usart_drv_open(int32_t, mode_t);
template <enum usart_device_e device> static int32_t usart_drv_ioctl(uint64_t, const void *, size_t);
template <enum usart_device_e device> static int32_t usart_drv_read(void *const, size_t);
template <enum usart_device_e device> static int32_t usart_drv_write(const void *, size_t);
template <enum usart_device_e device> static int32_t usart_drv_close();

template <enum usart_device_e device> static void usart_irq_handler();
template <enum usart_device_e device> static void usart_dma_irq_handler();

static void (*usart_on_send_callbacks[USARTDEV_MAX])(const void *, size_t);
static void (*usart_on_recv_callbacks[USARTDEV_MAX])(const void *, size_t);

/* USART helper functions */
template <enum usart_device_e device> static int32_t usart_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreTakeRecursive(usart_locks[device], portIO_MAX_DELAY)) != pdPASS) {
    // errno = ???
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

template <enum usart_device_e device> static int32_t usart_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreGiveRecursive(usart_locks[device])) != pdPASS) {
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
  .flock = usart_flock<USARTDEV1>, .funlock = usart_funlock<USARTDEV1>, .open = usart_drv_open<USARTDEV1>, .ioctl = usart_drv_ioctl<USARTDEV1>, .read = usart_drv_read<USARTDEV1>,
  .write = usart_drv_write<USARTDEV1>, .close = usart_drv_close<USARTDEV1>
};

struct file_ops_s usart_drv_USART2_fops {
  .flock = usart_flock<USARTDEV2>, .funlock = usart_funlock<USARTDEV2>, .open = usart_drv_open<USARTDEV2>, .ioctl = usart_drv_ioctl<USARTDEV2>, .read = usart_drv_read<USARTDEV2>,
  .write = usart_drv_write<USARTDEV2>, .close = usart_drv_close<USARTDEV2>
};

struct file_ops_s usart_drv_USART3_fops {
  .flock = usart_flock<USARTDEV3>, .funlock = usart_funlock<USARTDEV3>, .open = usart_drv_open<USARTDEV3>, .ioctl = usart_drv_ioctl<USARTDEV3>, .read = usart_drv_read<USARTDEV3>,
  .write = usart_drv_write<USARTDEV3>, .close = usart_drv_close<USARTDEV3>
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

  /* Enable USART1 clocking */
  for (const struct rcc_drv_req_s &req :
       {rcc_drv_req_s{rcc_bus_e::APB2, static_cast<uint32_t>(rcc_apb2_periph_e::RCC_USART1)}, rcc_drv_req_s{rcc_bus_e::APB1, static_cast<uint32_t>(rcc_apb1_periph_e::RCC_USART2)},
        rcc_drv_req_s{rcc_bus_e::APB1, static_cast<uint32_t>(rcc_apb1_periph_e::RCC_USART3)}}) {

    rcc_req = {.bus = req.bus, .periph = req.periph};
    if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_ENABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
      // errno = ???
      usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
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

  for (const usart_device_e &usartdev : {USARTDEV1, USARTDEV2, USARTDEV3}) {
    /* Open GPIO device */
    if ((gpio_fd = ::open(gpio, usart_gpio_port_letters[usartdev], 3, 2u)) < 0) {
      // errno = ???
      usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    gpio_req = {.pin = usart_tx_pins[usartdev]};
    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_ALT_PP, &gpio_req, sizeof(rcc_req))) < 0) {
      // errno = ???
      usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    gpio_req = {.pin = usart_rx_pins[usartdev]};
    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IPH, &gpio_req, sizeof(rcc_req))) < 0) {
      // errno = ???
      usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    /* Close gpio file */
    if ((rc = ::close(gpio, gpio_fd)) < 0) {
      usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  usart_drv_USART1_fops.owner = drv;
  usart_drv_USART2_fops.owner = drv;
  usart_drv_USART3_fops.owner = drv;

  /* Init locks */
  for (xSemaphoreHandle &lock : usart_locks) {
    lock = xSemaphoreCreateRecursiveMutex();
  }

  for (xSemaphoreHandle &dma_lock : usart_dma_locks) {
    dma_lock = xSemaphoreCreateBinary();
    xSemaphoreGive(dma_lock);
  }

  for (xQueueHandle &fifo : usart_fifos) {
    fifo = xQueueCreate(usart_drv_rxd_buffer_size, sizeof(char));
  }

  /* Register char device for each GPIO port */
  drv->register_chardev("usart1", &usart_drv_USART1_fops);
  drv->register_chardev("usart2", &usart_drv_USART2_fops);
  drv->register_chardev("usart3", &usart_drv_USART3_fops);

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
  for (const struct rcc_drv_req_s &req :
       {rcc_drv_req_s{rcc_bus_e::APB2, static_cast<uint32_t>(rcc_apb2_periph_e::RCC_USART1)}, rcc_drv_req_s{rcc_bus_e::APB1, static_cast<uint32_t>(rcc_apb1_periph_e::RCC_USART2)},
        rcc_drv_req_s{rcc_bus_e::APB1, static_cast<uint32_t>(rcc_apb1_periph_e::RCC_USART3)}}) {

    rcc_req = {.bus = req.bus, .periph = req.periph};
    if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_DISABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
      // errno = ???
      usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
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

  for (const usart_device_e &usartdev : {USARTDEV1, USARTDEV2, USARTDEV3}) {
    /* Open GPIO device */
    if ((gpio_fd = ::open(gpio, usart_gpio_port_letters[usartdev], 3, 2u)) < 0) {
      // errno = ???
      usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    gpio_req = {.pin = usart_tx_pins[usartdev]};
    if ((rc = ::ioctl(gpio, gpio_fd, gpio_ioctl_cmd_e::GPIO_SP_IFLOAT, &gpio_req, sizeof(rcc_req))) < 0) {
      // errno = ???
      usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    gpio_req = {.pin = usart_rx_pins[usartdev]};
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
  }

  usart_drv_USART1_fops.owner = nullptr;
  usart_drv_USART2_fops.owner = nullptr;
  usart_drv_USART3_fops.owner = nullptr;

  for (const usart_device_e &usartdev : {USARTDEV1, USARTDEV2, USARTDEV3}) {

    /* Remove locks */
    vSemaphoreDelete(usart_locks[usartdev]);
    vSemaphoreDelete(usart_dma_locks[usartdev]);
    vQueueDelete(usart_fifos[usartdev]);

    /* Unregister char device for each GPIO port */
    drv->unregister_chardev(usart_dev_names[usartdev]);
  }

  /* Initialize GPIO driver pointer */
  drv_ptr = nullptr;
  return;
error:
  return;
}

template <enum usart_device_e device> static int32_t usart_drv_open(int32_t, mode_t) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

template <enum usart_device_e device> static int32_t usart_drv_ioctl(uint64_t req, const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (req) {

  case USART_INIT: {
    const struct usart_setup_req_s *usart_req = reinterpret_cast<const struct usart_setup_req_s *>(buf);

    // Remap USART1 pins
    // if constexpr (device == USARTDEV1) {
    //   GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
    // }

    USART_InitTypeDef usart;
    usart.USART_BaudRate = usart_req->baudrate;
    usart.USART_HardwareFlowControl = usart_hw_low_ctrl[usart_req->hw_flow_ctrl];
    usart.USART_Mode = usart_mode[usart_req->mode];
    usart.USART_Parity = usart_parity[usart_req->parity];
    usart.USART_StopBits = usart_stop_bits[usart_req->sb];
    usart.USART_WordLength = usart_word_len[usart_req->wl];
    USART_Init(usart_devices[device], &usart);

    USART_ITConfig(usart_devices[device], USART_IT_RXNE, ENABLE);

    // Enable DMA on USART1
    if constexpr (device == USARTDEV1) {
      DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
    }

    USART_Cmd(usart_devices[device], ENABLE);

    // Enable DMA interrupt on USART1
    if constexpr (device == USARTDEV1) {
      g_pfnVectorsRam[DMA1_Channel4_IRQHandler_Vector] = usart_dma_irq_handler<device>;
    }

    g_pfnVectorsRam[usart_vectors[device]] = usart_irq_handler<device>;

    NVIC_SetPriority(usart_irqs[device], usart_req->irq_priority);
    NVIC_EnableIRQ(usart_irqs[device]);

    // Enable DMA irq on USART1
    if constexpr (device == USARTDEV1) {
      NVIC_SetPriority(DMA1_Channel4_IRQn, usart_req->irq_priority);
      NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    }

  } break;

  case USART_DEINIT: {
    const struct usart_setup_req_s *usart_req = reinterpret_cast<const struct usart_setup_req_s *>(buf);
    USART_InitTypeDef usart;

    // if constexpr (device == USARTDEV1) {
    //   GPIO_PinRemapConfig(GPIO_Remap_USART1, DISABLE);
    // }

    USART_Cmd(usart_devices[device], DISABLE);
    USART_DeInit(usart_devices[device]);
    USART_ITConfig(usart_devices[device], USART_IT_RXNE, DISABLE);

    if constexpr (device == USARTDEV1) {
      DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, DISABLE);
      NVIC_DisableIRQ(DMA1_Channel4_IRQn);
      g_pfnVectorsRam[DMA1_Channel4_IRQHandler_Vector] = nullptr;
    }

    NVIC_DisableIRQ(usart_irqs[device]);
    g_pfnVectorsRam[usart_vectors[device]] = nullptr;
  } break;

  case USART_ON_RECV: {
    const struct usart_callback_req_s *usart_req = reinterpret_cast<const struct usart_callback_req_s *>(buf);
    usart_on_recv_callbacks[device] = usart_req->callback;
  } break;

  case USART_ON_SEND: {
    const struct usart_callback_req_s *usart_req = reinterpret_cast<const struct usart_callback_req_s *>(buf);
    usart_on_send_callbacks[device] = usart_req->callback;
  } break;
  }

  return 0;
error:
  return -1;
}

template <enum usart_device_e device> static int32_t usart_drv_read(void *const buf, size_t size) {
  BaseType_t rc;

  for (size_t nbyte = 0u; nbyte < size; nbyte++) {
    if ((rc = xQueueReceive(usart_fifos[device], reinterpret_cast<char *const>(buf) + nbyte, portIO_MAX_DELAY)) != pdPASS) {
      usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  return 0;
error:
  return -1;
}

template <enum usart_device_e device> static int32_t usart_drv_write(const void *buf, size_t size) {
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
    while (!USART_GetFlagStatus(usart_devices[device], USART_FLAG_TC))
      ;
    USART_SendData(usart_devices[device], static_cast<const uint8_t *>(buf)[i]);
  }

  return size;
error:
  return -1;
}

template <enum usart_device_e device> static int32_t usart_drv_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    usart_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

template <enum usart_device_e device> static void usart_irq_handler() {
  BaseType_t rc;
  portBASE_TYPE hp_task_woken;
  char *buf, recvd;

  if (USART_GetITStatus(usart_devices[device], USART_IT_RXNE)) {
    recvd = USART_ReceiveData(usart_devices[device]);
    if (usart_on_recv_callbacks[device]) {
      buf = reinterpret_cast<char *>(malloc(sizeof(char)));
      *buf = recvd;
      typename events_worker_s::event_s event{.handler = usart_on_recv_callbacks[device], .data = buf, .size = sizeof(char)};

      if ((rc = xQueueSendToBackFromISR(events_worker_queue, &event, &hp_task_woken)) != pdPASS) {
        goto exit;
      }

      if (hp_task_woken == pdTRUE) {
        taskYIELD();
      }
    }

    if ((rc = xQueueSendToBackFromISR(usart_fifos[device], &recvd, &hp_task_woken)) != pdPASS) {
      goto exit;
    }
  } else if (USART_GetITStatus(usart_devices[device], USART_IT_TC)) {
    if (usart_on_send_callbacks[device]) {
      buf = reinterpret_cast<char *>(malloc(sizeof(char)));
      *buf = usart_devices[device]->DR;
      typename events_worker_s::event_s event{.handler = usart_on_send_callbacks[device], .data = buf, .size = sizeof(char)};

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

template <enum usart_device_e device> static void usart_dma_irq_handler() {
  BaseType_t rc;
  portBASE_TYPE hp_task_woken;
  if (DMA_GetITStatus(DMA1_IT_TC4)) {

    DMA_ClearITPendingBit(DMA1_Channel4_IRQn);
    DMA_ClearFlag(DMA1_IT_GL4 | DMA1_IT_HT4 | DMA1_IT_TE4 | DMA1_IT_TC4);
    if ((rc = xSemaphoreGiveFromISR(usart_dma_locks[device], &hp_task_woken)) != pdPASS) {
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
