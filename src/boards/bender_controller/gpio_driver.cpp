#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "arch/mcu_tags.hpp"
#include "boards/board_tags.hpp"
#include "drivers/gpio_driver.hpp"
#include "drivers/io/gpio_event.hpp"
#include "drivers/io/gpio_ioctl.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "fio/unistd.hpp"
#include "sys/events_worker.hpp"

/* This pointer will be used in interrupt handlers and will be initialized in driver init function */
static const struct drv_model_cmn_s *drv_ptr;
extern xQueueHandle events_worker_queue;

/* GPIOA lock */
static xSemaphoreHandle GPIOA_lock;

/* GPIOB lock */
static xSemaphoreHandle GPIOB_lock;

/* GPIOB lock */
static xSemaphoreHandle GPIOC_lock;

// Ports
static GPIO_TypeDef *const gpio_ports[]{GPIOA, GPIOB, GPIOC};

/* GPIO pin defenitions */
static const uint16_t gpio_pins[] = {GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_2,  GPIO_Pin_3,  GPIO_Pin_4,  GPIO_Pin_5,  GPIO_Pin_6,  GPIO_Pin_7,
                                     GPIO_Pin_8, GPIO_Pin_9, GPIO_Pin_10, GPIO_Pin_11, GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15};

static const uint16_t gpio_pin_sources[] = {GPIO_PinSource0, GPIO_PinSource1, GPIO_PinSource2,  GPIO_PinSource3,  GPIO_PinSource4,  GPIO_PinSource5,  GPIO_PinSource6,  GPIO_PinSource7,
                                            GPIO_PinSource8, GPIO_PinSource9, GPIO_PinSource10, GPIO_PinSource11, GPIO_PinSource12, GPIO_PinSource13, GPIO_PinSource14, GPIO_PinSource15};

const EXTITrigger_TypeDef exti_triggers[]{EXTI_Trigger_Rising, EXTI_Trigger_Falling, EXTI_Trigger_Rising_Falling};

const uint32_t exti_lines[]{EXTI_Line0,  EXTI_Line1,  EXTI_Line2,  EXTI_Line3,  EXTI_Line4,  EXTI_Line5,  EXTI_Line6,  EXTI_Line7,  EXTI_Line8,  EXTI_Line9,
                            EXTI_Line10, EXTI_Line11, EXTI_Line12, EXTI_Line13, EXTI_Line14, EXTI_Line15, EXTI_Line16, EXTI_Line17, EXTI_Line18, EXTI_Line19};

// Callback for each line
static void (*GPIO_EXTI_irq_callbacks[sizeof(exti_lines)])(const void *, size_t){nullptr};

/* GPIO driver file operations forward declaration */
static int32_t gpio_drv_GPIOA_open(int32_t, mode_t);
static int32_t gpio_drv_GPIOA_ioctl(uint64_t, const void *, size_t);
static int32_t gpio_drv_GPIOA_read(void *const, size_t);
static int32_t gpio_drv_GPIOA_write(const void *, size_t);
static int32_t gpio_drv_GPIOA_close();

static int32_t gpio_drv_GPIOB_open(int32_t, mode_t);
static int32_t gpio_drv_GPIOB_ioctl(uint64_t, const void *, size_t);
static int32_t gpio_drv_GPIOB_read(void *const, size_t);
static int32_t gpio_drv_GPIOB_write(const void *, size_t);
static int32_t gpio_drv_GPIOB_close();

static int32_t gpio_drv_GPIOC_open(int32_t, mode_t);
static int32_t gpio_drv_GPIOC_ioctl(uint64_t, const void *, size_t);
static int32_t gpio_drv_GPIOC_read(void *const, size_t);
static int32_t gpio_drv_GPIOC_write(const void *, size_t);
static int32_t gpio_drv_GPIOC_close();

static void GPIO_EXTI_IrqHandler();

/* GPIOA helper functions */
static int32_t GPIOA_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  BaseType_t rc;
  if (!(rc = xSemaphoreTakeRecursive(GPIOA_lock, portIO_MAX_DELAY))) {
    // errno = ???
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t GPIOA_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  BaseType_t rc;
  if (!(rc = xSemaphoreGiveRecursive(GPIOA_lock))) {
    // errno = ???
    goto error;
  }

  return 0;
error:
  return -1;
}

/* GPIOB helper functions */
static int32_t GPIOB_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  BaseType_t rc;
  if (!(rc = xSemaphoreTakeRecursive(GPIOB_lock, portIO_MAX_DELAY))) {
    // errno = ???
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t GPIOB_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  BaseType_t rc;
  if (!(rc = xSemaphoreGiveRecursive(GPIOB_lock))) {
    // errno = ???
    goto error;
  }

  return 0;
error:
  return -1;
}

/* GPIOC helper functions */
static int32_t GPIOC_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  BaseType_t rc;
  if (!(rc = xSemaphoreTakeRecursive(GPIOC_lock, portIO_MAX_DELAY))) {
    // errno = ???
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t GPIOC_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  BaseType_t rc;
  if (!(rc = xSemaphoreGiveRecursive(GPIOC_lock))) {
    // errno = ???
    goto error;
  }

  return 0;
error:
  return -1;
}

/* GPIO driver operations */
struct drv_ops_s gpio_drv_ops {
  .init = gpio_drv_init, .exit = gpio_drv_exit
};

/* GPIO driver file operations secification */
struct file_ops_s gpio_drv_GPIOA_fops {
  .flock = GPIOA_flock, .funlock = GPIOA_funlock, .open = gpio_drv_GPIOA_open, .ioctl = gpio_drv_GPIOA_ioctl, .read = gpio_drv_GPIOA_read, .write = gpio_drv_GPIOA_write, .close = gpio_drv_GPIOA_close
};

struct file_ops_s gpio_drv_GPIOB_fops {
  .flock = GPIOB_flock, .funlock = GPIOB_funlock, .open = gpio_drv_GPIOB_open, .ioctl = gpio_drv_GPIOB_ioctl, .read = gpio_drv_GPIOB_read, .write = gpio_drv_GPIOB_write, .close = gpio_drv_GPIOB_close
};

struct file_ops_s gpio_drv_GPIOC_fops {
  .flock = GPIOC_flock, .funlock = GPIOC_funlock, .open = gpio_drv_GPIOC_open, .ioctl = gpio_drv_GPIOC_ioctl, .read = gpio_drv_GPIOC_read, .write = gpio_drv_GPIOC_write, .close = gpio_drv_GPIOC_close
};

/* Driver operations */
void gpio_drv_init(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *rcc;
  struct rcc_drv_req_s rcc_req;
  int32_t rcc_fd, rc;

  /* Get rcc driver from dependencies */
  if (!(rcc = drv->dep("rcc"))) {
    // errno = ENOENT
    goto error;
  }

  /* Open RCC device */
  if ((rcc_fd = ::open(rcc, "rcc0", 3, 2u)) < 0) {
    // errno = ???
    goto error;
  }

  /* Enable GPIOA clocking */
  rcc_req = {rcc_bus_e::APB2, rcc_apb2_periph_e::RCC_GPIOA};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_ENABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    goto error;
  }

  /* Enable GPIOB clocking */
  rcc_req = {rcc_bus_e::APB2, rcc_apb2_periph_e::RCC_GPIOB};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_ENABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    goto error;
  }

  /* Enable GPIOC clocking */
  rcc_req = {rcc_bus_e::APB2, rcc_apb2_periph_e::RCC_GPIOC};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_ENABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    goto error;
  }

  /* Enable AFIO clocking */
  rcc_req = {rcc_bus_e::APB2, rcc_apb2_periph_e::RCC_AFIO};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_ENABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    goto error;
  }

  /* Close rcc file */
  if ((rc = ::close(rcc, rcc_fd)) < 0) {
    goto error;
  }

  // Disable JTAG mapping
  AFIO->MAPR = AFIO->MAPR | AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

  gpio_drv_GPIOA_fops.owner = drv;
  gpio_drv_GPIOB_fops.owner = drv;
  gpio_drv_GPIOC_fops.owner = drv;

  /* Init locks */
  GPIOA_lock = xSemaphoreCreateRecursiveMutex();
  GPIOB_lock = xSemaphoreCreateRecursiveMutex();
  GPIOC_lock = xSemaphoreCreateRecursiveMutex();

  /* Register char device for each GPIO port */
  drv->register_chardev("A", &gpio_drv_GPIOA_fops);
  drv->register_chardev("B", &gpio_drv_GPIOB_fops);
  drv->register_chardev("C", &gpio_drv_GPIOC_fops);

  /* Initialize GPIO driver pointer */
  drv_ptr = drv;
  return;

  /* Something went wrong -- reset drv_ptr and exit */
error:
  gpio_drv_exit(drv);
  return;
};

/* GPIO driver exit function */
void gpio_drv_exit(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *rcc;
  struct rcc_drv_req_s rcc_req;
  int32_t rcc_fd, rc;

  /* Get rcc driver from dependencies */
  if (!(rcc = drv->dep("rcc"))) {
    // errno = ENOENT
    goto error;
  }

  /* Open RCC device */
  if ((rcc_fd = ::open(rcc, "rcc0", 3, 2u)) < 0) {
    // errno = ???
    goto error;
  }

  /* Disable GPIOA clocking */
  rcc_req = {rcc_bus_e::APB2, rcc_apb2_periph_e::RCC_GPIOA};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_DISABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    goto error;
  }

  /* Disable GPIOB clocking */
  rcc_req = {rcc_bus_e::APB2, rcc_apb2_periph_e::RCC_GPIOB};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_DISABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    goto error;
  }

  /* Disable GPIOC clocking */
  rcc_req = {rcc_bus_e::APB2, rcc_apb2_periph_e::RCC_GPIOC};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_DISABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    goto error;
  }

  /* Disable AFIO clocking */
  rcc_req = {rcc_bus_e::APB2, rcc_apb2_periph_e::RCC_AFIO};
  if ((rc = ::ioctl(rcc, rcc_fd, rcc_drv_ioctl_cmd_e::RCC_DISABLE_CLOCK, &rcc_req, sizeof(rcc_req))) < 0) {
    // errno = ???
    goto error;
  }

  /* Close rcc file */
  if ((rc = ::close(rcc, rcc_fd)) < 0) {
    goto error;
  }

  /* Unregister char devices */
  drv->unregister_chardev("A");
  drv->unregister_chardev("B");
  drv->unregister_chardev("C");

  /* Remove locks */

  vSemaphoreDelete(GPIOA_lock);
  vSemaphoreDelete(GPIOB_lock);
  vSemaphoreDelete(GPIOC_lock);

  /* Reset driver ptr */
  drv_ptr = nullptr;

error:
  return;
};

/* File operations */
static int32_t gpio_drv_GPIOA_open(int32_t flags, mode_t mode) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
};

static int32_t gpio_drv_GPIOA_ioctl(uint64_t req, const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  switch (req) {
  case GPIO_SP_IFLOAT ... GPIO_SP_ANALOG_IN: {
    const struct gpio_setup_req_s *gpio_req = reinterpret_cast<const struct gpio_setup_req_s *>(buf);
    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = gpio_pins[gpio_req->pin];

    switch (req) {
      /* Set pin direction INPUT floating */
    case GPIO_SP_IFLOAT: {
      gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    } break;

      /* Set pin direction INPUT pulled down */
    case GPIO_SP_IPL: {
      gpio.GPIO_Mode = GPIO_Mode_IPD;
    } break;

      /* Set pin direction INPUT pulled up */
    case GPIO_SP_IPH: {
      gpio.GPIO_Mode = GPIO_Mode_IPU;
    } break;

      /* Set pin direction OUTPUT push - pull */
    case GPIO_SP_PP: {
      gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    } break;

      /* Set pin direction OUTPUT open - drain */
    case GPIO_SP_OD: {
      gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    } break;

      /* Set pin direction Alternative OUTPUT push - pull */
    case GPIO_SP_ALT_PP: {
      gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    } break;

      /* Set pin direction Alternative OUTPUT open - drain */
    case GPIO_SP_ALT_OD: {
      gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    } break;

      /* Set pin direction Analog INPUT */
    case GPIO_SP_ANALOG_IN: {
      gpio.GPIO_Mode = GPIO_Mode_AIN;
    } break;

    default:
      break;
    }

    GPIO_Init(GPIOA, &gpio);
  } break;

  case GPIO_PIN_WRITE: {
    const struct gpio_write_pin_req_s *write_req = reinterpret_cast<const struct gpio_write_pin_req_s *>(buf);
    write_req->val ? GPIO_SetBits(GPIOA, gpio_pins[write_req->pin]) : GPIO_ResetBits(GPIOA, gpio_pins[write_req->pin]);
  } break;

  case GPIO_PORT_WRITE: {
    const struct gpio_write_port_req_s *write_req = reinterpret_cast<const struct gpio_write_port_req_s *>(buf);
    GPIO_Write(GPIOA, write_req->val);
  } break;

  case GPIO_STROBE: {
    const struct gpio_strobe_req_s *strobe_req = reinterpret_cast<const struct gpio_strobe_req_s *>(buf);

    for (uint32_t i = 0u; i < strobe_req->n_times; i++) {
      GPIO_SetBits(GPIOA, gpio_pins[strobe_req->pin]);
      strobe_req->delay_fn(strobe_req->up_ticks);
      GPIO_ResetBits(GPIOA, gpio_pins[strobe_req->pin]);
      strobe_req->delay_fn(strobe_req->down_ticks);
    }
  } break;

  case GPIO_IRQ_ENABLE: {
    const struct gpio_irq_mgm_req_s *gpio_irq_req = reinterpret_cast<const struct gpio_irq_mgm_req_s *>(buf);
    EXTI_InitTypeDef exti;
    IRQn_Type irqn;

    EXTI_StructInit(&exti);
    exti.EXTI_Line = exti_lines[gpio_irq_req->exti_line];
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = exti_triggers[gpio_irq_req->trigger];
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    switch (gpio_irq_req->exti_line) {

    case 0: {
      g_pfnVectorsRam[EXTI0_IRQHandler_Vector] = GPIO_EXTI_IrqHandler;
      irqn = EXTI0_IRQn;
    } break;

    case 1: {
      g_pfnVectorsRam[EXTI1_IRQHandler_Vector] = GPIO_EXTI_IrqHandler;
      irqn = EXTI1_IRQn;
    } break;

    case 2: {
      g_pfnVectorsRam[EXTI2_IRQHandler_Vector] = GPIO_EXTI_IrqHandler;
      irqn = EXTI2_IRQn;
    } break;

    case 3: {
      g_pfnVectorsRam[EXTI3_IRQHandler_Vector] = GPIO_EXTI_IrqHandler;
      irqn = EXTI3_IRQn;
    } break;

    case 4: {
      g_pfnVectorsRam[EXTI4_IRQHandler_Vector] = GPIO_EXTI_IrqHandler;
      irqn = EXTI4_IRQn;
    } break;

    case 5 ... 9: {
      g_pfnVectorsRam[EXTI9_5_IRQHandler_Vector] = GPIO_EXTI_IrqHandler;
      irqn = EXTI9_5_IRQn;
    } break;

    case 10 ... 15: {
      g_pfnVectorsRam[EXTI15_10_IRQHandler_Vector] = GPIO_EXTI_IrqHandler;
      irqn = EXTI15_10_IRQn;
    } break;
    }

    GPIO_EXTI_irq_callbacks[gpio_irq_req->exti_line] = gpio_irq_req->callback;

    EXTI_ClearITPendingBit(exti_lines[gpio_irq_req->exti_line]);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, gpio_pin_sources[gpio_irq_req->exti_line]);
    NVIC_SetPriority(irqn, gpio_irq_req->priority);
    NVIC_EnableIRQ(irqn);
  } break;

  case GPIO_IRQ_DISABLE: {
    const struct gpio_irq_mgm_req_s *gpio_irq_req = reinterpret_cast<const struct gpio_irq_mgm_req_s *>(buf);
    EXTI_InitTypeDef exti;
    IRQn_Type irqn;

    EXTI_StructInit(&exti);
    exti.EXTI_Line = exti_lines[gpio_irq_req->exti_line];
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = exti_triggers[gpio_irq_req->trigger];
    exti.EXTI_LineCmd = DISABLE;
    EXTI_Init(&exti);

    switch (gpio_irq_req->exti_line) {

    case 0: {
      g_pfnVectorsRam[EXTI0_IRQHandler_Vector] = nullptr;
      irqn = EXTI0_IRQn;
    } break;

    case 1: {
      g_pfnVectorsRam[EXTI1_IRQHandler_Vector] = nullptr;
      irqn = EXTI1_IRQn;
    } break;

    case 2: {
      g_pfnVectorsRam[EXTI2_IRQHandler_Vector] = nullptr;
      irqn = EXTI2_IRQn;
    } break;

    case 3: {
      g_pfnVectorsRam[EXTI3_IRQHandler_Vector] = nullptr;
      irqn = EXTI3_IRQn;
    } break;

    case 4: {
      g_pfnVectorsRam[EXTI4_IRQHandler_Vector] = nullptr;
      irqn = EXTI4_IRQn;
    } break;

    case 5 ... 9: {
      g_pfnVectorsRam[EXTI9_5_IRQHandler_Vector] = nullptr;
      irqn = EXTI9_5_IRQn;
    } break;

    case 10 ... 15: {
      g_pfnVectorsRam[EXTI15_10_IRQHandler_Vector] = nullptr;
      irqn = EXTI15_10_IRQn;
    } break;
    }

    GPIO_EXTI_irq_callbacks[gpio_irq_req->exti_line] = nullptr;
    NVIC_DisableIRQ(irqn);
  } break;
  default:
    break;
  }

  return 0;

error:
  return -1;
};

static int32_t gpio_drv_GPIOA_read(void *const buf, size_t) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  *reinterpret_cast<uint16_t *>(buf) = GPIO_ReadInputData(GPIOA);
  // | GPIO_ReadOutputData(GPIOA);
  return sizeof(uint16_t);

error:
  return -1;
}

static int32_t gpio_drv_GPIOA_write(const void *buf, size_t) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  GPIO_Write(GPIOA, *reinterpret_cast<const uint16_t *>(buf));
  return sizeof(uint16_t);

error:
  return -1;
}

static int32_t gpio_drv_GPIOA_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;

error:
  return -1;
}

static int32_t gpio_drv_GPIOB_open(int32_t flags, mode_t mode) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t gpio_drv_GPIOB_ioctl(uint64_t req, const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  switch (req) {
  case GPIO_SP_IFLOAT ... GPIO_SP_ANALOG_IN: {
    const struct gpio_setup_req_s *gpio_req = reinterpret_cast<const struct gpio_setup_req_s *>(buf);
    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = gpio_pins[gpio_req->pin];

    switch (req) {
      /* Set pin direction INPUT floating */
    case GPIO_SP_IFLOAT: {
      gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    } break;

      /* Set pin direction INPUT pulled down */
    case GPIO_SP_IPL: {
      gpio.GPIO_Mode = GPIO_Mode_IPD;
    } break;

      /* Set pin direction INPUT pulled up */
    case GPIO_SP_IPH: {
      gpio.GPIO_Mode = GPIO_Mode_IPU;
    } break;

      /* Set pin direction OUTPUT push - pull */
    case GPIO_SP_PP: {
      gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    } break;

      /* Set pin direction OUTPUT open - drain */
    case GPIO_SP_OD: {
      gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    } break;

      /* Set pin direction Alternative OUTPUT push - pull */
    case GPIO_SP_ALT_PP: {
      gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    } break;

      /* Set pin direction Alternative OUTPUT open - drain */
    case GPIO_SP_ALT_OD: {
      gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    } break;

      /* Set pin direction Analog INPUT */
    case GPIO_SP_ANALOG_IN: {
      gpio.GPIO_Mode = GPIO_Mode_AIN;
    } break;

    default:
      break;
    }

    GPIO_Init(GPIOB, &gpio);
  } break;

  case GPIO_PIN_WRITE: {
    const struct gpio_write_pin_req_s *write_req = reinterpret_cast<const struct gpio_write_pin_req_s *>(buf);
    write_req->val ? GPIO_SetBits(GPIOB, gpio_pins[write_req->pin]) : GPIO_ResetBits(GPIOB, gpio_pins[write_req->pin]);
  } break;

  case GPIO_PORT_WRITE: {
    const struct gpio_write_port_req_s *write_req = reinterpret_cast<const struct gpio_write_port_req_s *>(buf);
    GPIO_Write(GPIOB, write_req->val);
  } break;

  case GPIO_STROBE: {
    const struct gpio_strobe_req_s *strobe_req = reinterpret_cast<const struct gpio_strobe_req_s *>(buf);

    for (uint32_t i = 0u; i < strobe_req->n_times; i++) {
      GPIO_SetBits(GPIOB, gpio_pins[strobe_req->pin]);
      strobe_req->delay_fn(strobe_req->up_ticks);
      GPIO_ResetBits(GPIOB, gpio_pins[strobe_req->pin]);
      strobe_req->delay_fn(strobe_req->down_ticks);
    }
  } break;

    /* Enable IRQ by number */
  case GPIO_IRQ_ENABLE: {
    const struct gpio_irq_mgm_req_s *gpio_irq_req = reinterpret_cast<const struct gpio_irq_mgm_req_s *>(buf);
    EXTI_InitTypeDef exti;
    IRQn_Type irqn;

    EXTI_StructInit(&exti);
    exti.EXTI_Line = exti_lines[gpio_irq_req->exti_line];
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = exti_triggers[gpio_irq_req->trigger];
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    switch (gpio_irq_req->exti_line) {

    case 0: {
      g_pfnVectorsRam[EXTI0_IRQHandler_Vector] = GPIO_EXTI_IrqHandler;
      irqn = EXTI0_IRQn;
    } break;

    case 1: {
      g_pfnVectorsRam[EXTI1_IRQHandler_Vector] = GPIO_EXTI_IrqHandler;
      irqn = EXTI1_IRQn;
    } break;

    case 2: {
      g_pfnVectorsRam[EXTI2_IRQHandler_Vector] = GPIO_EXTI_IrqHandler;
      irqn = EXTI2_IRQn;
    } break;

    case 3: {
      g_pfnVectorsRam[EXTI3_IRQHandler_Vector] = GPIO_EXTI_IrqHandler;
      irqn = EXTI3_IRQn;
    } break;

    case 4: {
      g_pfnVectorsRam[EXTI4_IRQHandler_Vector] = GPIO_EXTI_IrqHandler;
      irqn = EXTI4_IRQn;
    } break;

    case 5 ... 9: {
      g_pfnVectorsRam[EXTI9_5_IRQHandler_Vector] = GPIO_EXTI_IrqHandler;
      irqn = EXTI9_5_IRQn;
    } break;

    case 10 ... 15: {
      g_pfnVectorsRam[EXTI15_10_IRQHandler_Vector] = GPIO_EXTI_IrqHandler;
      irqn = EXTI15_10_IRQn;
    } break;
    }

    GPIO_EXTI_irq_callbacks[gpio_irq_req->exti_line] = gpio_irq_req->callback;
    EXTI_ClearITPendingBit(exti_lines[gpio_irq_req->exti_line]);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, gpio_pin_sources[gpio_irq_req->exti_line]);
    NVIC_SetPriority(irqn, gpio_irq_req->priority);
    NVIC_EnableIRQ(irqn);
  } break;

  case GPIO_IRQ_DISABLE: {
    const struct gpio_irq_mgm_req_s *gpio_irq_req = reinterpret_cast<const struct gpio_irq_mgm_req_s *>(buf);
    EXTI_InitTypeDef exti;
    IRQn_Type irqn;

    EXTI_StructInit(&exti);
    exti.EXTI_Line = exti_lines[gpio_irq_req->exti_line];
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = exti_triggers[gpio_irq_req->trigger];
    exti.EXTI_LineCmd = DISABLE;
    EXTI_Init(&exti);

    switch (gpio_irq_req->exti_line) {

    case 0: {
      g_pfnVectorsRam[EXTI0_IRQHandler_Vector] = nullptr;
      irqn = EXTI0_IRQn;
    } break;

    case 1: {
      g_pfnVectorsRam[EXTI1_IRQHandler_Vector] = nullptr;
      irqn = EXTI1_IRQn;
    } break;

    case 2: {
      g_pfnVectorsRam[EXTI2_IRQHandler_Vector] = nullptr;
      irqn = EXTI2_IRQn;
    } break;

    case 3: {
      g_pfnVectorsRam[EXTI3_IRQHandler_Vector] = nullptr;
      irqn = EXTI3_IRQn;
    } break;

    case 4: {
      g_pfnVectorsRam[EXTI4_IRQHandler_Vector] = nullptr;
      irqn = EXTI4_IRQn;
    } break;

    case 5 ... 9: {
      g_pfnVectorsRam[EXTI9_5_IRQHandler_Vector] = nullptr;
      irqn = EXTI9_5_IRQn;
    } break;

    case 10 ... 15: {
      g_pfnVectorsRam[EXTI15_10_IRQHandler_Vector] = nullptr;
      irqn = EXTI15_10_IRQn;
    } break;
    }

    GPIO_EXTI_irq_callbacks[gpio_irq_req->exti_line] = nullptr;
    NVIC_DisableIRQ(irqn);
  } break;

  default:
    break;
  }

  return 0;
error:
  return -1;
};

static int32_t gpio_drv_GPIOB_read(void *const buf, size_t) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  *reinterpret_cast<uint16_t *>(buf) = GPIO_ReadInputData(GPIOB);
  // | GPIO_ReadOutputData(GPIOB);
  return sizeof(uint16_t);
error:
  return -1;
}

static int32_t gpio_drv_GPIOB_write(const void *buf, size_t) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  GPIO_Write(GPIOB, *reinterpret_cast<const uint16_t *>(buf));
  return sizeof(uint16_t);
error:
  return -1;
}

static int32_t gpio_drv_GPIOB_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t gpio_drv_GPIOC_open(int32_t flags, mode_t mode) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t gpio_drv_GPIOC_ioctl(uint64_t req, const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  switch (req) {
  case GPIO_SP_IFLOAT ... GPIO_SP_ANALOG_IN: {
    GPIO_InitTypeDef gpio;
    const struct gpio_setup_req_s *gpio_req = reinterpret_cast<const struct gpio_setup_req_s *>(buf);
    GPIO_StructInit(&gpio);
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = gpio_pins[gpio_req->pin];

    switch (req) {
      /* Set pin direction INPUT floating */
    case GPIO_SP_IFLOAT: {
      gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    } break;

      /* Set pin direction INPUT pulled down */
    case GPIO_SP_IPL: {
      gpio.GPIO_Mode = GPIO_Mode_IPD;
    } break;

      /* Set pin direction INPUT pulled up */
    case GPIO_SP_IPH: {
      gpio.GPIO_Mode = GPIO_Mode_IPU;
    } break;

      /* Set pin direction OUTPUT push - pull */
    case GPIO_SP_PP: {
      gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    } break;

      /* Set pin direction OUTPUT open - drain */
    case GPIO_SP_OD: {
      gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    } break;

      /* Set pin direction Alternative OUTPUT push - pull */
    case GPIO_SP_ALT_PP: {
      gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    } break;

      /* Set pin direction Alternative OUTPUT open - drain */
    case GPIO_SP_ALT_OD: {
      gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    } break;

      /* Set pin direction Analog INPUT */
    case GPIO_SP_ANALOG_IN: {
      gpio.GPIO_Mode = GPIO_Mode_AIN;
    } break;

    default:
      break;
    }

    GPIO_Init(GPIOC, &gpio);
  } break;

  case GPIO_PIN_WRITE: {
    const struct gpio_write_pin_req_s *write_req = reinterpret_cast<const struct gpio_write_pin_req_s *>(buf);
    write_req->val ? GPIO_SetBits(GPIOC, gpio_pins[write_req->pin]) : GPIO_ResetBits(GPIOC, gpio_pins[write_req->pin]);
  } break;

  case GPIO_PORT_WRITE: {
    const struct gpio_write_port_req_s *write_req = reinterpret_cast<const struct gpio_write_port_req_s *>(buf);
    GPIO_Write(GPIOB, write_req->val);
  } break;

  case GPIO_STROBE: {
    const struct gpio_strobe_req_s *strobe_req = reinterpret_cast<const struct gpio_strobe_req_s *>(buf);

    for (uint32_t i = 0u; i < strobe_req->n_times; i++) {
      GPIO_SetBits(GPIOC, gpio_pins[strobe_req->pin]);
      strobe_req->delay_fn(strobe_req->up_ticks);
      GPIO_ResetBits(GPIOC, gpio_pins[strobe_req->pin]);
      strobe_req->delay_fn(strobe_req->down_ticks);
    }
  } break;

    /* Enable IRQ by number */
  case GPIO_IRQ_ENABLE: {
    const struct gpio_irq_mgm_req_s *gpio_irq_req = reinterpret_cast<const struct gpio_irq_mgm_req_s *>(buf);
    EXTI_InitTypeDef exti;
    IRQn_Type irqn;

    EXTI_StructInit(&exti);
    exti.EXTI_Line = exti_lines[gpio_irq_req->exti_line];
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = exti_triggers[gpio_irq_req->trigger];
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    switch (gpio_irq_req->exti_line) {

    case 0: {
      g_pfnVectorsRam[EXTI0_IRQHandler_Vector] = GPIO_EXTI_IrqHandler;
      irqn = EXTI0_IRQn;
    } break;

    case 1: {
      g_pfnVectorsRam[EXTI1_IRQHandler_Vector] = GPIO_EXTI_IrqHandler;
      irqn = EXTI1_IRQn;
    } break;

    case 2: {
      g_pfnVectorsRam[EXTI2_IRQHandler_Vector] = GPIO_EXTI_IrqHandler;
      irqn = EXTI2_IRQn;
    } break;

    case 3: {
      g_pfnVectorsRam[EXTI3_IRQHandler_Vector] = GPIO_EXTI_IrqHandler;
      irqn = EXTI3_IRQn;
    } break;

    case 4: {
      g_pfnVectorsRam[EXTI4_IRQHandler_Vector] = GPIO_EXTI_IrqHandler;
      irqn = EXTI4_IRQn;
    } break;

    case 5 ... 9: {
      g_pfnVectorsRam[EXTI9_5_IRQHandler_Vector] = GPIO_EXTI_IrqHandler;
      irqn = EXTI9_5_IRQn;
    } break;

    case 10 ... 15: {
      g_pfnVectorsRam[EXTI15_10_IRQHandler_Vector] = GPIO_EXTI_IrqHandler;
      irqn = EXTI15_10_IRQn;
    } break;
    }

    GPIO_EXTI_irq_callbacks[gpio_irq_req->exti_line] = gpio_irq_req->callback;
    EXTI_ClearITPendingBit(exti_lines[gpio_irq_req->exti_line]);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, gpio_pin_sources[gpio_irq_req->exti_line]);
    NVIC_SetPriority(irqn, gpio_irq_req->priority);
    NVIC_EnableIRQ(irqn);
  } break;

  case GPIO_IRQ_DISABLE: {
    const struct gpio_irq_mgm_req_s *gpio_irq_req = reinterpret_cast<const struct gpio_irq_mgm_req_s *>(buf);
    EXTI_InitTypeDef exti;
    IRQn_Type irqn;

    EXTI_StructInit(&exti);
    exti.EXTI_Line = exti_lines[gpio_irq_req->exti_line];
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = exti_triggers[gpio_irq_req->trigger];
    exti.EXTI_LineCmd = DISABLE;
    EXTI_Init(&exti);

    switch (gpio_irq_req->exti_line) {

    case 0: {
      g_pfnVectorsRam[EXTI0_IRQHandler_Vector] = nullptr;
      irqn = EXTI0_IRQn;
    } break;

    case 1: {
      g_pfnVectorsRam[EXTI1_IRQHandler_Vector] = nullptr;
      irqn = EXTI1_IRQn;
    } break;

    case 2: {
      g_pfnVectorsRam[EXTI2_IRQHandler_Vector] = nullptr;
      irqn = EXTI2_IRQn;
    } break;

    case 3: {
      g_pfnVectorsRam[EXTI3_IRQHandler_Vector] = nullptr;
      irqn = EXTI3_IRQn;
    } break;

    case 4: {
      g_pfnVectorsRam[EXTI4_IRQHandler_Vector] = nullptr;
      irqn = EXTI4_IRQn;
    } break;

    case 5 ... 9: {
      g_pfnVectorsRam[EXTI9_5_IRQHandler_Vector] = nullptr;
      irqn = EXTI9_5_IRQn;
    } break;

    case 10 ... 15: {
      g_pfnVectorsRam[EXTI15_10_IRQHandler_Vector] = nullptr;
      irqn = EXTI15_10_IRQn;
    } break;
    }

    GPIO_EXTI_irq_callbacks[gpio_irq_req->exti_line] = nullptr;
    NVIC_DisableIRQ(irqn);
  } break;

  default:
    break;
  }

  return 0;
error:
  return -1;
};

static int32_t gpio_drv_GPIOC_read(void *const buf, size_t) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  *reinterpret_cast<uint16_t *>(buf) = GPIO_ReadInputData(GPIOC);
  // | GPIO_ReadOutputData(GPIOC);
  return sizeof(uint16_t);
error:
  return -1;
}

static int32_t gpio_drv_GPIOC_write(const void *buf, size_t) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  GPIO_Write(GPIOC, *reinterpret_cast<const uint16_t *>(buf));
  return sizeof(uint16_t);
error:
  return -1;
}

static int32_t gpio_drv_GPIOC_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
};

static void GPIO_EXTI_IrqHandler() {
  BaseType_t rc;
  portBASE_TYPE hp_task_woken;
  typename events_worker_s::event_s event;
  struct gpio_event_s *gpio_event = reinterpret_cast<struct gpio_event_s *>(malloc(sizeof(struct gpio_event_s)));
  gpio_event->pin_val = reinterpret_cast<uint8_t *>(malloc(sizeof(uint8_t) * (sizeof(gpio_ports) / sizeof(gpio_ports[0u]))));
  gpio_event->val_num = sizeof(gpio_ports) / sizeof(gpio_ports[0u]);

  // Check all EXTI lines
  for (uint32_t line_no = 0u; line_no < sizeof(exti_lines) / sizeof(exti_lines[0u]); line_no++) {
    if (EXTI_GetITStatus(exti_lines[line_no])) {
      EXTI_ClearITPendingBit(exti_lines[line_no]);

      // Save line
      gpio_event->line = line_no;

      // Read GPIOx values from interrupted line
      for (uint32_t i = 0u; i < sizeof(gpio_ports) / sizeof(gpio_ports[0u]); i++) {
        gpio_event->pin_val[i] = GPIO_ReadInputDataBit(gpio_ports[i], gpio_pins[line_no]);
      }

      // Fill event
      event.data = gpio_event;
      event.size = sizeof(*gpio_event);
      event.handler = GPIO_EXTI_irq_callbacks[line_no];

      if ((rc = xQueueSendFromISR(events_worker_queue, &event, &hp_task_woken)) != pdPASS) {
        return;
      }

      if (hp_task_woken == pdTRUE) {
        taskYIELD();
      }
    }
  }
}
