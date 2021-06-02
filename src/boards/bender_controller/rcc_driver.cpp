#include <stdarg.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "arch/mcu_tags.hpp"
#include "boards/board_tags.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "drivers/rcc_driver.hpp"

static const struct drv_model_cmn_s *drv_ptr;

static uint32_t rcc_ahb_periph[]{RCC_AHBPeriph_CRC,  RCC_AHBPeriph_DMA1, RCC_AHBPeriph_DMA2, RCC_AHBPeriph_FLITF,
                                 RCC_AHBPeriph_FSMC, RCC_AHBPeriph_SDIO, RCC_AHBPeriph_SRAM};

static uint32_t rcc_apb1_periph[]{
    RCC_APB1Periph_BKP,   RCC_APB1Periph_CAN1,   RCC_APB1Periph_CAN2,   RCC_APB1Periph_CEC,  RCC_APB1Periph_DAC,
    RCC_APB1Periph_I2C1,  RCC_APB1Periph_I2C2,   RCC_APB1Periph_PWR,    RCC_APB1Periph_SPI2, RCC_APB1Periph_SPI3,
    RCC_APB1Periph_TIM12, RCC_APB1Periph_TIM13,  RCC_APB1Periph_TIM14,  RCC_APB1Periph_TIM2, RCC_APB1Periph_TIM3,
    RCC_APB1Periph_TIM4,  RCC_APB1Periph_TIM5,   RCC_APB1Periph_TIM6,   RCC_APB1Periph_TIM7, RCC_APB1Periph_UART4,
    RCC_APB1Periph_UART5, RCC_APB1Periph_USART2, RCC_APB1Periph_USART3, RCC_APB1Periph_USB,  RCC_APB1Periph_WWDG};

static uint32_t rcc_apb2_periph[]{
    RCC_APB2Periph_ADC1,  RCC_APB2Periph_ADC2,  RCC_APB2Periph_ADC3,  RCC_APB2Periph_AFIO,  RCC_APB2Periph_GPIOA,
    RCC_APB2Periph_GPIOB, RCC_APB2Periph_GPIOC, RCC_APB2Periph_GPIOD, RCC_APB2Periph_GPIOE, RCC_APB2Periph_GPIOF,
    RCC_APB2Periph_GPIOG, RCC_APB2Periph_SPI1,  RCC_APB2Periph_TIM1,  RCC_APB2Periph_TIM10, RCC_APB2Periph_TIM11,
    RCC_APB2Periph_TIM15, RCC_APB2Periph_TIM16, RCC_APB2Periph_TIM17, RCC_APB2Periph_TIM8,  RCC_APB2Periph_TIM9,
    RCC_APB2Periph_USART1};

/* RCC lock */
static xSemaphoreHandle rcc_lock;
extern xQueueHandle events_worker_queue;

/* BSP dependent file operations functions -- forward reference */
static int32_t rcc_drv_open(int32_t, mode_t);
static int32_t rcc_drv_ioctl(uint64_t, const void *, size_t);
static int32_t rcc_drv_read(void *const, size_t);
static int32_t rcc_drv_write(const void *, size_t);
static int32_t rcc_drv_close();

/* Helper functions */
static int32_t rcc_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  BaseType_t rc;
  if (!(rc = xSemaphoreTakeRecursive(rcc_lock, portIO_MAX_DELAY))) {
    // errno = ???
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t rcc_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  BaseType_t rc;
  if (!(rc = xSemaphoreGiveRecursive(rcc_lock))) {
    // errno = ???
    goto error;
  }

  return 0;
error:
  return -1;
}

struct drv_ops_s rcc_drv_ops {
  .init = rcc_drv_init, .exit = rcc_drv_exit
};

/* Driver file operations specification */
struct file_ops_s rcc_drv_fops {
  .flock = rcc_flock, .funlock = rcc_funlock, .open = rcc_drv_open, .ioctl = rcc_drv_ioctl, .read = rcc_drv_read,
  .write = rcc_drv_write, .close = rcc_drv_close
};

/* Driver operations */
void rcc_drv_init(const struct drv_model_cmn_s *drv) {
  RCC_DeInit();
  RCC_HSEConfig(RCC_HSE_ON);
  RCC_WaitForHSEStartUp();
  RCC_HSICmd(DISABLE);

  FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
  FLASH_SetLatency(FLASH_Latency_2);

  RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
  RCC_PLLCmd(ENABLE);

  while (!RCC_GetFlagStatus(RCC_FLAG_PLLRDY))
    ; /* Wait for PLL startup */

  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); /* Set SYSCLK clock source */

  while (RCC_GetSYSCLKSource() != 0x08)
    ; /* Wait for sysclk source */

  RCC_HCLKConfig(RCC_SYSCLK_Div1); /* Set HCLK clock source */
  RCC_PCLK1Config(RCC_HCLK_Div2);  /* Set PCLK1 clock source */
  RCC_PCLK2Config(RCC_HCLK_Div1);  /* Set PCLK2 clock source */

  SystemCoreClockUpdate();
  SysTick_Config(SystemCoreClock / 1000ul);

  rcc_drv_fops.owner = drv;
  rcc_lock = xSemaphoreCreateRecursiveMutex();
  drv->register_chardev("rcc0", &rcc_drv_fops);

  drv_ptr = drv;
  CoreDebug->DHCSR = CoreDebug->DHCSR & (~CoreDebug_DHCSR_C_DEBUGEN_Msk);
  return;

error:
  rcc_drv_exit(drv);
  return;
};

void rcc_drv_exit(const struct drv_model_cmn_s *drv) {
  RCC_DeInit();
  drv->unregister_chardev("rcc0");
  vSemaphoreDelete(rcc_lock);
  drv_ptr = nullptr;
};

static int32_t rcc_drv_open(int32_t, mode_t) { /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t rcc_drv_ioctl(uint64_t req, const void *buf, size_t size) {
  int32_t rc;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  switch (req) {
  case RCC_SET_SYSCLK_1MHZ: {
    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_OFF);
    RCC_HSICmd(ENABLE);

    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    FLASH_SetLatency(FLASH_Latency_0);
    RCC_PLLCmd(DISABLE);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI); /* Set SYSCLK clock source */

    while (RCC_GetSYSCLKSource() != RCC_CFGR_SWS_HSI)
      ; /* Wait for sysclk source */

    RCC_HCLKConfig(RCC_SYSCLK_Div8); /* Set HCLK clock source */
    RCC_PCLK1Config(RCC_HCLK_Div1);  /* Set PCLK1 clock source */
    RCC_PCLK2Config(RCC_HCLK_Div1);  /* Set PCLK2 clock source */

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000ul);
  } break;

  case RCC_SET_SYSCLK_72MHZ: {
    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON);
    RCC_WaitForHSEStartUp();
    RCC_HSICmd(DISABLE);

    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    FLASH_SetLatency(FLASH_Latency_2);

    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
    RCC_PLLCmd(ENABLE);

    while (!RCC_GetFlagStatus(RCC_FLAG_PLLRDY))
      ; /* Wait for PLL startup */

    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); /* Set SYSCLK clock source */

    while (RCC_GetSYSCLKSource() != 0x08)
      ; /* Wait for sysclk source */

    RCC_HCLKConfig(RCC_SYSCLK_Div1); /* Set HCLK clock source */
    RCC_PCLK1Config(RCC_HCLK_Div2);  /* Set PCLK1 clock source */
    RCC_PCLK2Config(RCC_HCLK_Div1);  /* Set PCLK2 clock source */

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000ul);
  } break;

  case RCC_ENABLE_CLOCK: {
    const struct rcc_drv_req_s *rcc_req = reinterpret_cast<const struct rcc_drv_req_s *>(buf);
    switch (rcc_req->bus) {

    case AHB: {
      RCC_AHBPeriphClockCmd(rcc_ahb_periph[rcc_req->periph], ENABLE);
      rc = 0;
    } break;

    case APB1: {
      RCC_APB1PeriphClockCmd(rcc_apb1_periph[rcc_req->periph], ENABLE);
      rc = 0;
    } break;

    case APB2: {
      RCC_APB2PeriphClockCmd(rcc_apb2_periph[rcc_req->periph], ENABLE);
      rc = 0;
    } break;
    default: {
      rc = -1;
    } break;
    }
  } break;

  case RCC_DISABLE_CLOCK: {
    const struct rcc_drv_req_s *rcc_req = reinterpret_cast<const struct rcc_drv_req_s *>(buf);
    switch (rcc_req->bus) {

    case AHB: {
      RCC_AHBPeriphClockCmd(rcc_ahb_periph[rcc_req->periph], DISABLE);
      rc = 0;
    } break;

    case APB1: {
      RCC_APB1PeriphClockCmd(rcc_apb1_periph[rcc_req->periph], DISABLE);
      rc = 0;
    } break;

    case APB2: {
      RCC_APB2PeriphClockCmd(rcc_apb2_periph[rcc_req->periph], DISABLE);
      rc = 0;
    } break;
    }
  } break;

  case RCC_REBOOT: {
    NVIC_SystemReset();
  } break;

  default: {
    rc = -1;
  } break;
  }

  return rc;
error:
  return -1;
}

static int32_t rcc_drv_read(void *const buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t rcc_drv_write(const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t rcc_drv_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    goto error;
  }

  return 0;
error:
  return -1;
}
