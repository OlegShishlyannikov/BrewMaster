#ifndef RCC_HPP
#define RCC_HPP

#include <cstdint>

/* RCC driver ioctl requests */
enum rcc_drv_ioctl_cmd_e : uint64_t { RCC_SET_SYSCLK_1MHZ = 0u, RCC_SET_SYSCLK_72MHZ, RCC_ENABLE_CLOCK, RCC_DISABLE_CLOCK, RCC_REBOOT };

/* Bus enumeration */
enum rcc_bus_e : uint32_t { AHB = 0u, APB1, APB2 };

/* Periphery enumeration */
enum rcc_ahb_periph_e : uint32_t { RCC_CRC = 0u, RCC_DMA1, RCC_DMA2, RCC_FLITF, RCC_FSMC, RCC_SDIO, RCC_SRAM };

enum rcc_apb1_periph_e : uint32_t {
  RCC_BKP = 0u,
  RCC_CAN1,
  RCC_CAN2,
  RCC_CEC,
  RCC_DAC,
  RCC_I2C1,
  RCC_I2C2,
  RCC_PWR,
  RCC_SPI2,
  RCC_SPI3,
  RCC_TIM12,
  RCC_TIM13,
  RCC_TIM14,
  RCC_TIM2,
  RCC_TIM3,
  RCC_TIM4,
  RCC_TIM5,
  RCC_TIM6,
  RCC_TIM7,
  RCC_UART4,
  RCC_UART5,
  RCC_USART2,
  RCC_USART3,
  RCC_USB,
  RCC_WWDG
};

enum rcc_apb2_periph_e : uint32_t {
  RCC_ADC1 = 0u,
  RCC_ADC2,
  RCC_ADC3,
  RCC_AFIO,
  RCC_GPIOA,
  RCC_GPIOB,
  RCC_GPIOC,
  RCC_GPIOD,
  RCC_GPIOE,
  RCC_GPIOF,
  RCC_GPIOG,
  RCC_SPI1,
  RCC_TIM1,
  RCC_TIM10,
  RCC_TIM11,
  RCC_TIM15,
  RCC_TIM16,
  RCC_TIM17,
  RCC_TIM8,
  RCC_TIM9,
  RCC_USART1
};

struct rcc_drv_req_s {
  enum rcc_bus_e bus;
  uint32_t periph;
};

#endif /* RCC_HPP */
