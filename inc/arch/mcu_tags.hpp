#ifndef MCU_TAG_HPP
#define MCU_TAG_HPP
#pragma GCC diagnostic ignored "-Wregister"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <sys/types.h>

struct mcu_tag_s {};

/* Defined from ld-script */
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t _estack;

struct __attribute__((packed)) context_state_frame_s {
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  uint32_t return_address;
  uint32_t xpsr;
  uint32_t bfar;
  uint32_t cfsr;
  uint32_t hfsr;
  uint32_t dfsr;
  uint32_t afsr;
};

/* Hardware exception handlers */
extern "C" void __attribute__((used)) __hard_fault_handler(context_state_frame_s *frame);
extern "C" void __attribute__((used)) __usage_fault_handler(context_state_frame_s *frame);
extern "C" void __attribute__((used)) __bus_fault_handler(context_state_frame_s *frame);
extern "C" void __attribute__((used)) __mem_manage_handler(context_state_frame_s *frame);
extern "C" void __attribute__((used)) __nmi_handler(context_state_frame_s *frame);

/* Entry point */
void _start(void) __attribute__((noreturn));

#ifdef STM32F10X_XL
#  include "stm32f10x_conf.h"
#  define LOOP() while (true)

enum isr_vectors_e : uint32_t {
  ESTACK = 0u,
  Reset_Handler_Vector,
  NMI_Handler_Vector,
  HardFault_Handler_Vector,
  MemManage_Handler_Vector,
  BusFault_Handler_Vector,
  UsageFault_Handler_Vector,

  SVC_Handler_Vector = 11u,
  DebugMon_Handler_Vector,

  PendSV_Handler_Vector = 14u,
  SysTick_Handler_Vector,
  WWDG_IRQHandler_Vector,
  PVD_IRQHandler_Vector,
  TAMPER_IRQHandler_Vector,
  RTC_IRQHandler_Vector,
  FLASH_IRQHandler_Vector,
  RCC_IRQHandler_Vector,
  EXTI0_IRQHandler_Vector,
  EXTI1_IRQHandler_Vector,
  EXTI2_IRQHandler_Vector,
  EXTI3_IRQHandler_Vector,
  EXTI4_IRQHandler_Vector,
  DMA1_Channel1_IRQHandler_Vector,
  DMA1_Channel2_IRQHandler_Vector,
  DMA1_Channel3_IRQHandler_Vector,
  DMA1_Channel4_IRQHandler_Vector,
  DMA1_Channel5_IRQHandler_Vector,
  DMA1_Channel6_IRQHandler_Vector,
  DMA1_Channel7_IRQHandler_Vector,
  ADC1_2_IRQHandler_Vector,
  USB_HP_CAN1_TX_IRQHandler_Vector,
  USB_LP_CAN1_RX0_IRQHandler_Vector,
  CAN1_RX1_IRQHandler_Vector,
  CAN1_SCE_IRQHandler_Vector,
  EXTI9_5_IRQHandler_Vector,
  TIM1_BRK_TIM9_IRQHandler_Vector,
  TIM1_UP_TIM10_IRQHandler_Vector,
  TIM1_TRG_COM_TIM11_IRQHandler_Vector,
  TIM1_CC_IRQHandler_Vector,
  TIM2_IRQHandler_Vector,
  TIM3_IRQHandler_Vector,
  TIM4_IRQHandler_Vector,
  I2C1_EV_IRQHandler_Vector,
  I2C1_ER_IRQHandler_Vector,
  I2C2_EV_IRQHandler_Vector,
  I2C2_ER_IRQHandler_Vector,
  SPI1_IRQHandler_Vector,
  SPI2_IRQHandler_Vector,
  USART1_IRQHandler_Vector,
  USART2_IRQHandler_Vector,
  USART3_IRQHandler_Vector,
  EXTI15_10_IRQHandler_Vector,
  RTCAlarm_IRQHandler_Vector,
  USBWakeUp_IRQHandler_Vector,
  TIM8_BRK_TIM12_IRQHandler_Vector,
  TIM8_UP_TIM13_IRQHandler_Vector,
  TIM8_TRG_COM_TIM14_IRQHandler_Vector,
  TIM8_CC_IRQHandler_Vector,
  ADC3_IRQHandler_Vector,
  FSMC_IRQHandler_Vector,
  SDIO_IRQHandler_Vector,
  TIM5_IRQHandler_Vector,
  SPI3_IRQHandler_Vector,
  UART4_IRQHandler_Vector,
  UART5_IRQHandler_Vector,
  TIM6_IRQHandler_Vector,
  TIM7_IRQHandler_Vector,
  DMA2_Channel1_IRQHandler_Vector,
  DMA2_Channel2_IRQHandler_Vector,
  DMA2_Channel3_IRQHandler_Vector,
  DMA2_Channel4_5_IRQHandler_Vector,

  MAGIC = 120u,
  ISR_MAX_NUM
};

/* Irq handlers in RAM */
extern void __attribute__((section(".data"))) (*volatile g_pfnVectorsRam[ISR_MAX_NUM])();
extern void __attribute__((section(".isr_vector"))) (*volatile const g_pfnVectors[ISR_MAX_NUM])();

#  define HALT_IF_DEBUGGING()                                                                                                                                                                          \
    do {                                                                                                                                                                                               \
      if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) {                                                                                                                                          \
        while (true)                                                                                                                                                                                   \
          ;                                                                                                                                                                                            \
      } else {                                                                                                                                                                                         \
        NVIC_SystemReset();                                                                                                                                                                            \
      }                                                                                                                                                                                                \
    } while (0)

extern "C" void __attribute__((weak)) Reset_Handler() { _start(); };
extern "C" void __attribute__((weak)) NMI_Handler() {

  asm("tst lr, #4");
  asm("ite eq");
  asm("mrseq r0, msp");
  asm("mrsne r0, psp");
  asm("b __nmi_handler");
}

extern "C" void __attribute__((weak)) HardFault_Handler() {
  asm("tst lr, #4");
  asm("ite eq");
  asm("mrseq r0, msp");
  asm("mrsne r0, psp");
  asm("b __hard_fault_handler");
}

extern "C" void __attribute__((weak)) MemManage_Handler() {
  asm("tst lr, #4");
  asm("ite eq");
  asm("mrseq r0, msp");
  asm("mrsne r0, psp");
  asm("b __hard_fault_handler");
}

extern "C" void __attribute__((weak)) BusFault_Handler() {
  asm("tst lr, #4");
  asm("ite eq");
  asm("mrseq r0, msp");
  asm("mrsne r0, psp");
  asm("b __hard_fault_handler");
}

extern "C" void __attribute__((weak)) UsageFault_Handler() {
  asm("tst lr, #4");
  asm("ite eq");
  asm("mrseq r0, msp");
  asm("mrsne r0, psp");
  asm("b __hard_fault_handler");
}

extern "C" void __attribute__((weak)) SVC_Handler() { g_pfnVectorsRam[SVC_Handler_Vector](); };
extern "C" void __attribute__((weak)) DebugMon_Handler() { g_pfnVectorsRam[DebugMon_Handler_Vector](); };
extern "C" void __attribute__((weak)) PendSV_Handler() { g_pfnVectorsRam[PendSV_Handler_Vector](); };
extern "C" void __attribute__((weak)) SysTick_Handler() { g_pfnVectorsRam[SysTick_Handler_Vector](); };
extern "C" void __attribute__((weak)) WWDG_IRQHandler() { g_pfnVectorsRam[WWDG_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) PVD_IRQHandler() { g_pfnVectorsRam[PVD_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) TAMPER_IRQHandler() { g_pfnVectorsRam[TAMPER_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) FLASH_IRQHandler() { g_pfnVectorsRam[FLASH_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) RCC_IRQHandler() { g_pfnVectorsRam[RCC_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) EXTI0_IRQHandler() { g_pfnVectorsRam[EXTI0_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) EXTI1_IRQHandler() { g_pfnVectorsRam[EXTI1_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) EXTI2_IRQHandler() { g_pfnVectorsRam[EXTI2_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) EXTI3_IRQHandler() { g_pfnVectorsRam[EXTI3_IRQHandler_Vector](); }
extern "C" void __attribute__((weak)) EXTI4_IRQHandler() { g_pfnVectorsRam[EXTI4_IRQHandler_Vector](); };

extern "C" void __attribute__((weak)) DMA1_Channel1_IRQHandler() {
  g_pfnVectorsRam[DMA1_Channel1_IRQHandler_Vector]();
};

extern "C" void __attribute__((weak)) DMA1_Channel2_IRQHandler() {
  g_pfnVectorsRam[DMA1_Channel2_IRQHandler_Vector]();
};

extern "C" void __attribute__((weak)) DMA1_Channel3_IRQHandler() {
  g_pfnVectorsRam[DMA1_Channel3_IRQHandler_Vector]();
};

extern "C" void __attribute__((weak)) DMA1_Channel4_IRQHandler() {
  g_pfnVectorsRam[DMA1_Channel4_IRQHandler_Vector]();
};

extern "C" void __attribute__((weak)) DMA1_Channel5_IRQHandler() {
  g_pfnVectorsRam[DMA1_Channel5_IRQHandler_Vector]();
};

extern "C" void __attribute__((weak)) DMA1_Channel6_IRQHandler() {
  g_pfnVectorsRam[DMA1_Channel6_IRQHandler_Vector]();
};

extern "C" void __attribute__((weak)) DMA1_Channel7_IRQHandler() {
  g_pfnVectorsRam[DMA1_Channel7_IRQHandler_Vector]();
};

extern "C" void __attribute__((weak)) ADC1_2_IRQHandler() { g_pfnVectorsRam[ADC1_2_IRQHandler_Vector](); };

extern "C" void __attribute__((weak)) USB_HP_CAN1_TX_IRQHandler() {
  g_pfnVectorsRam[USB_HP_CAN1_TX_IRQHandler_Vector]();
};

extern "C" void __attribute__((weak)) USB_LP_CAN1_RX0_IRQHandler() {
  g_pfnVectorsRam[USB_LP_CAN1_RX0_IRQHandler_Vector]();
};

extern "C" void __attribute__((weak)) CAN1_RX1_IRQHandler() { g_pfnVectorsRam[CAN1_RX1_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) CAN1_SCE_IRQHandler() { g_pfnVectorsRam[CAN1_SCE_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) EXTI9_5_IRQHandler() { g_pfnVectorsRam[EXTI9_5_IRQHandler_Vector](); };

extern "C" void __attribute__((weak)) TIM1_BRK_TIM9_IRQHandler() {
  g_pfnVectorsRam[TIM1_BRK_TIM9_IRQHandler_Vector]();
};

extern "C" void __attribute__((weak)) TIM1_UP_TIM10_IRQHandler() {
  g_pfnVectorsRam[TIM1_UP_TIM10_IRQHandler_Vector]();
};

extern "C" void __attribute__((weak)) TIM1_TRG_COM_TIM11_IRQHandler() {
  g_pfnVectorsRam[TIM1_TRG_COM_TIM11_IRQHandler_Vector]();
};

extern "C" void __attribute__((weak)) TIM1_CC_IRQHandler() { g_pfnVectorsRam[TIM1_CC_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) TIM2_IRQHandler() { g_pfnVectorsRam[TIM2_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) TIM3_IRQHandler() { g_pfnVectorsRam[TIM3_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) TIM4_IRQHandler() { g_pfnVectorsRam[TIM4_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) I2C1_EV_IRQHandler() { g_pfnVectorsRam[I2C1_EV_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) I2C1_ER_IRQHandler() { g_pfnVectorsRam[I2C1_ER_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) I2C2_EV_IRQHandler() { g_pfnVectorsRam[I2C2_EV_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) I2C2_ER_IRQHandler() { g_pfnVectorsRam[I2C2_ER_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) SPI1_IRQHandler() { g_pfnVectorsRam[SPI1_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) SPI2_IRQHandler() { g_pfnVectorsRam[SPI2_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) USART1_IRQHandler() { g_pfnVectorsRam[USART1_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) USART2_IRQHandler() { g_pfnVectorsRam[USART2_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) USART3_IRQHandler() { g_pfnVectorsRam[USART3_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) EXTI15_10_IRQHandler() { g_pfnVectorsRam[EXTI15_10_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) RTCAlarm_IRQHandler() { g_pfnVectorsRam[RTCAlarm_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) RTC_IRQHandler() { g_pfnVectorsRam[RTC_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) USBWakeUp_IRQHandler() { g_pfnVectorsRam[USBWakeUp_IRQHandler_Vector](); };

extern "C" void __attribute__((weak)) TIM8_BRK_TIM12_IRQHandler() {
  g_pfnVectorsRam[TIM8_BRK_TIM12_IRQHandler_Vector]();
};

extern "C" void __attribute__((weak)) TIM8_UP_TIM13_IRQHandler() {
  g_pfnVectorsRam[TIM8_UP_TIM13_IRQHandler_Vector]();
};

extern "C" void __attribute__((weak)) TIM8_TRG_COM_TIM14_IRQHandler() {
  g_pfnVectorsRam[TIM8_TRG_COM_TIM14_IRQHandler_Vector]();
};

extern "C" void __attribute__((weak)) TIM8_CC_IRQHandler() { g_pfnVectorsRam[TIM8_CC_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) ADC3_IRQHandler() { g_pfnVectorsRam[ADC3_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) FSMC_IRQHandler() { g_pfnVectorsRam[FSMC_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) SDIO_IRQHandler() { g_pfnVectorsRam[SDIO_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) TIM5_IRQHandler() { g_pfnVectorsRam[TIM5_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) SPI3_IRQHandler() { g_pfnVectorsRam[SPI3_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) UART4_IRQHandler() { g_pfnVectorsRam[UART4_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) UART5_IRQHandler() { g_pfnVectorsRam[UART5_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) TIM6_IRQHandler() { g_pfnVectorsRam[TIM6_IRQHandler_Vector](); };
extern "C" void __attribute__((weak)) TIM7_IRQHandler() { g_pfnVectorsRam[TIM7_IRQHandler_Vector](); };

extern "C" void __attribute__((weak)) DMA2_Channel1_IRQHandler() {
  g_pfnVectorsRam[DMA2_Channel1_IRQHandler_Vector]();
};

extern "C" void __attribute__((weak)) DMA2_Channel2_IRQHandler() {
  g_pfnVectorsRam[DMA2_Channel2_IRQHandler_Vector]();
};

extern "C" void __attribute__((weak)) DMA2_Channel3_IRQHandler() {
  g_pfnVectorsRam[DMA2_Channel3_IRQHandler_Vector]();
};

extern "C" void __attribute__((weak)) DMA2_Channel4_5_IRQHandler() {
  g_pfnVectorsRam[DMA2_Channel4_5_IRQHandler_Vector]();
};

extern void (*__preinit_array_start[])() __attribute__((weak));
extern void (*__preinit_array_end[])() __attribute__((weak));
extern void (*__init_array_start[])() __attribute__((weak));
extern void (*__init_array_end[])() __attribute__((weak));
extern void (*__fini_array_start[])() __attribute__((weak));
extern void (*__fini_array_end[])() __attribute__((weak));

struct mcu_stm32f10x_xl_tag_s : mcu_tag_s {
  explicit mcu_stm32f10x_xl_tag_s() = default;
  virtual ~mcu_stm32f10x_xl_tag_s() = default;

  static inline void __attribute__((always_inline))
  init_data(uint32_t *from, uint32_t *section_begin, uint32_t *section_end) {
    uint32_t *p = section_begin;
    while (p < section_end)
      *p++ = *from++;
  }

  static inline void __attribute__((always_inline)) init_bss(uint32_t *section_begin, uint32_t *section_end) {
    uint32_t *p = section_begin;
    while (p < section_end)
      *p++ = 0;
  }

  static inline void __attribute__((always_inline)) run_init_array() {
    int32_t count;
    int32_t i;
    count = __preinit_array_end - __preinit_array_start;

    for (i = 0; i < count; i++)
      __preinit_array_start[i]();

    count = __init_array_end - __init_array_start;

    for (i = 0; i < count; i++)
      __init_array_start[i]();
  }

  static inline void __attribute__((always_inline)) run_fini_array() {
    int32_t count;
    int32_t i;
    count = __fini_array_end - __fini_array_start;

    for (i = count; i > 0; i--)
      __fini_array_start[i - 1]();
  }

#  if defined(DEBUG) && defined(OS_INCLUDE_STARTUP_GUARD_CHECKS)
#    define BSS_GUARD_BAD_VALUE (0xCADEBABA)

  static uint32_t volatile __attribute__((section(".bss_begin"))) __bss_begin_guard;
  static uint32_t volatile __attribute__((section(".bss_end"))) __bss_end_guard;

#    define DATA_GUARD_BAD_VALUE (0xCADEBABA)
#    define DATA_BEGIN_GUARD_VALUE (0x12345678)
#    define DATA_END_GUARD_VALUE (0x98765432)

  static uint32_t volatile __attribute__((section(".data_begin"))) __data_begin_guard = DATA_BEGIN_GUARD_VALUE;
  static uint32_t volatile __attribute__((section(".data_end"))) __data_end_guard = DATA_END_GUARD_VALUE;

#  endif /* defined( DEBUG ) && defined( OS_INCLUDE_STARTUP_GUARD_CHECKS ) */

  static void hard_exceptions_enable() {
    SCB->SHCSR = SCB->SHCSR | SCB_SHCSR_BUSFAULTENA;
    SCB->SHCSR = SCB->SHCSR | SCB_SHCSR_MEMFAULTENA;
    SCB->SHCSR = SCB->SHCSR | SCB_SHCSR_USGFAULTENA;
    SCB->SHCSR = SCB->SHCSR | SCB_SHCSR_MEMFAULTENA;
  }

  static void hard_exceptions_disable() {
    SCB->SHCSR = SCB->SHCSR & ~(SCB_SHCSR_BUSFAULTENA);
    SCB->SHCSR = SCB->SHCSR & ~(SCB_SHCSR_MEMFAULTENA);
    SCB->SHCSR = SCB->SHCSR & ~(SCB_SHCSR_USGFAULTENA);
    SCB->SHCSR = SCB->SHCSR & ~(SCB_SHCSR_MEMFAULTENA);
  }
};

#endif /* STM32F10X_XL */
#endif /* MCU_TAG_HPP */
