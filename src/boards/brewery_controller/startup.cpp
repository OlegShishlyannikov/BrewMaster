#include "arch/mcu_tags.hpp"

/* RAM isr table */
void __attribute__((used, section(".data"))) (*volatile g_pfnVectorsRam[ISR_MAX_NUM])(void){0x0u};

/* ROM isr table */
void __attribute__((used, section(".isr_vector"))) (*volatile const g_pfnVectors[ISR_MAX_NUM])(void) = {

    reinterpret_cast<void (*)(void)>(&_estack),
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0,
    0,
    0,
    0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
    WWDG_IRQHandler,
    PVD_IRQHandler,
    TAMPER_IRQHandler,
    RTC_IRQHandler,
    FLASH_IRQHandler,
    RCC_IRQHandler,
    EXTI0_IRQHandler,
    EXTI1_IRQHandler,
    EXTI2_IRQHandler,
    EXTI3_IRQHandler,
    EXTI4_IRQHandler,
    DMA1_Channel1_IRQHandler,
    DMA1_Channel2_IRQHandler,
    DMA1_Channel3_IRQHandler,
    DMA1_Channel4_IRQHandler,
    DMA1_Channel5_IRQHandler,
    DMA1_Channel6_IRQHandler,
    DMA1_Channel7_IRQHandler,
    ADC1_2_IRQHandler,
    USB_HP_CAN1_TX_IRQHandler,
    USB_LP_CAN1_RX0_IRQHandler,
    CAN1_RX1_IRQHandler,
    CAN1_SCE_IRQHandler,
    EXTI9_5_IRQHandler,
    TIM1_BRK_TIM9_IRQHandler,
    TIM1_UP_TIM10_IRQHandler,
    TIM1_TRG_COM_TIM11_IRQHandler,
    TIM1_CC_IRQHandler,
    TIM2_IRQHandler,
    TIM3_IRQHandler,
    TIM4_IRQHandler,
    I2C1_EV_IRQHandler,
    I2C1_ER_IRQHandler,
    I2C2_EV_IRQHandler,
    I2C2_ER_IRQHandler,
    SPI1_IRQHandler,
    SPI2_IRQHandler,
    USART1_IRQHandler,
    USART2_IRQHandler,
    USART3_IRQHandler,
    EXTI15_10_IRQHandler,
    RTCAlarm_IRQHandler,
    USBWakeUp_IRQHandler,
    TIM8_BRK_TIM12_IRQHandler,
    TIM8_UP_TIM13_IRQHandler,
    TIM8_TRG_COM_TIM14_IRQHandler,
    TIM8_CC_IRQHandler,
    ADC3_IRQHandler,
    FSMC_IRQHandler,
    SDIO_IRQHandler,
    TIM5_IRQHandler,
    SPI3_IRQHandler,
    UART4_IRQHandler,
    UART5_IRQHandler,
    TIM6_IRQHandler,
    TIM7_IRQHandler,
    DMA2_Channel1_IRQHandler,
    DMA2_Channel2_IRQHandler,
    DMA2_Channel3_IRQHandler,
    DMA2_Channel4_5_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    reinterpret_cast<void (*)(void)>(0xF1E0F85F)};

void panic(context_state_frame_s *frame) {
  frame->bfar = SCB->HFSR;
  frame->cfsr = SCB->CFSR;
  frame->hfsr = SCB->HFSR;
  frame->dfsr = SCB->DFSR;
  frame->afsr = SCB->AFSR;
  HALT_IF_DEBUGGING();
}

extern "C" void __attribute__((used)) __nmi_handler(context_state_frame_s *frame) { panic(frame); }
extern "C" void __attribute__((used)) __usage_fault_handler(context_state_frame_s *frame) { panic(frame); }
extern "C" void __attribute__((used)) __bus_fault_handler(context_state_frame_s *frame) { panic(frame); }
extern "C" void __attribute__((used)) __mem_manage_handler(context_state_frame_s *frame) { panic(frame); }
extern "C" void __attribute__((used)) __hard_fault_handler(context_state_frame_s *frame) { panic(frame); }

extern int main(int argc, char *argv[]);

void __attribute__((noreturn)) _exit(int32_t) { LOOP(); }

void __attribute__((used, noreturn, section(".after_vectors"))) _start(void) {
#if defined(DEBUG) && defined(OS_INCLUDE_STARTUP_GUARD_CHECKS)
  __bss_begin_guard = BSS_GUARD_BAD_VALUE;
  __bss_end_guard = BSS_GUARD_BAD_VALUE;
#endif

  mcu_stm32f10x_xl_tag_s::init_bss(&__bss_start__, &__bss_end__);

#if defined(DEBUG) && defined(OS_INCLUDE_STARTUP_GUARD_CHECKS)
  if ((__bss_begin_guard != 0) || (__bss_end_guard != 0)) {

    LOOP();
  }
#endif

#if defined(DEBUG) && defined(OS_INCLUDE_STARTUP_GUARD_CHECKS)
  __data_begin_guard = DATA_GUARD_BAD_VALUE;
  __data_end_guard = DATA_GUARD_BAD_VALUE;
#endif

  mcu_stm32f10x_xl_tag_s::init_data(&_sidata, &_sdata, &_edata);

#if defined(DEBUG) && defined(OS_INCLUDE_STARTUP_GUARD_CHECKS)
  if ((__data_begin_guard != DATA_BEGIN_GUARD_VALUE)) {

    LOOP();
  }
#endif

  int32_t argc, rc;
  char **argv;
  mcu_stm32f10x_xl_tag_s::hard_exceptions_enable();
  mcu_stm32f10x_xl_tag_s::run_init_array();
  rc = main(argc, argv);
  mcu_stm32f10x_xl_tag_s::run_fini_array();
  _exit(rc);
}
