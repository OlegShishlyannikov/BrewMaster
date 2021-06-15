#ifndef TIMER_IOCTL_HPP
#define TIMER_IOCTL_HPP

#include <cstdint>
#include <cstdlib>

enum timer_ioctl_cmd_e : uint64_t { TIMER_INIT = 0u, TIMER_DEINIT, TIMER_START, TIMER_STOP, TIMER_CBK_SET, TIMER_CBK_RESET };

struct timer_cbk_req_s {
  void (*callback)(const void *, size_t);
};

enum timer_setup_cnt_mode_e : uint64_t { TIM_CNT_MODE_UP = 0u, TIM_CNT_MODE_DOWN, TIM_CNT_CA1, TIM_CNT_CA2, TIM_CNT_CA3 };
enum timer_setup_clkdiv_e : uint64_t { TIM_CLKDIV1 = 0u, TIM_CLKDIV2, TIM_CLKDIV4 };

enum timer_setup_period_mode_e : uint64_t {

};
struct timer_setup_req_s {
  enum timer_setup_cnt_mode_e cnt_mode;
  uint16_t psc;
  uint16_t period;
  enum timer_setup_clkdiv_e clk_div;
  uint8_t rep_cnt;
  uint16_t irq_priority;
};

#endif /* TIMER_IOCTL_HPP */
