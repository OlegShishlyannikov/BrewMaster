#ifndef BUZZER_HPP
#define BUZZER_HPP

#include <cstdint>

/* BUZZER driver ioctl requests */
enum buzzer_drv_ioctl_cmd_e : uint64_t { BUZZER_BEEP = 0u };

struct buzzer_beep_req_s {
  void(*delay_fn)(uint32_t);
  uint32_t up;
  uint32_t down;
  uint32_t n;
};

#endif /* BUZZER_HPP */
