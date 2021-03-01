#ifndef LEDS_IOCTL_HPP
#define LEDS_IOCTL_HPP

#include <cstdint>

enum leds_ioctl_cmd_e : uint64_t { LEDS_BLINK };

/* These structures describe LEDS driver requests */
/* LEDS_IOCTL blink request */
struct led_blink_req_s {
  uint32_t up_ticks;
  uint32_t down_ticks;
  uint32_t n_times;
  void (*delay_fn)(uint32_t);
};

#endif /* LEDS_IOCTL_HPP */
