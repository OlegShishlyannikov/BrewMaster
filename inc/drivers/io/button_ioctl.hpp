#ifndef BUTTON_IOCTL_HPP
#define BUTTON_IOCTL_HPP

#include <cstdint>
#include <cstdlib>

enum button_ioctl_cmd_e : uint64_t { BUTTON_IRQ_ENABLE, BUTTON_IRQ_DISABLE, BUTTON_GET_STATE };
enum button_triggers_e : uint32_t { BUTTON_TRIGGER_RISING = 0u, BUTTON_TRIGGER_FALLING, BUTTON_TRIGGER_RISING_FALLING };
enum button_state_e : uint32_t { BUTTON_PRESSED = 0u, BUTTON_RELEASED };

/* GPIO interrupts management requests */
struct button_irq_mgm_req_s {
  enum button_triggers_e trigger;
  void (*callback)(const void *, size_t);
};

struct button_get_state_req_s {
  enum button_state_e *const state;
};

#endif /* BUTTON_IOCTL_HPP */
