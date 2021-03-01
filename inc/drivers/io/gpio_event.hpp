#ifndef GPIO_EVENT_HPP
#define GPIO_EVENT_HPP

#include <cstdint>

/* GPIO event */
struct gpio_event_s {
  uint8_t line;
  uint8_t *pin_val;
  uint8_t val_num;
};

#endif /* GPIO_EVENT_HPP */
