#ifndef BUTTON_EVENT_HPP
#define BUTTON_EVENT_HPP

#include <cstdint>
#include <cstdlib>

/* BUTTON event */
struct button_event_s {
  uint8_t pin_val;
  uint8_t btn_num;
};

#endif /* BUTTON_EVENT_HPP */
