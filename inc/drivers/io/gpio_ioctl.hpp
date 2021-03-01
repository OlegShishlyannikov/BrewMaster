#ifndef GPIO_IOCTL_HPP
#define GPIO_IOCTL_HPP

#include <cstdint>
#include <cstdlib>

enum gpio_ioctl_cmd_e : uint64_t {
  GPIO_SP_IFLOAT = 0u,
  GPIO_SP_IPL,
  GPIO_SP_IPH,
  GPIO_SP_PP,
  GPIO_SP_OD,
  GPIO_SP_ALT_PP,
  GPIO_SP_ALT_OD,
  GPIO_SP_ANALOG_IN,
  GPIO_PIN_WRITE,
  GPIO_PORT_WRITE,
  GPIO_STROBE,
  GPIO_IRQ_ENABLE,
  GPIO_IRQ_DISABLE
};

enum exti_triggers_e : uint32_t { GPIO_TRIGGER_RISING = 0u, GPIO_TRIGGER_FALLING, GPIO_TRIGGER_RISING_FALLING };

/* These structures describe GPIO driver requests */
/* GPIO setup requests */
struct gpio_setup_req_s {
  uint8_t pin;
};

/* GPIO write requests (entire port) */
struct gpio_write_port_req_s {
  uint16_t val;
};

/* GPIO write requests (one pin) */
struct gpio_write_pin_req_s {
  uint8_t pin;
  uint8_t val;
};

/* GPIO strobe request */
struct gpio_strobe_req_s {
  uint8_t pin;
  uint32_t up_ticks;
  uint32_t down_ticks;
  uint32_t n_times;
  void (*delay_fn)(uint32_t);
};

/* GPIO interrupts management requests */
struct gpio_irq_mgm_req_s {
  uint8_t exti_line;
  uint8_t priority;
  enum exti_triggers_e trigger;
  void (*callback)(const void *, size_t);
};

#endif /* GPIO_IOCTL_HPP */
