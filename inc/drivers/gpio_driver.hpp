#ifndef GPIO_DRIVER_HPP
#define GPIO_DRIVER_HPP

#include "driver.hpp"

/* Driver specific init - exit functions */
void gpio_drv_init(const struct drv_model_cmn_s *);
void gpio_drv_exit(const struct drv_model_cmn_s *);

extern struct drv_ops_s gpio_drv_ops;

struct gpio_drv_name_s {
  static constexpr const char *name = "gpio";
};

#endif /* GPIO_DRIVER_HPP */
