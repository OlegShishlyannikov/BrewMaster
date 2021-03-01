#ifndef LEDS_DRIVER_HPP
#define LEDS_DRIVER_HPP

#include "driver.hpp"

/* Driver specific init - exit functions */
void leds_drv_init(const struct drv_model_cmn_s *);
void leds_drv_exit(const struct drv_model_cmn_s *);

extern struct drv_ops_s leds_drv_ops;

struct leds_drv_name_s {
  static constexpr const char *const name = "leds";
};

#endif /* LEDS_DRIVER_HPP */
