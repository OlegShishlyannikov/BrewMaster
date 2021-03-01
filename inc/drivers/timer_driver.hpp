#ifndef TIMER_DRIVER_HPP
#define TIMER_DRIVER_HPP

#include "driver.hpp"

/* Driver specific init - exit functions */
void timer_drv_init(const struct drv_model_cmn_s *);
void timer_drv_exit(const struct drv_model_cmn_s *);

extern struct drv_ops_s timer_drv_ops;

struct timer_drv_name_s {
  static constexpr const char *const name = "timer";
};

#endif /* TIMER_DRIVER_HPP */
