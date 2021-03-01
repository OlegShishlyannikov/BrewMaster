#ifndef BUTTON_DRIVER_HPP
#define BUTTON_DRIVER_HPP

#include "driver.hpp"

/* Driver specific init - exit functions */
void button_drv_init(const struct drv_model_cmn_s *);
void button_drv_exit(const struct drv_model_cmn_s *);

extern struct drv_ops_s button_drv_ops;

struct button_drv_name_s {
  static constexpr const char *const name = "button";
};

#endif /* BUTTON_DRIVER_HPP */
