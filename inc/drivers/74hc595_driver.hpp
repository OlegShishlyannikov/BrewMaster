#ifndef IC74HC595_DRIVER_HPP
#define IC74HC595_DRIVER_HPP

#include "driver.hpp"

/* Driver specific init - exit functions */
void ic74hc595_drv_init(const struct drv_model_cmn_s *);
void ic74hc595_drv_exit(const struct drv_model_cmn_s *);

extern struct drv_ops_s ic74hc595_drv_ops;

struct ic74hc595_drv_name_s {
  static constexpr const char *const name = "ic74hc595";
};

struct ic74hc595_reg_val_s {
  uint8_t val;
};

#endif /* IC74HC595_DRIVER_HPP */
