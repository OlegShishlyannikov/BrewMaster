#ifndef HD44780_DRIVER_HPP
#define HD44780_DRIVER_HPP

#include "driver.hpp"

/* Driver specific init - exit functions */
void hd44780_drv_init(const struct drv_model_cmn_s *);
void hd44780_drv_exit(const struct drv_model_cmn_s *);

extern struct drv_ops_s hd44780_drv_ops;

struct hd44780_drv_name_s {
  static constexpr const char *const name = "hd44780";
};

#endif /* HD44780_DRIVER_HPP */
