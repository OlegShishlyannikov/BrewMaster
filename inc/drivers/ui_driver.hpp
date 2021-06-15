#ifndef UI_DRIVER_HPP
#define UI_DRIVER_HPP

#include "driver.hpp"

/* Driver specific init - exit functions */
void ui_drv_init(const struct drv_model_cmn_s *);
void ui_drv_exit(const struct drv_model_cmn_s *);

extern struct drv_ops_s ui_drv_ops;

struct ui_drv_name_s {
  static constexpr const char *const name = "ui";
};

struct ui_reg_val_s {
  uint8_t val;
};

#endif /* UI_DRIVER_HPP */
