#ifndef AC_LOAD_DRIVER_HPP
#define AC_LOAD_DRIVER_HPP

#include "driver.hpp"

/* Driver specific init - exit functions */
void ac_load_drv_init(const struct drv_model_cmn_s *);
void ac_load_drv_exit(const struct drv_model_cmn_s *);

extern struct drv_ops_s ac_load_drv_ops;

struct ac_load_drv_name_s {
  static constexpr const char *const name = "ac_load";
};

#endif /* AC_LOAD_DRIVER_HPP */
