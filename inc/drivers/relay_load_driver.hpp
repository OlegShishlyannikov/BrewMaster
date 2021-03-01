#ifndef RELAY_LOAD_DRIVER_HPP
#define RELAY_LOAD_DRIVER_HPP

#include "driver.hpp"

/* Driver specific init - exit functions */
void relay_load_drv_init(const struct drv_model_cmn_s *);
void relay_load_drv_exit(const struct drv_model_cmn_s *);

extern struct drv_ops_s relay_load_drv_ops;

struct relay_load_drv_name_s {
  static constexpr const char *const name = "relay_load";
};

#endif /* RELAY_LOAD_DRIVER_HPP */
