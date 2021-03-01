#ifndef BUZZER_DRIVER_HPP
#define BUZZER_DRIVER_HPP

#include "driver.hpp"

/* Driver specific init - exit functions */
void buzzer_drv_init(const struct drv_model_cmn_s *);
void buzzer_drv_exit(const struct drv_model_cmn_s *);

extern struct drv_ops_s buzzer_drv_ops;

struct buzzer_drv_name_s {
  static constexpr const char *const name = "buzzer";
};

#endif /* BUZZER_DRIVER_HPP */
