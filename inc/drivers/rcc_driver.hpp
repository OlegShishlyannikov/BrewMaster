#ifndef RCC_DRIVER_HPP
#define RCC_DRIVER_HPP

#include "driver.hpp"

/* Driver specific init - exit functions */
void rcc_drv_init(const struct drv_model_cmn_s *);
void rcc_drv_exit(const struct drv_model_cmn_s *);

extern struct drv_ops_s rcc_drv_ops;

struct rcc_drv_name_s {
  static constexpr const char *const name = "rcc";
};

#endif /* RCC_DRIVER_HPP */
