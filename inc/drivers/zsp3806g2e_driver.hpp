#ifndef ZSP3806G2E_DRIVER_HPP
#define ZSP3806G2E_DRIVER_HPP

#include "driver.hpp"

/* Driver specific init - exit functions */
void zsp3806g2e_drv_init(const struct drv_model_cmn_s *);
void zsp3806g2e_drv_exit(const struct drv_model_cmn_s *);

extern struct drv_ops_s zsp3806g2e_drv_ops;

struct zsp3806g2e_drv_name_s {
  static constexpr const char *const name = "zsp3806g2e";
};

#endif /* ZSP3806G2E_DRIVER_HPP */
