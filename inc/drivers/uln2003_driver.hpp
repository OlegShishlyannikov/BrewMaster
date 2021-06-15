#ifndef ULN2003_DRIVER_HPP
#define ULN2003_DRIVER_HPP

#include "driver.hpp"

/* Driver specific init - exit functions */
void uln2003_drv_init(const struct drv_model_cmn_s *);
void uln2003_drv_exit(const struct drv_model_cmn_s *);

extern struct drv_ops_s uln2003_drv_ops;

struct uln2003_drv_name_s {
  static constexpr const char *const name = "uln2003";
};

#endif /* ULN2003_DRIVER_HPP */
