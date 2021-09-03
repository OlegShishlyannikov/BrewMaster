#ifndef VFD_DRIVER_HPP
#define VFD_DRIVER_HPP

#include "driver.hpp"

/* Driver specific init - exit functions */
void vfd_drv_init(const struct drv_model_cmn_s *);
void vfd_drv_exit(const struct drv_model_cmn_s *);

extern struct drv_ops_s vfd_drv_ops;

struct vfd_drv_name_s {
  static constexpr const char *const name = "vfd";
};

#endif /* VFD_DRIVER_HPP */
