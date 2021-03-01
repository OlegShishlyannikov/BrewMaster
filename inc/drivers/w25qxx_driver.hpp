#ifndef W25QXX_DRIVER_HPP
#define W25QXX_DRIVER_HPP

#include "driver.hpp"

/* Driver specific init - exit functions */
void w25qxx_drv_init(const struct drv_model_cmn_s *);
void w25qxx_drv_exit(const struct drv_model_cmn_s *);
int32_t w25qxx_printf(const char *, ...);

extern struct drv_ops_s w25qxx_drv_ops;

struct w25qxx_drv_name_s {
  static constexpr const char *const name = "w25qxx";
};

#endif /* W25QXX_DRIVER_HPP */
