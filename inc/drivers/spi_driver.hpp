#ifndef SPI_DRIVER_HPP
#define SPI_DRIVER_HPP

#include "driver.hpp"

/* Driver specific init - exit functions */
void spi_drv_init(const struct drv_model_cmn_s *);
void spi_drv_exit(const struct drv_model_cmn_s *);

extern struct drv_ops_s spi_drv_ops;

struct spi_drv_name_s {
  static constexpr const char *const name = "spi";
};

#endif /* SPI_DRIVER_HPP */
