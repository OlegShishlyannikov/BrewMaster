#ifndef USART_DRIVER_HPP
#define USART_DRIVER_HPP

#include "driver.hpp"

/* Driver specific init - exit functions */
void usart_drv_init(const struct drv_model_cmn_s *);
void usart_drv_exit(const struct drv_model_cmn_s *);

extern struct drv_ops_s usart_drv_ops;

struct usart_drv_name_s {
  static constexpr const char *const name = "usart";
};

#endif /* USART_DRIVER_HPP */
