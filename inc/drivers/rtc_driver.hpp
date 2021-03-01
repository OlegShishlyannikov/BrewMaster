#ifndef RTC_DRIVER_HPP
#define RTC_DRIVER_HPP

#include "driver.hpp"

/* Driver specific init - exit functions */
void rtc_drv_init(const struct drv_model_cmn_s *);
void rtc_drv_exit(const struct drv_model_cmn_s *);

extern struct drv_ops_s rtc_drv_ops;

struct rtc_drv_name_s {
  static constexpr const char *const name = "rtc";
};

#endif /* RTC_DRIVER_HPP */
