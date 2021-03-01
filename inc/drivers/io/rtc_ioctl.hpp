#ifndef RTC_IOCTL_HPP
#define RTC_IOCTL_HPP

#include <cstdint>
#include <cstdlib>

enum rtc_ioctl_cmd_e : uint64_t {
  RTC_ON_SECOND_SET = 0u,
  RTC_ON_MINUTE_SET,
  RTC_ON_HOUR_SET,
  RTC_ON_DAY_SET,
  RTC_ON_SECOND_RESET,
  RTC_ON_MINUTE_RESET,
  RTC_ON_HOUR_RESET,
  RTC_ON_DAY_RESET,
  RTC_SET_TIME,
  RTC_GET_TIME,
  RTC_IRQ_ENABLE,
  RTC_IRQ_DISABLE
};

struct rtc_s {
  uint8_t hour; /* 0..23 */
  uint8_t min;  /* 0..59 */
  uint8_t sec;  /* 0..59 */
};

struct rtc_set_time_req_s {
  const struct rtc_s *rtc;
};

struct rtc_get_time_req_s {
  struct rtc_s *const rtc;
};

struct rtc_callback_req_s {
  void (*callback)(const void *, size_t);
};

struct rtc_irq_req_s {
  int32_t priority;
};

#endif /* RTC_IOCTL_HPP */
