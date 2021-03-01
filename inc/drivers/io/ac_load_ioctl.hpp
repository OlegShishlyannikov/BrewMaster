#ifndef AC_LOAD_IOCTL_HPP
#define AC_LOAD_IOCTL_HPP

#include <cstdint>

enum ac_load_ioctl_cmd_e : uint64_t { AC_LOAD_ON = 0u, AC_LOAD_OFF, AC_LOAD_GET_STATE };
enum ac_load_state_e : uint32_t { AC_LOAD_STATE_ON = 0u, AC_LOAD_STATE_OFF };

/* These structures describe RELAY_LOAD driver requests */
struct ac_load_get_state_req_s {
  enum ac_load_state_e *const state;
};

#endif /* AC_LOAD_IOCTL_HPP */
