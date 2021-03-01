#ifndef RELAY_LOAD_IOCTL_HPP
#define RELAY_LOAD_IOCTL_HPP

#include <cstdint>

enum relay_load_ioctl_cmd_e : uint64_t { RELAY_LOAD_ON = 0u, RELAY_LOAD_OFF, RELAY_LOAD_GET_STATE };
enum relay_load_state_e : uint32_t { RELAY_LOAD_STATE_ON = 0u, RELAY_LOAD_STATE_OFF };

/* These structures describe RELAY_LOAD driver requests */
struct relay_load_get_state_req_s {
  enum relay_load_state_e *const state;
};

#endif /* RELAY_LOAD_IOCTL_HPP */
