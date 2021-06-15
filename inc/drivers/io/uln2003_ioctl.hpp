#ifndef ULN2003_IOCTL_HPP
#define ULN2003_IOCTL_HPP

#include <cstdint>
#include <cstdlib>

enum uln2003_ioctl_cmd_e : uint64_t { ULN2003_CHANNEL_ON = 0u, ULN2003_CHANNEL_OFF, ULN2003_GET_STATE };

struct uln2003_ch_on_off_req_s {
  uint8_t ch_num;
};

struct uln2003_get_state_req_s {
  uint8_t ch_num;
  uint8_t *const state_reg;
};

#endif /* ULN2003_IOCTL_HPP */
