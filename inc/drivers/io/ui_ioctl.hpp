#ifndef INDICATION_IOCTL_HPP
#define INDICATION_IOCTL_HPP

#include <cstdint>
#include <cstdlib>

enum ui_ioctl_cmd_e : uint64_t {
  UI_DISP_LEFT_UP_SET_NUM = 0u,
  UI_DISP_LEFT_DOWN_SET_NUM,
  UI_DISP_RIGHT_UP_SET_NUM,
  UI_DISP_RIGHT_DOWN_SET_NUM,
  UI_DISP_LEFT_UP_RESET,
  UI_DISP_LEFT_DOWN_RESET,
  UI_DISP_RIGHT_UP_RESET,
  UI_DISP_RIGHT_DOWN_RESET,
  UI_KEY_CBK_SET,
  UI_KEY_CBK_RESET,
  UI_PEDAL_CBK_SET,
  UI_PEDAL_CBK_RESET
};

enum ui_key_cbk_type_e : uint64_t { UI_KEY_CBK_PRESS = 0u, UI_KEY_CBK_RELEASE };
enum ui_key_number_e : uint64_t { UI_KEY1 = 0u, UI_KEY2, UI_KEY3, UI_KEY4, UI_KEY5, UI_KEY6, UI_KEY7, UI_KEY8, UI_KEY9 };

struct ui_4d_disp_set_num_req_s {
  char num_str[4u];
  bool is_negative;
};

struct ui_2d_disp_set_num_req_s {
  char num_str[2u];
};

struct ui_key_cbk_req_s {
  enum ui_key_number_e key_no;
  enum ui_key_cbk_type_e type;
  void (*cbk)(const void *, size_t);
};

struct ui_pedal_cbk_req_s {
  enum ui_key_cbk_type_e type;
  void (*cbk)(const void *, size_t);
};

#endif /* INDICATION_IOCTL_HPP */
