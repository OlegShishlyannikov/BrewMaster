#ifndef HD44780_IOCTL_HPP
#define HD44780_IOCTL_HPP

#include <cstdint>

enum hd44780_ioctl_cmd_e : uint64_t { HD44780_BL_ENABLE = 0u, HD44780_BL_DISABLE, HD44780_INIT_8BIT, HD44780_INIT_4BIT, HD44780_SET_LINE, HD44780_SET_BLINKING, HD44780_SET_CURSOR, HD44780_CLEAR };

/* These structures describe HD44780 driver requests */
struct hd44780_cursor_pos_req_s {
  uint8_t line;
  uint8_t col;
};

#endif /* HD44780_IOCTL_HPP */
