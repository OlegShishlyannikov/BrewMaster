#ifndef USART_IOCTL_HPP
#define USART_IOCTL_HPP

#include <cstdint>
#include <cstdlib>

enum usart_ioctl_cmd_e : uint64_t { USART_INIT = 0u, USART_DEINIT, USART_ON_RECV, USART_ON_SEND };
enum usart_hw_flow_ctrl_e : uint64_t { NONE = 0u, RTS, CTS, RTS_CTS };
enum usart_mode_e : uint64_t { RX = 0u, TX, RXTX };
enum usart_parity_e : uint64_t { NO = 0u, EVEN, ODD };
enum usart_stop_bits_e : uint64_t { SB_0_5 = 0u, SB_1, SB_1_5, SB_2 };
enum usart_word_len_e : uint64_t { WL_8B = 0u, WL_9B };

struct usart_setup_req_s {
  uint32_t baudrate;
  uint32_t irq_priority;
  enum usart_hw_flow_ctrl_e hw_flow_ctrl;
  enum usart_mode_e mode;
  enum usart_parity_e parity;
  enum usart_stop_bits_e sb;
  enum usart_word_len_e wl;
};

struct usart_callback_req_s {
  void (*callback)(const void *, size_t);
};

#endif /* USART_IOCTL_HPP */
