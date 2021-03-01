#ifndef USART_IOCTL_HPP
#define USART_IOCTL_HPP

#include <cstdint>
#include <cstdlib>

enum usart_ioctl_cmd_e : uint64_t { USART_INIT = 0u, USART_DEINIT, USART_ON_RECV, USART_ON_SEND };

struct usart_setup_req_s {
  uint32_t baudrate;
  uint32_t irq_priority;
};

struct usart_callback_req_s {
  void (*callback)(const void *, size_t);
};

#endif /* USART_IOCTL_HPP */
