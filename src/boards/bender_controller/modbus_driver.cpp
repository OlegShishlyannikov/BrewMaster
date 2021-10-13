#include <cstdarg>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "arch/mcu_tags.hpp"
#include "boards/board_tags.hpp"
#include "drivers/io/gpio_ioctl.hpp"
#include "drivers/io/modbus_ioctl.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "drivers/io/usart_ioctl.hpp"
#include "drivers/modbus_driver.hpp"
#include "fio/unistd.hpp"
#include "sys/events_worker.hpp"

/* This pointer will be used in interrupt handlers and will be initialized in driver init function */
static const struct drv_model_cmn_s *drv_ptr;
extern bool debug_log_enabled;

/* Latch, clock and data pins used */
static constexpr const char *modbus_drv_usart_devstr = "usart1";
static constexpr const char *console_devstr = "usart2";

/* 74HC595 lock & fifos */
static xSemaphoreHandle modbus_lock;
extern xQueueHandle events_worker_queue;

// Input buffer
static char sequence_buffer[UINT8_MAX * 2u]{'\0'};

// Printf to console
static int32_t modbus_printf(const char *, ...);

/* 74HC595 file IO functions forward reference */
static int32_t modbus_drv_open(int32_t, mode_t);
static int32_t modbus_drv_ioctl(uint64_t, const void *, size_t);
static int32_t modbus_drv_read(void *const, size_t);
static int32_t modbus_drv_write(const void *, size_t);
static int32_t modbus_drv_close();

// Helper functions
static uint16_t crc16(uint8_t *, size_t);
static int32_t transmit_request(const uint8_t *, size_t);
static void modbus_char_recvd_callback(const void *, size_t);
static void (*modbus_sequence_recvd_callback)(const void *, size_t) = nullptr;
static void (*protocol_error_handler)(uint8_t) = nullptr;

/* 74HC595 helper functions */
static int32_t modbus_flock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreTakeRecursive(modbus_lock, portMAX_DELAY)) != pdPASS) {
    // errno = ???
    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t modbus_funlock() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;
  if ((rc = xSemaphoreGiveRecursive(modbus_lock)) != pdPASS) {
    // errno = ???
    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

struct drv_ops_s modbus_drv_ops {
  .init = modbus_drv_init, .exit = modbus_drv_exit
};

/* 74HC595 driver file operations secification */
struct file_ops_s modbus_drv_fops {
  .flock = modbus_flock, .funlock = modbus_funlock, .open = modbus_drv_open, .ioctl = modbus_drv_ioctl, .read = modbus_drv_read, .write = modbus_drv_write, .close = modbus_drv_close
};

void modbus_drv_init(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *usart;
  int32_t usart_fd, rc;

  struct usart_setup_req_s usart_setup_req {
    .baudrate = 9600u, .irq_priority = 5u, .hw_flow_ctrl = usart_hw_flow_ctrl_e::NONE, .mode = usart_mode_e::RXTX, .parity = usart_parity_e::NO, .sb = usart_stop_bits_e::SB_1,
    .wl = usart_word_len_e::WL_8B
  };

  struct usart_callback_req_s usart_cbk_req {
    .callback = modbus_char_recvd_callback
  };

  if (!(usart = drv->dep("usart"))) {
    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((usart_fd = ::open(usart, modbus_drv_usart_devstr, 3, 3u)) < 0) {
    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(usart, usart_fd, usart_ioctl_cmd_e::USART_INIT, &usart_setup_req, sizeof(usart_setup_req))) < 0) {
    if ((rc = ::close(usart, usart_fd)) < 0) {
      modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(usart, usart_fd, usart_ioctl_cmd_e::USART_ON_RECV, &usart_cbk_req, sizeof(usart_cbk_req))) < 0) {
    if ((rc = ::close(usart, usart_fd)) < 0) {
      modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(usart, usart_fd)) < 0) {
    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  modbus_drv_fops.owner = drv;

  /* Init locks */
  modbus_lock = xSemaphoreCreateRecursiveMutex();
  xSemaphoreGive(modbus_lock);

  /* Register char device for each GPIO port */
  drv->register_chardev("modbus0", &modbus_drv_fops);

  /* Initialize GPIO driver pointer */
  drv_ptr = drv;
  return;

  /* Something went wrong -- reset drv_ptr and exit */
error:
  modbus_drv_exit(drv);
  return;
}

void modbus_drv_exit(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *usart;
  int32_t rc, usart_fd;

  struct usart_callback_req_s usart_cbk_req {
    .callback = nullptr
  };

  if (!(usart = drv->dep("usart"))) {
    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((usart_fd = ::open(usart, modbus_drv_usart_devstr, 3, 3u)) < 0) {
    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(usart, usart_fd, usart_ioctl_cmd_e::USART_DEINIT, nullptr, 0u)) < 0) {
    if ((rc = ::close(usart, usart_fd)) < 0) {
      modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(usart, usart_fd, usart_ioctl_cmd_e::USART_ON_RECV, &usart_cbk_req, sizeof(usart_cbk_req))) < 0) {
    if ((rc = ::close(usart, usart_fd)) < 0) {
      modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(usart, usart_fd)) < 0) {
    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  modbus_drv_fops.owner = nullptr;

  /* Remove locks */
  vSemaphoreDelete(modbus_lock);

  /* Register char device for each GPIO port */
  drv->unregister_chardev("modbus0");

  /* Initialize GPIO driver pointer */
  drv_ptr = nullptr;
  return;
error:
  return;
}

/* 74HC595 file IO functions */
static int32_t modbus_drv_open(int32_t, mode_t) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t modbus_drv_ioctl(uint64_t req, const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (req) {
  case MODBUS_QUERY: {
    const struct modbus_req_s *modbus_req = reinterpret_cast<const struct modbus_req_s *>(buf);

    switch (modbus_req->func_code) {

    case MODBUS_RTU_READ_DO ... MODBUS_RTU_READ_AI: {
      uint8_t buffer[8u];
      uint16_t crc;

      buffer[0u] = modbus_req->device_id;
      buffer[1u] = static_cast<uint8_t>(modbus_req->func_code);
      buffer[2u] = modbus_req->reg_addr >> 8u;
      buffer[3u] = modbus_req->reg_addr & 0xffu;
      buffer[4u] = modbus_req->size >> 8u;
      buffer[5u] = modbus_req->size & 0xffu;

      crc = crc16(buffer, sizeof(buffer) - sizeof(crc));
      buffer[6u] = crc & 0xffu;
      buffer[7u] = crc >> 8u;
      transmit_request(buffer, sizeof(buffer));
    } break;

    case MODBUS_RTU_WRITE_DO ... MODBUS_RTU_WRITE_AO: {
      uint8_t buffer[8u];
      uint16_t crc;

      buffer[0u] = modbus_req->device_id;
      buffer[1u] = static_cast<uint8_t>(modbus_req->func_code);
      buffer[2u] = modbus_req->reg_addr >> 8u;
      buffer[3u] = modbus_req->reg_addr & 0xffu;
      buffer[4u] = modbus_req->data[1u];
      buffer[5u] = modbus_req->data[0u];

      crc = crc16(buffer, sizeof(buffer) - sizeof(crc));
      buffer[6u] = crc & 0xffu;
      buffer[7u] = crc >> 8u;
      transmit_request(buffer, sizeof(buffer));
    } break;

    case MODBUS_RTU_MULT_WRITE_DO ... MODBUS_RTU_MULT_WRITE_AO: {
      uint8_t *buffer = reinterpret_cast<uint8_t *>(std::calloc(4u + modbus_req->size, sizeof(uint8_t)));
    } break;

    default:
      break;
    }
  } break;

  case MODBUS_RTU_ON_SEQ_RECVD: {
    const struct modbus_cbk_req_s *request = reinterpret_cast<const struct modbus_cbk_req_s *>(buf);
    modbus_sequence_recvd_callback = request->cbk;
  } break;

  case MODBUS_RTU_ON_ERR: {
    const struct modbus_err_cbk_req_s *request = reinterpret_cast<const struct modbus_err_cbk_req_s *>(buf);
    protocol_error_handler = request->cbk;
  } break;

  default:
    break;
  }

  return 0;
error:
  return -1;
}

static int32_t modbus_drv_read(void *const, size_t) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t modbus_drv_write(const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t modbus_drv_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t modbus_printf(const char *fmt, ...) {
  int32_t rc, usart_fd, strlen;
  static char *temp;
  const struct drv_model_cmn_s *usart;

  std::va_list arg;
  va_start(arg, fmt);
  size_t bufsz = std::vsnprintf(nullptr, 0u, fmt, arg);
  temp = static_cast<char *>(calloc(bufsz, sizeof(char)));
  strlen = std::vsprintf(temp, fmt, arg);
  va_end(arg);

  if (!debug_log_enabled) {
    goto exit;
  }

  if (!(usart = drv_ptr)) {
    goto error;
  }

  if (strlen) {
    if ((usart_fd = ::open(usart, console_devstr, 3, 3u)) < 0) {
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, "[modbus] : ", std::strlen("[modbus] : "))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        goto error;
      }
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, temp, std::strlen(temp))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        goto error;
      }

      goto error;
    }

    if ((rc = ::close(usart, usart_fd)) < 0) {
      goto error;
    }
  }

exit:
  free(temp);
  return strlen;
error:

  free(temp);
  return -1;
}

static uint16_t crc16(uint8_t *data, size_t size) {
  static const uint16_t crc16_table[] = {
      0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440, 0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
      0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841, 0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
      0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641, 0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
      0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441, 0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
      0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40, 0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
      0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041, 0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
      0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41, 0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840, 0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
      0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40, 0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
      0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440, 0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
      0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841, 0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
      0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};

  uint16_t crc = 0xffffu;

  while (size--) {
    crc = (crc >> 8u) ^ crc16_table[(crc ^ *data++) & 0xffu];
  }

  return crc;
}

static int32_t transmit_request(const uint8_t *data, size_t size) {
  const struct drv_model_cmn_s *usart;
  int32_t rc, usart_fd;

  if (!(usart = drv_ptr->dep("usart"))) {
    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((usart_fd = ::open(usart, modbus_drv_usart_devstr, 3, 3u)) < 0) {
    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::write(usart, usart_fd, data, size)) < 0) {
    if ((rc = ::close(usart, usart_fd)) < 0) {
      modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(usart, usart_fd)) < 0) {
    modbus_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Clear sequence buffer before receiveing response from slave
  std::memset(sequence_buffer, '\0', sizeof(sequence_buffer));
  return rc;
error:
  return -1;
}

static void modbus_char_recvd_callback(const void *data, size_t size) {
  struct modbus_resp_s *response;
  static uint32_t byte_cnt = 0u;

  static uint8_t dev_id = 0u;
  static uint8_t func_code = 0u;
  static uint16_t reg_addr = 0u;
  static uint8_t nbyte = 0u;
  static uint8_t *data_buffer = nullptr;
  static uint8_t error_code = 0u;
  static uint16_t crc = 0u;
  static uint16_t value = 0u;

  const char recvd = *static_cast<const char *>(data);

  switch (byte_cnt) {

    // First byte -- Device ID received
  case 0u: {
    dev_id = recvd;
    byte_cnt++;
  } break;

    // Second byte -- functional code received -- check errors here
  case 1u: {
    func_code = recvd;
    byte_cnt++;
  } break;

  default: {

    // Error detected
    if (func_code & 0x80) {

      // Error code received
      if (byte_cnt == 2u) {
        error_code = recvd;
        byte_cnt++;
      } else if (byte_cnt == 3u) {
        // MSB CRC byte received
        crc = recvd << 8u;
        byte_cnt++;
      } else if (byte_cnt == 4u) {
        // LSB CRC byte received
        crc |= recvd;

        // Handle error:
        if (protocol_error_handler) {
          protocol_error_handler(error_code);
        }

        byte_cnt = 0u;
        dev_id = 0u;
        func_code = 0u;
        error_code = 0u;
        crc = 0u;
      }
    } else {
      switch (func_code) {
      case MODBUS_RTU_READ_DO ... MODBUS_RTU_READ_AI: {

        // Third received byte and with read functional code
        if (byte_cnt == 2u) {
          nbyte = recvd;
          byte_cnt++;

          // Free & reallocate data buffer if needed
          if (data_buffer) {
            std::free(data_buffer);
            data_buffer = nullptr;
          }

          data_buffer = reinterpret_cast<uint8_t *>(std::calloc(nbyte, sizeof(uint8_t)));
        } else if (byte_cnt - 2u <= nbyte) {
          // Fill data buffer
          data_buffer[byte_cnt - 2u] = recvd;
          byte_cnt++;

          // Data received -- CRC left
        } else if (byte_cnt - 2u == nbyte + 1u) {
          // MSB byte of CRC received
          crc = recvd << 8u;
          byte_cnt++;
        } else if (byte_cnt - 2u == nbyte + 2u) {

          uint8_t *buffer;
          uint16_t calc_crc;

          // LSB byte of CRC received
          crc |= recvd;

          // CRC checking
          // Allocate memory to calculate CRC and check it
          buffer = reinterpret_cast<uint8_t *>(std::calloc(nbyte + 3u, sizeof(uint8_t)));
          buffer[0u] = dev_id;
          buffer[1u] = func_code;
          buffer[2u] = nbyte;

          for (uint32_t i = 0u; i < nbyte; i++) {
            buffer[3u + i] = data_buffer[i];
          }

          calc_crc = crc16(buffer, nbyte + 3u);
          std::free(buffer);

          // Modbus protocol sequence received -- create buffer and invoke "sequence received" callback
          if (modbus_sequence_recvd_callback) {
            response = reinterpret_cast<struct modbus_resp_s *>(std::malloc(sizeof(modbus_resp_s)));
            response->data = data_buffer;
            response->size = nbyte;
            modbus_sequence_recvd_callback(response, sizeof(struct modbus_resp_s));
            std::free(response);
          }

          byte_cnt = 0u;
          dev_id = 0u;
          func_code = 0u;
          nbyte = 0u;
        }
      } break;

      case MODBUS_RTU_WRITE_DO ... MODBUS_RTU_WRITE_AO: {

        // Register address MSB byte received
        if (byte_cnt == 2u) {
          reg_addr = recvd << 8u;
          // Register address LSB byte received
        } else if (byte_cnt == 3u) {
          reg_addr |= recvd;
        } else if (byte_cnt == 4u) {
          // Register value MSB byte received
          value = recvd << 8u;
        } else if (byte_cnt == 5u) {
          // Register value LSB byte received
          value |= recvd;
        } else if (byte_cnt == 6u) {
          // MSB byte of CRC received
          crc = recvd << 8u;
        } else if (byte_cnt == 7u) {
          // LSB byte of CRC received
          crc |= recvd;

          // Reset variables
          crc = 0u;
          value = 0u;
          error_code = 0u;
        }
      } break;
      }
    }
  } break;
  }
}
