#ifndef MODBUS_IOCTL_HPP
#define MODBUS_IOCTL_HPP

#include <cstdint>
#include <cstdio>

enum modbus_ioctl_cmd_e : uint64_t { MODBUS_QUERY, MODBUS_RTU_ON_SEQ_RECVD, MODBUS_RTU_ON_ERR };

enum modbus_func_code_e : uint64_t {
  MODBUS_RTU_READ_DO = 0x01u,
  MODBUS_RTU_READ_DI = 0x02u,
  MODBUS_RTU_READ_AO = 0x03u,
  MODBUS_RTU_READ_AI = 0x04u,
  MODBUS_RTU_WRITE_DO = 0x05u,
  MODBUS_RTU_WRITE_AO = 0x06u,
  MODBUS_RTU_MULT_WRITE_DO = 0x0fu,
  MODBUS_RTU_MULT_WRITE_AO = 0x10u
};

struct modbus_cbk_req_s {
  void (*cbk)(const void *, size_t);
};

struct modbus_err_cbk_req_s {
  void (*cbk)(uint8_t);
};

struct modbus_req_s {
  uint8_t device_id;
  enum modbus_func_code_e func_code;
  uint16_t reg_addr;
  uint8_t *data;
  uint16_t size;
};

struct modbus_resp_s {
  uint8_t *data;
  uint16_t size;
};

#endif /* MODBUS_IOCTL_HPP */
