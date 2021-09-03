#ifndef MODBUS_DRIVER_HPP
#define MODBUS_DRIVER_HPP

#include "driver.hpp"

/* Driver specific init - exit functions */
void modbus_drv_init(const struct drv_model_cmn_s *);
void modbus_drv_exit(const struct drv_model_cmn_s *);

extern struct drv_ops_s modbus_drv_ops;

struct modbus_drv_name_s {
  static constexpr const char *const name = "modbus";
};

struct modbus_master_rtu_read_req_s {
  uint8_t dev_id;
  uint8_t func_code;
  uint16_t reg_addr;
  uint16_t cnt;
  uint16_t crc;
};

struct modbus_master_rtu_read_resp_s {
  uint8_t dev_id;
  uint8_t func_code;
  uint8_t cnt;
  uint8_t *data;
  uint16_t crc;
};

struct modbus_master_rtu_write_req_s {
  uint8_t dev_id;
  uint8_t func_code;
  uint16_t reg_addr;
  uint16_t value;
  uint16_t crc;
};

struct modbus_master_rtu_write_resp_s {
  uint8_t dev_id;
  uint8_t func_code;
  uint16_t reg_addr;
  uint16_t value;
  uint16_t crc;
};

struct modbus_master_rtu_mult_write_req_s {
  uint8_t dev_id;
  uint8_t func_code;
  uint16_t reg_addr;
  uint16_t reg_cnt;
  uint8_t byte_cnt;
  uint8_t *value;
  uint16_t crc;
};

struct modbus_master_rtu_mult_write_resp_s {
  uint8_t dev_id;
  uint8_t func_code;
  uint16_t reg_addr;
  uint16_t reg_cnt;
  uint16_t crc;
};

#endif /* MODBUS_DRIVER_HPP */
