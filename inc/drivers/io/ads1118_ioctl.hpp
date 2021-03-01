#ifndef ADS1118_IOCTL_HPP
#define ADS1118_IOCTL_HPP

#include <cstdint>
#include <cstdlib>

enum ads1118_ioctl_cmd_e : uint32_t {
  ADS1118_CONV_OPMODE = 0u,
  ADS1118_CONV_TEMPMODE,
  ADS1118_GET_VOLTAGE,
  ADS1118_GET_TEMP_TC,
  ADS1118_GET_CHIP_TEMP,
  ADS1118_GET_TEMP_DIODE,
  ADS1118_GET_TEMP_THERMISTOR
};

enum ads1118_mux_e : uint32_t {
  ADS1118_MUX_MODE_0P_1N = 0u,
  ADS1118_MUX_MODE_0P_3N,
  ADS1118_MUX_MODE_1P_3N,
  ADS1118_MUX_MODE_2P_3N,
  ADS1118_MUX_MODE_0P_GN,
  ADS1118_MUX_MODE_1P_GN,
  ADS1118_MUX_MODE_2P_GN,
  ADS1118_MUX_MODE_3P_GN
};

struct ads1118_start_conv_req_s {
  enum ads1118_mux_e mux_setting;
  void (*on_data_recvd)(const void *, size_t);
  void (*delay_fn)(uint32_t);
  uint16_t *const value;
};

struct ads1118_get_voltage_req_s {
  enum ads1118_mux_e mux_setting;
  void (*on_data_recvd)(const void *, size_t);
  void (*delay_fn)(uint32_t);
  double *const value;
};

struct ads1118_get_temp_req_s {
  void (*on_data_recvd)(const void *, size_t);
  void (*delay_fn)(uint32_t);
  double *const value;
};


#endif /* ADS1118_IOCTL_HPP */
