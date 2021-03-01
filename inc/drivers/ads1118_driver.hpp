#ifndef ADS1118_DRIVER_HPP
#define ADS1118_DRIVER_HPP

#include "driver.hpp"

/* Driver specific init - exit functions */
void ads1118_drv_init(const struct drv_model_cmn_s *);
void ads1118_drv_exit(const struct drv_model_cmn_s *);

extern struct drv_ops_s ads1118_drv_ops;

struct ads1118_drv_name_s {
  static constexpr const char *const name = "ads1118";
};

struct tc_k_lookup_table_entry_s {
  int16_t temp_k;
  double mv[11u];
};

struct thermistor_lookup_table_entry_s {
  int16_t temp_k;
  double rel;
};

struct compensation_offset_lookup_table_entry_s {
  int16_t temp_k;
  double value;
  double k;
};

extern const struct tc_k_lookup_table_entry_s tc_k_lookup[164u];
extern const struct compensation_offset_lookup_table_entry_s tc_compensation_offset_lookup[107u];
extern const struct thermistor_lookup_table_entry_s thermistor_lookup[43u];

#endif /* ADS1118_DRIVER_HPP */
