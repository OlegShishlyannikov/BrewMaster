#include <cmath>
#include <cstdarg>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "arch/mcu_tags.hpp"
#include "boards/board_tags.hpp"
#include "drivers/ads1118_driver.hpp"
#include "drivers/io/ads1118_ioctl.hpp"
#include "drivers/io/gpio_ioctl.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "drivers/io/spi_ioctl.hpp"
#include "drivers/io/usart_ioctl.hpp"
#include "fio/unistd.hpp"

static const struct drv_model_cmn_s *drv_ptr;

extern xQueueHandle events_worker_queue;
static xQueueHandle ads1118_fifo;
static xSemaphoreHandle ads1118_lock;
extern bool debug_log_enabled;

static constexpr const uint32_t ads1118_fifo_length = 8u;
static constexpr const uint32_t ADS1118_SPI_miso_pin_no = 4u;

enum ads1118_cmd_set_e : uint16_t {
  ADC_SS_RESET_MODE = 0b0,
  ADC_SS_START_MODE = 0b1,
  ADC_SS_FALSE_MODE = 0b0,
  ADS1118_SS_POS = 15,
  ADS1118_SS_BITS = 1,

  ADC_MUX_RESET_MODE = 0b000,
  ADC_MUX_0P_1N_MODE = 0b000,
  ADC_MUX_0P_3N_MODE = 0b001,
  ADC_MUX_1P_3N_MODE = 0b010,
  ADC_MUX_2P_3N_MODE = 0b011,
  ADC_MUX_0P_GN_MODE = 0b100,
  ADC_MUX_1P_GN_MODE = 0b101,
  ADC_MUX_2P_GN_MODE = 0b110,
  ADC_MUX_3P_GN_MODE = 0b111,
  ADS1118_MUX_POS = 12,
  ADS1118_MUX_BITS = 3,

  ADC_PGA_RESET_MODE = 0b010,
  ADC_PGA_FSR_6144_MODE = 0b000,
  ADC_PGA_FSR_4096_MODE = 0b001,
  ADC_PGA_FSR_2048_MODE = 0b010,
  ADC_PGA_FSR_1024_MODE = 0b011,
  ADC_PGA_FSR_512_MODE = 0b100,
  ADC_PGA_FSR_256_MODE = 0b101,
  ADC_PGA_FSR_128_MODE = 0b110,
  ADC_PGA_FSR_64_MODE = 0b111,
  ADS1118_PGA_POS = 9,
  ADS1118_PGA_BITS = 3,

  ADC_OPMODE_RESET_MODE = 0b1,
  ADC_OPMODE_CC_MODE = 0b0,
  ADC_OPMODE_SS_PWRDWN_MODE = 0b1,
  ADS1118_OPMODE_POS = 8,
  ADS1118_OPMODE_BITS = 1,

  ADC_DR_RESET_MODE = 0b100,
  ADC_DR_8_MODE = 0b000,
  ADC_DR_16_MODE = 0b001,
  ADC_DR_32_MODE = 0b010,
  ADC_DR_64_MODE = 0b011,
  ADC_DR_128_MODE = 0b100,
  ADC_DR_250_MODE = 0b101,
  ADC_DR_475_MODE = 0b110,
  ADC_DR_860_MODE = 0b111,
  ADS1118_DR_POS = 5,
  ADS1118_DR_BITS = 3,

  ADC_TSMODE_RESET_MODE = 0b0,
  ADC_TSMODE_ADC_MODE = 0b0,
  ADC_TSMODE_TEMP_MODE = 0b1,
  ADS1118_TSMODE_POS = 4,
  ADS1118_TSMODE_BITS = 1,

  ADC_PULLUP_RESET_MODE = 0b1,
  ADC_PULLUP_DIS_MODE = 0b0,
  ADC_PULLUP_EN_MODE = 0b1,
  ADS1118_PULLUPEN_POS = 3,
  ADS1118_PULLUPEN_BITS = 1,

  ADC_NOP_RESET_MODE = 0b01,
  ADC_NOP_INVALID_DATA_MODE = 0b00,
  ADC_NOP_VALID_DATA_MODE = 0b01,
  ADS1118_NOP_POS = 1,
  ADS1118_NOP_BITS = 2,

  ADC_RDY_FLAG_MODE = 0b0,
  ADS1118_RDY_FLAG_POS = 0,
  ADS1118_RDY_FLAG_BITS = 1
};

static constexpr double ADS1118_CONST_6_144V_LSB_mV = 0.1875f;
static constexpr double ADS1118_CONST_4_096V_LSB_mV = 0.125f;
static constexpr double ADS1118_CONST_2_048V_LSB_mV = 0.0625f;
static constexpr double ADS1118_CONST_1_024V_LSB_mV = 0.03125f;
static constexpr double ADS1118_CONST_0_512V_LSB_mV = 0.015625f;
static constexpr double ADS1118_CONST_0_256V_LSB_mV = 0.0078125f;

static enum ads1118_cmd_set_e ads1118_mux_opts[]{ADC_MUX_RESET_MODE, ADC_MUX_0P_1N_MODE, ADC_MUX_0P_3N_MODE, ADC_MUX_1P_3N_MODE, ADC_MUX_2P_3N_MODE,
                                                 ADC_MUX_0P_GN_MODE, ADC_MUX_1P_GN_MODE, ADC_MUX_2P_GN_MODE, ADC_MUX_3P_GN_MODE};

static struct ads1118_conf_s {
  uint8_t ss : ADS1118_SS_BITS = ADC_SS_RESET_MODE;
  uint8_t mux : ADS1118_MUX_BITS = ADC_MUX_RESET_MODE;
  uint8_t pga : ADS1118_PGA_BITS = ADC_PGA_RESET_MODE;
  uint8_t opmode : ADS1118_OPMODE_BITS = ADC_OPMODE_RESET_MODE;
  uint8_t dr : ADS1118_DR_BITS = ADC_DR_RESET_MODE;
  uint8_t ts_mode : ADS1118_TSMODE_BITS = ADC_TSMODE_RESET_MODE;
  uint8_t pullup_en : ADS1118_PULLUPEN_BITS = ADC_PULLUP_RESET_MODE;
  uint8_t nop : ADS1118_NOP_BITS = ADC_NOP_RESET_MODE;
  uint8_t rdy_flag : ADS1118_RDY_FLAG_BITS = ADC_RDY_FLAG_MODE;
} ads1118_cnf;

static uint16_t ads1118_serialize_conf();
static int32_t ads1118_deserialize_conf(uint16_t, struct ads1118_conf_s *const);
static int32_t ads1118_printf(const char *fmt, ...);

// static void ads1118_spi_miso_irq_callback(const void *, size_t);
// static void (*ads1118_spi_miso_irq_callback_nested)(const void *, size_t);

static int32_t ads1118_drv_open(int32_t, mode_t);
static int32_t ads1118_drv_ioctl(uint64_t, const void *, size_t);
static int32_t ads1118_drv_read(void *const, size_t);
static int32_t ads1118_drv_write(const void *, size_t);
static int32_t ads1118_drv_close();

/* Ads1118 helper functions */
static int32_t ads1118_flock() {
  BaseType_t rc;
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = xSemaphoreTakeRecursive(ads1118_lock, portIO_MAX_DELAY)) != pdPASS) {
    // errno = ???
    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t ads1118_funlock() {
  BaseType_t rc;
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = xSemaphoreGiveRecursive(ads1118_lock)) != pdPASS) {
    // errno = ???
    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

struct drv_ops_s ads1118_drv_ops {
  .init = ads1118_drv_init, .exit = ads1118_drv_exit
};

/* SPI driver file operations secification */
struct file_ops_s ads1118_drv_fops {
  .flock = ads1118_flock, .funlock = ads1118_funlock, .open = ads1118_drv_open, .ioctl = ads1118_drv_ioctl, .read = ads1118_drv_read, .write = ads1118_drv_write, .close = ads1118_drv_close
};

constexpr const size_t tc_k_lookup_table_size = sizeof(tc_k_lookup) / sizeof(tc_k_lookup[0u]);
constexpr const size_t sample_num = 8u;

static int8_t ads1118_get_temp_thermo_type_k(double, double, double *const);
static uint16_t ads1118_median_filter(uint16_t *const);
static double ads1118_median_average_filter(double *const);
static int32_t ads1118_diode_voltage_to_temp(double, double *const);
static int32_t ads1118_thermistor_voltage_to_temp(double, double *const);

void ads1118_drv_init(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *spi;
  int32_t rc, spi_fd;
  struct spi_setup_req_s spi_setup_req {
    .irq_priority = 5u, .bdr_psc = SPI_BDRPSC_2, .chpa = SPI_CPHA_2EDGE, .cpol = SPI_CPOL_LOW, .datasize = SPI_DATASIZE_16B, .direction = SPI_DIR_2L_FD, .endianess = SPI_FIRST_BIT_MSB,
    .mode = SPI_MASTER
  };

  if (!(spi = drv->dep("spi"))) {
    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((spi_fd = ::open(spi, "spi3", 3, 3u)) < 0) {
    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_INIT, &spi_setup_req, sizeof(spi_setup_req))) < 0) {
    if ((rc = ::close(spi, spi_fd)) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(spi, spi_fd)) < 0) {
    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  ads1118_drv_fops.owner = drv;

  ads1118_lock = xSemaphoreCreateRecursiveMutex();
  ads1118_fifo = xQueueCreate(ads1118_fifo_length, sizeof(double));

  drv->register_chardev("ads1118_adc", &ads1118_drv_fops);
  drv_ptr = drv;
  return;
error:
  ads1118_drv_exit(drv);
  return;
}

void ads1118_drv_exit(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *spi;
  int32_t rc, spi_fd;
  if (!(spi = drv->dep("spi"))) {
    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((spi_fd = ::open(spi, "spi3", 3, 3u)) < 0) {
    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_DEINIT, nullptr, 0u)) < 0) {
    if ((rc = ::close(spi, spi_fd)) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(spi, spi_fd)) < 0) {
    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return;

  ads1118_drv_fops.owner = nullptr;

  vSemaphoreDelete(ads1118_lock);
  vQueueDelete(ads1118_fifo);

  drv->unregister_chardev("ads1118_adc");
  drv_ptr = nullptr;
  return;
error:
  return;
}

static int32_t ads1118_drv_open(int32_t oflags, mode_t mode) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t ads1118_drv_ioctl(uint64_t req, const void *buf, size_t size) {
  const struct drv_model_cmn_s *spi, *gpio;
  int32_t rc, spi_fd, usart_fd, gpio_fd;
  uint16_t ads1118_seq, recvd, gpio_value, conv_result;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(spi = drv_ptr->dep("spi"))) {
    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(gpio = drv_ptr->dep("gpio"))) {
    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (req) {
  case ADS1118_CONV_OPMODE ... ADS1118_CONV_TEMPMODE: {
    const struct ads1118_start_conv_req_s *ads1118_conv_req = reinterpret_cast<const struct ads1118_start_conv_req_s *>(buf);
    struct ads1118_conf_s conf;
    struct spi_send_seq_req_s spi_seq_req;

    ads1118_cnf.ss = ADC_SS_START_MODE;
    ads1118_cnf.pga = ADC_PGA_FSR_4096_MODE;
    ads1118_cnf.opmode = ADC_OPMODE_SS_PWRDWN_MODE;
    ads1118_cnf.dr = ADC_DR_8_MODE;
    ads1118_cnf.ts_mode = (req == ADS1118_CONV_OPMODE) ? ADC_TSMODE_ADC_MODE : (req == ADS1118_CONV_TEMPMODE) ? ADC_TSMODE_TEMP_MODE : ADC_TSMODE_RESET_MODE;
    ads1118_cnf.pullup_en = ADC_PULLUP_EN_MODE;
    ads1118_cnf.nop = ADC_NOP_VALID_DATA_MODE;
    ads1118_cnf.rdy_flag = ADC_RDY_FLAG_MODE;
    ads1118_cnf.mux = ads1118_conv_req->mux_setting;

    if ((spi_fd = ::open(spi, "spi3", 3, 3u)) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    // Select ADS1118 chip
    if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_SELECT, nullptr, 0u)) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    // Write sequence to ADS1118 to start conversion
    ads1118_seq = ads1118_serialize_conf();
    spi_seq_req.seq = ads1118_seq;
    if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_SEND_SEQ, &spi_seq_req, sizeof(spi_seq_req))) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    // Read (result unused)
    if ((rc = ::read(spi, spi_fd, &recvd, sizeof(recvd))) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    ads1118_deserialize_conf(recvd, &conf);

    // And second time
    if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_SEND_SEQ, &spi_seq_req, sizeof(spi_seq_req))) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    // Read (result unused)
    if ((rc = ::read(spi, spi_fd, &recvd, sizeof(recvd))) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    ads1118_deserialize_conf(recvd, &conf);
    if ((gpio_fd = ::open(gpio, "B", 3, 3u)) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    do {
      if ((rc = ::read(gpio, gpio_fd, &gpio_value, sizeof(gpio_value))) < 0) {
        ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      ads1118_conv_req->delay_fn(1u);
    } while ((gpio_value >> ADS1118_SPI_miso_pin_no) & 0x01);

    if ((rc = ::close(gpio, gpio_fd)) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    // MISO IRQ disabled -- read data
    ads1118_cnf.ss = ADC_SS_RESET_MODE;
    ads1118_seq = ads1118_serialize_conf();
    spi_seq_req.seq = ads1118_seq;

    // Read data from prevous conversion
    if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_SEND_SEQ, &spi_seq_req, sizeof(spi_seq_req))) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::read(spi, spi_fd, &recvd, sizeof(recvd))) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    // Store measured data
    conv_result = recvd;
    xQueueSendToBack(ads1118_fifo, &conv_result, portIO_MAX_DELAY);

    // Read data second time for parity reasons
    if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_SEND_SEQ, &spi_seq_req, sizeof(spi_seq_req))) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    // Stored data is unused
    if ((rc = ::read(spi, spi_fd, &recvd, sizeof(recvd))) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    // Unselect ADS1118 chip
    if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_UNSELECT, nullptr, 0u)) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::close(spi, spi_fd)) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    *ads1118_conv_req->value = conv_result;
    if (ads1118_conv_req->on_data_recvd && size != 0u) {
      ads1118_conv_req->on_data_recvd(&conv_result, sizeof(conv_result));
    }
  } break;

  case ADS1118_GET_VOLTAGE: {
    int32_t rc;
    double result;
    const struct ads1118_get_voltage_req_s *req = static_cast<const struct ads1118_get_voltage_req_s *>(buf);
    uint16_t adc_val;

    struct ads1118_start_conv_req_s conv_req {
      .mux_setting = req->mux_setting, .on_data_recvd = req->on_data_recvd, .delay_fn = req->delay_fn, .value = &adc_val
    };

    if ((rc = ads1118_drv_ioctl(ads1118_ioctl_cmd_e::ADS1118_CONV_OPMODE, &conv_req, 0u)) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if (adc_val & 0x8000) {
      result = (0x3fffu - (adc_val >> 2u) + 1u) * (-1.0f);
    } else {
      result = adc_val * 1.0f;
    }

    switch (ads1118_cnf.pga) {
    case ADC_PGA_FSR_256_MODE ... ADC_PGA_FSR_64_MODE: {
      result *= ADS1118_CONST_0_256V_LSB_mV;
    } break;

    case ADC_PGA_FSR_512_MODE: {
      result *= ADS1118_CONST_0_512V_LSB_mV;
    } break;

    case ADC_PGA_FSR_1024_MODE: {
      result *= ADS1118_CONST_1_024V_LSB_mV;
    } break;

    case ADC_PGA_FSR_2048_MODE: {
      result *= ADS1118_CONST_2_048V_LSB_mV;
    } break;

    case ADC_PGA_FSR_4096_MODE: {
      result *= ADS1118_CONST_4_096V_LSB_mV;
    } break;

    case ADC_PGA_FSR_6144_MODE: {
      result *= ADS1118_CONST_6_144V_LSB_mV;
    } break;

    default:
      break;
    }

    *req->value = result;
    if (req->on_data_recvd && size != 0u) {
      req->on_data_recvd(req->value, sizeof(*req->value));
    }
  } break;

  case ADS1118_GET_TEMP_TC: {
    int32_t rc;
    double cold_junction_voltage, hot_junction_voltage, result;
    const struct ads1118_get_temp_req_s *req = static_cast<const struct ads1118_get_temp_req_s *>(buf);

    const struct ads1118_get_voltage_req_s cold_voltage_req {
      .mux_setting = ads1118_mux_e::ADS1118_MUX_MODE_2P_3N, .on_data_recvd = req->on_data_recvd, .delay_fn = req->delay_fn, .value = &cold_junction_voltage
    };

    const struct ads1118_get_voltage_req_s hot_voltage_req {
      .mux_setting = ads1118_mux_e::ADS1118_MUX_MODE_0P_1N, .on_data_recvd = req->on_data_recvd, .delay_fn = req->delay_fn, .value = &hot_junction_voltage
    };

    // Get cold junction voltage
    if ((rc = ads1118_drv_ioctl(ads1118_ioctl_cmd_e::ADS1118_GET_VOLTAGE, &cold_voltage_req, 0u)) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    // Get hot junction voltage
    if ((rc = ads1118_drv_ioctl(ads1118_ioctl_cmd_e::ADS1118_GET_VOLTAGE, &hot_voltage_req, 0u)) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    // Get value from lookup table and correct it by cold junction voltage
    if ((rc = ads1118_get_temp_thermo_type_k(hot_junction_voltage, cold_junction_voltage, &result)) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    *req->value = result;
    if (req->on_data_recvd && size != 0u) {
      req->on_data_recvd(req->value, sizeof(*req->value));
    }
  } break;

  case ADS1118_GET_CHIP_TEMP: {

  } break;

  case ADS1118_GET_TEMP_DIODE: {
    int32_t rc;
    double diode_voltage, result;
    const struct ads1118_get_temp_req_s *req = static_cast<const struct ads1118_get_temp_req_s *>(buf);

    const struct ads1118_get_voltage_req_s diode_voltage_req {
      .mux_setting = ads1118_mux_e::ADS1118_MUX_MODE_0P_1N, .on_data_recvd = req->on_data_recvd, .delay_fn = req->delay_fn, .value = &diode_voltage
    };

    // Get hot junction voltage
    if ((rc = ads1118_drv_ioctl(ads1118_ioctl_cmd_e::ADS1118_GET_VOLTAGE, &diode_voltage_req, 0u)) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    // Voltage to temperature
    if ((rc = ads1118_diode_voltage_to_temp(diode_voltage, &result)) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

  } break;

  case ADS1118_GET_TEMP_THERMISTOR: {
    int32_t rc;
    double thermistor_voltage, result;
    const struct ads1118_get_temp_req_s *req = static_cast<const struct ads1118_get_temp_req_s *>(buf);

    const struct ads1118_get_voltage_req_s cold_voltage_req {
      .mux_setting = ads1118_mux_e::ADS1118_MUX_MODE_2P_3N, .on_data_recvd = req->on_data_recvd, .delay_fn = req->delay_fn, .value = &thermistor_voltage
    };

    const struct ads1118_get_voltage_req_s hot_voltage_req {
      .mux_setting = ads1118_mux_e::ADS1118_MUX_MODE_0P_1N, .on_data_recvd = req->on_data_recvd, .delay_fn = req->delay_fn, .value = &thermistor_voltage
    };

    // Get cold junction voltage
    if ((rc = ads1118_drv_ioctl(ads1118_ioctl_cmd_e::ADS1118_GET_VOLTAGE, &cold_voltage_req, 0u)) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    // Get hot junction voltage
    if ((rc = ads1118_drv_ioctl(ads1118_ioctl_cmd_e::ADS1118_GET_VOLTAGE, &hot_voltage_req, 0u)) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    // Get value from lookup table and correct it by cold junction voltage
    if ((rc = ads1118_thermistor_voltage_to_temp(thermistor_voltage, &result)) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    *req->value = result;
    if (req->on_data_recvd && size != 0u) {
      req->on_data_recvd(req->value, sizeof(*req->value));
    }

  } break;

  default:
    break;
  }

  return 0;
error:
  return -1;
}

static int32_t ads1118_drv_read(void *const buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  BaseType_t rc;

  for (size_t nbyte = 0u; nbyte < size / sizeof(uint16_t); nbyte++) {
    if ((rc = xQueueReceive(ads1118_fifo, reinterpret_cast<uint16_t *const>(buf) + nbyte, portIO_MAX_DELAY)) != pdPASS) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  return 0;
error:
  return -1;
}

// Nothing to write here
static int32_t ads1118_drv_write(const void *buf, size_t size) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t ads1118_drv_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static uint16_t ads1118_serialize_conf() {
  uint16_t res = (ads1118_cnf.ss << ADS1118_SS_POS) | (ads1118_cnf.mux << ADS1118_MUX_POS) | (ads1118_cnf.pga << ADS1118_PGA_POS) | (ads1118_cnf.opmode << ADS1118_OPMODE_POS) |
                 (ads1118_cnf.dr << ADS1118_DR_POS) | (ads1118_cnf.ts_mode << ADS1118_TSMODE_POS) | (ads1118_cnf.pullup_en << ADS1118_PULLUPEN_POS) | (ads1118_cnf.nop << ADS1118_NOP_POS) |
                 (ads1118_cnf.rdy_flag << ADS1118_RDY_FLAG_POS);

  return res;
error:
  return 0u;
}

static int32_t ads1118_deserialize_conf(uint16_t from, struct ads1118_conf_s *const to) {
  to->ss = (from >> ADS1118_SS_POS) & ~(0xffff << ADS1118_SS_BITS);
  to->mux = (from >> ADS1118_MUX_POS) & ~(0xffff << ADS1118_MUX_BITS);
  to->pga = (from >> ADS1118_PGA_POS) & ~(0xffff << ADS1118_PGA_BITS);
  to->opmode = (from >> ADS1118_OPMODE_POS) & ~(0xffff << ADS1118_OPMODE_BITS);
  to->dr = (from >> ADS1118_DR_POS) & ~(0xffff << ADS1118_DR_BITS);
  to->ts_mode = (from >> ADS1118_TSMODE_POS) & ~(0xffff << ADS1118_TSMODE_BITS);
  to->pullup_en = (from >> ADS1118_PULLUPEN_POS) & ~(0xffff << ADS1118_PULLUPEN_BITS);
  to->nop = (from >> ADS1118_NOP_POS) & ~(0xffff << ADS1118_NOP_BITS);
  to->rdy_flag = (from >> ADS1118_RDY_FLAG_POS) & ~(0xffff << ADS1118_RDY_FLAG_BITS);

  from = (ads1118_cnf.ss << ADS1118_SS_POS) | (ads1118_cnf.mux << ADS1118_MUX_POS) | (ads1118_cnf.pga << ADS1118_PGA_POS) | (ads1118_cnf.opmode << ADS1118_OPMODE_POS) |
         (ads1118_cnf.dr << ADS1118_DR_POS) | (ads1118_cnf.ts_mode << ADS1118_TSMODE_POS) | (ads1118_cnf.pullup_en << ADS1118_PULLUPEN_POS) | (ads1118_cnf.nop << ADS1118_NOP_POS) |
         (ads1118_cnf.rdy_flag << ADS1118_RDY_FLAG_POS);

  return 0;
error:
  return -1;
}

static int32_t ads1118_printf(const char *fmt, ...) {
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

  if (!(usart = drv_ptr->dep("usart"))) {
    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (strlen) {
    if ((usart_fd = ::open(usart, "usart1", 3, 3u)) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, "[ads1118] : ", std::strlen("[ads1118] : "))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, temp, std::strlen(temp))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::close(usart, usart_fd)) < 0) {
      ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
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

int8_t ads1118_get_temp_thermo_type_k(double input_voltage_mv, double input_cold_junction_mv, double *output_hot_junction_k) {
  const struct tc_k_lookup_table_entry_s *lookup_entry = nullptr;
  size_t tc_k_lookup_table_size = sizeof(tc_k_lookup) / sizeof(tc_k_lookup[0u]);
  double range, offset, temp;

  // Get temperature from lookup table by measured voltage and interpolate
  for (uint32_t i = 0u; i < tc_k_lookup_table_size; i++) {
    for (uint32_t j = 0u; j < sizeof(tc_k_lookup[i].mv) / sizeof(tc_k_lookup[i].mv[0u]); j++) {
      if (j != 0u) {
        if (input_voltage_mv > 0u) {
          if (input_voltage_mv > tc_k_lookup[i].mv[j - 1u] && input_voltage_mv < tc_k_lookup[i].mv[j]) {

            range = std::fabs(tc_k_lookup[i].mv[j - 1u] - tc_k_lookup[i].mv[j]);
            offset = (std::fabs(input_voltage_mv - tc_k_lookup[i].mv[j - 1u]) / range);

            lookup_entry = &tc_k_lookup[i];
            temp = lookup_entry->temp_k + j + offset;
            ads1118_printf("Got temperature: %lf, input voltage : %.3lf, cold junction : %.3lf, hot junction : %.3lf, range : %lf, offset : %lf\r\n", temp - 273, input_voltage_mv,
                           input_cold_junction_mv, input_voltage_mv, range, offset);
            break;
          } else if (input_voltage_mv == tc_k_lookup[i].mv[j]) {

            lookup_entry = &tc_k_lookup[i];
            temp = lookup_entry->temp_k + j;
            ads1118_printf("Got temperature: %lf, input voltage : %.3lf, cold junction : %.3lf, hot junction : %.3lf, range : %lf\r\n", temp - 273, input_voltage_mv, input_cold_junction_mv,
                           input_voltage_mv, range);
            break;
          } else if (input_voltage_mv == tc_k_lookup[i].mv[j - 1u]) {

            lookup_entry = &tc_k_lookup[i];
            temp = lookup_entry->temp_k + j - 1u;
            ads1118_printf("Got temperature: %lf, input voltage : %.3lf, cold junction : %.3lf, hot junction : %.3lf, range : %lf\r\n", temp - 273, input_voltage_mv, input_cold_junction_mv,
                           input_voltage_mv, range);
            break;
          }
        } else {
          if (input_voltage_mv < tc_k_lookup[i].mv[j - 1u] && input_voltage_mv > tc_k_lookup[i].mv[j]) {

            range = std::fabs(tc_k_lookup[i].mv[j - 1u] - tc_k_lookup[i].mv[j]);
            offset = (std::fabs(input_voltage_mv - tc_k_lookup[i].mv[j - 1u]) / range);

            lookup_entry = &tc_k_lookup[i];
            temp = lookup_entry->temp_k - j - offset;
            ads1118_printf("Got temperature: %lf, input voltage : %.3lf, cold junction : %.3lf, hot junction : %.3lf, range : %lf, offset : %lf\r\n", temp - 273, input_voltage_mv,
                           input_cold_junction_mv, input_voltage_mv, range, offset);
            break;
          } else if (input_voltage_mv == tc_k_lookup[i].mv[j]) {

            lookup_entry = &tc_k_lookup[i];
            temp = lookup_entry->temp_k - j;
            ads1118_printf("Got temperature: %lf, input voltage : %.3lf, cold junction : %.3lf, hot junction : %.3lf, range : %lf\r\n", temp - 273, input_voltage_mv, input_cold_junction_mv,
                           input_voltage_mv, range);
            break;
          } else if (input_voltage_mv == tc_k_lookup[i].mv[j - 1u]) {

            lookup_entry = &tc_k_lookup[i];
            temp = lookup_entry->temp_k - j - 1u;
            ads1118_printf("Got temperature: %lf, input voltage : %.3lf, cold junction : %.3lf, hot junction : %.3lf, range : %lf\r\n", temp - 273, input_voltage_mv, input_cold_junction_mv,
                           input_voltage_mv, range);
            break;
          }
        }
      }
    }
  }

  if (!lookup_entry) {
    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Compensate
  if (temp >= tc_compensation_offset_lookup[0u].temp_k && temp <= tc_compensation_offset_lookup[sizeof(tc_compensation_offset_lookup) / sizeof(tc_compensation_offset_lookup[0u]) - 1u].temp_k) {
    for (uint32_t i = 1u; i < sizeof(tc_compensation_offset_lookup) / sizeof(tc_compensation_offset_lookup[0u]); i++) {
      if (temp > tc_compensation_offset_lookup[i - 1u].temp_k && temp < tc_compensation_offset_lookup[i].temp_k) {

        ads1118_printf("Compensation: +(%.3lf)\r\n", tc_compensation_offset_lookup[i - 1u].value);
        temp += tc_compensation_offset_lookup[i].value;
        break;
      } else if (temp == tc_compensation_offset_lookup[i - 1u].temp_k) {

        ads1118_printf("Compensation: +(%.3lf)\r\n", tc_compensation_offset_lookup[i - 1u].value);
        temp += tc_compensation_offset_lookup[i].value;
        break;
      } else if (temp == tc_compensation_offset_lookup[i].temp_k) {

        ads1118_printf("Compensation: +(%.3lf)\r\n", tc_compensation_offset_lookup[i - 1u].value);
        temp += tc_compensation_offset_lookup[i].value;
        break;
      }
    }
  }

  // while (i < (lookup_size - 1u)) {
  //   if (input_cold_junction_k <= lookup_table[i].temp_k) {
  //     break;
  //   }

  //   i++;
  // }

  // total_mv = lookup_table[i - 1].mv[0u] + (lookup_table[i].mv - lookup_table[i - 1].mv) * ((input_cold_junction_k - lookup_table[i - 1].temp_k) / (lookup_table[i].temp_k - lookup_table[i -
  // 1].temp_k)); total_mv += input_voltage_mv;

  // if ((total_mv < lookup_table[0].mv) || (total_mv > lookup_table[lookup_size - 1].mv)) {
  //   *output_hot_junction_k = 0;
  //   return -1;
  // }

  // i = 1;
  // while (i < (lookup_size - 1)) {
  //   if (total_mv <= lookup_table[i].mv) {
  //     break;
  //   }

  //   i++;
  // }

  // *output_hot_junction_k = lookup_table[i - 1].temp_k + (lookup_table[i].temp_k - lookup_table[i - 1].temp_k) * ((total_mv - lookup_table[i - 1].mv) / (lookup_table[i].mv - lookup_table[i -
  // 1].mv));

  *output_hot_junction_k = temp;
  return 0;

error:
  return -1;
}

static uint16_t ads1118_median_filter(uint16_t *pbuffer) {
  uint16_t value_buf_input[sample_num];
  uint8_t i, j, temp;
  std::memcpy(value_buf_input, pbuffer, sample_num);

  for (j = 0; j <= sample_num; j++) {
    for (i = 0; i <= sample_num - j; i++) {
      if (value_buf_input[i] > value_buf_input[i + 1]) {
        temp = value_buf_input[i];
        value_buf_input[i] = value_buf_input[i + 1];
        value_buf_input[i + 1] = temp;
      }
    }
  }
  return value_buf_input[(sample_num - 1) / 2];
}

static double ads1118_median_average_filter(double *pbuffer) {
  uint8_t count, i, j;
  double value_buf_input[sample_num];
  double sum = 0, temp;
  std::memcpy(value_buf_input, pbuffer, sample_num);

  for (j = 0; j < sample_num - 1; j++) {
    for (i = 0; i < sample_num - j; i++) {
      if (value_buf_input[i] > value_buf_input[i + 1]) {
        temp = value_buf_input[i];
        value_buf_input[i] = value_buf_input[i + 1];
        value_buf_input[i + 1] = temp;
      }
    }
  }

  for (count = 1; count < sample_num - 1; count++) {
    sum += value_buf_input[count];
  }

  return (double)(sum / (sample_num - 2));
}

static int32_t ads1118_diode_voltage_to_temp(double voltage, double *const temp) {
  const double k = 1.234f;
  const double offset = 12.23f;

  if (temp) {
    *temp = voltage * k + offset;
    return 0;
  } else {
    return -1;
  }
}

static int32_t ads1118_thermistor_voltage_to_temp(double voltage_mv, double *const temp) {

  // Supply voltage_mv
  static constexpr const double supply_voltage_mv = 3300.0f;

  // NTC thermistor voltage_mv at 25 celsius degrees
  static constexpr const double t25_voltage_mv = 1225.0f;

  const struct thermistor_lookup_table_entry_s *lookup_entry = nullptr;
  size_t thermistor_lookup_table_size = sizeof(thermistor_lookup) / sizeof(thermistor_lookup[0u]);
  double range, offset, temperature, rel = voltage_mv / t25_voltage_mv;

  // Get temperature from lookup table by measured voltage_mv and interpolate
  for (uint32_t i = 1u; i < thermistor_lookup_table_size; i++) {
    if (rel <= thermistor_lookup[i - 1u].rel && rel >= thermistor_lookup[i].rel) {

      range = std::fabs(thermistor_lookup[i - 1u].rel - thermistor_lookup[i].rel);
      offset = (std::fabs(thermistor_lookup[i - 1u].rel - rel) / range) * std::abs(thermistor_lookup[i].temp_k - thermistor_lookup[i - 1u].temp_k);

      lookup_entry = &thermistor_lookup[i - 1u];
      temperature = lookup_entry->temp_k + offset;
      ads1118_printf("Got temperature: %lf, voltage_mv : %.3lf, range : %lf, offset : %lf\r\n", temperature - 273, voltage_mv, range, offset);
      break;
    }
  }

  if (!lookup_entry) {
    ads1118_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  *temp = temperature;
  return 0;

error:
  return -1;
}
