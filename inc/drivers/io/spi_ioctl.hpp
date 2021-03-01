#ifndef SPI_IOCTL_HPP
#define SPI_IOCTL_HPP

#include <cstdint>
#include <cstdlib>

enum spi_ioctl_cmd_e : uint64_t {
  SPI_INIT = 0u,
  SPI_DEINIT,
  SPI_ON_RECV,
  SPI_ON_SEND,
  SPI_SELECT,
  SPI_UNSELECT,
  SPI_MISO_IRQ_ENABLE,
  SPI_MISO_IRQ_DISABLE,
  SPI_SEND_SEQ
};

enum spi_bdr_psc_e : uint32_t {
  SPI_BDRPSC_2 = 0u,
  SPI_BDRPSC_4,
  SPI_BDRPSC_8,
  SPI_BDRPSC_16,
  SPI_BDRPSC_32,
  SPI_BDRPSC_64,
  SPI_BDRPSC_128,
  SPI_BDRPSC_256
};

enum spi_mode_e : uint32_t { SPI_MASTER = 0u, SPI_SLAVE };
enum spi_datasize_e : uint32_t { SPI_DATASIZE_8B = 0u, SPI_DATASIZE_16B };
enum spi_direction_e : uint32_t { SPI_DIR_2L_FD = 0u, SPI_DIR_2L_RXD, SPI_DIR_1L_RXD, SPI_DIR_1L_TXD };
enum spi_endianess_e : uint32_t { SPI_FIRST_BIT_MSB = 0u, SPI_FIRST_BIT_LSB };
enum spi_cpha_e : uint32_t { SPI_CPHA_1EDGE = 0u, SPI_CPHA_2EDGE };
enum spi_cpol_e : uint32_t { SPI_CPOL_LOW = 0u, SPI_CPOL_HIGH };

struct spi_setup_req_s {
  uint8_t irq_priority;
  enum spi_bdr_psc_e bdr_psc;
  enum spi_cpha_e chpa;
  enum spi_cpol_e cpol;
  enum spi_datasize_e datasize;
  enum spi_direction_e direction;
  enum spi_endianess_e endianess;
  enum spi_mode_e mode;
  uint16_t crc_polynomial;
};

struct spi_callback_req_s {
  void (*callback)(const void *, size_t);
};

struct spi_select_req_s {
  uint8_t cs_no;
};

struct spi_irq_mgm_req_s {
  uint8_t irq_priority;
  void (*callback)(const void *, size_t);
};

struct spi_send_seq_req_s {
  uint16_t seq;
};

#endif /* SPI_IOCTL_HPP */
