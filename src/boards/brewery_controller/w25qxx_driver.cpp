#include <cstdarg>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "arch/mcu_tags.hpp"
#include "boards/board_tags.hpp"
#include "drivers/io/gpio_ioctl.hpp"
#include "drivers/io/rcc_ioctl.hpp"
#include "drivers/io/spi_ioctl.hpp"
#include "drivers/io/usart_ioctl.hpp"
#include "drivers/io/w25qxx_ioctl.hpp"
#include "drivers/w25qxx_driver.hpp"
#include "fio/unistd.hpp"

static const struct drv_model_cmn_s *drv_ptr;

extern xQueueHandle events_worker_queue;
static xSemaphoreHandle w25qxx_lock;
extern bool debug_log_enabled;

static constexpr const uint32_t w25qxx_nwp_pin_no = 3u;
static const void *addr;

enum w25qxx_id_e : uint16_t {
  W25Q10 = 1,
  W25Q20,
  W25Q40,
  W25Q80,
  W25Q16,
  W25Q32,
  W25Q64,
  W25Q128,
  W25Q256,
  W25Q512,
};

static struct w25qxx_conf_s {
  uint16_t id;
  uint8_t uniq_id[8u];
  uint16_t page_size;
  uint32_t page_count;
  uint32_t sector_size;
  uint32_t sector_count;
  uint32_t block_size;
  uint32_t block_count;
  uint32_t capacity_kb;
  uint8_t status_reg1;
  uint8_t status_reg2;
  uint8_t status_reg3;
  uint8_t lock;
} w25qxx_conf;

enum w25qxx_cmd_set_e : uint16_t {
  W25QXX_W25Q10_DEV_ID = 0x4011,
  W25QXX_W25Q20_DEV_ID = 0x4012,
  W25QXX_W25Q40_DEV_ID = 0x4013,
  W25QXX_W25Q80_DEV_ID = 0x4014,
  W25QXX_W25Q16_DEV_ID = 0x4015,
  W25QXX_W25Q32_DEV_ID = 0x4016,
  W25QXX_W25Q64_DEV_ID = 0x4017,
  W25QXX_W25Q128_DEV_ID = 0x4018,
  W25QXX_W25Q256_DEV_ID = 0x4019,
  W25QXX_W25Q512_DEV_ID = 0x401a,

  W25QXX_CMD_WriteEnable = 0x06,
  W25QXX_CMD_WriteDisable = 0x04,

  W25QXX_CMD_WriteStatusReg1 = 0x01,
  W25QXX_CMD_WriteStatusReg2 = 0x31,
  W25QXX_CMD_WriteStatusReg3 = 0x11,

  W25QXX_CMD_PageProgram = 0x02,
  W25QXX_CMD_QuadPageProgram = 0x32,

  W25QXX_CMD_BlockErase64 = 0xd8,
  W25QXX_CMD_BlockErase32 = 0x52,
  W25QXX_CMD_ChipErase = 0xc7,
  W25QXX_CMD_SectorErase = 0x20,
  W25QXX_CMD_EraseSuspend = 0x75,
  W25QXX_CMD_EraseResume = 0x7a,

  W25QXX_CMD_ReadStatusReg1 = 0x05,
  W25QXX_CMD_ReadStatusReg2 = 0x35,
  W25QXX_CMD_ReadStatusReg3 = 0x15,

  W25QXX_CMD_HighPerformMode = 0xa3,
  W25QXX_CMD_ContiReadModeRet = 0xff,

  W25QXX_CMD_WakeUp = 0xab,
  W25QXX_CMD_JedecDeviceId = 0x9f,
  W25QXX_CMD_ManufactDeviceId = 0x90,
  W25QXX_CMD_ReadUniqueId = 0x4b,

  W25QXX_PowerDown = 0xb9,

  W25QXX_CMD_ReadData = 0x03,
  W25QXX_CMD_FastRead = 0x0b,
  W25QXX_CMD_FastReadDualOut = 0x3b,
  W25QXX_CMD_FastReadDualIO = 0xbb,
  W25QXX_CMD_FastReadQuadOut = 0x6b,
  W25QXX_CMD_FastReadQuadIO = 0xeb,
  W25QXX_CMD_OctalWordRead = 0xe3,

  W25QXX_DummyByte = 0xff,
  W25QXX_SectorSize = 0x1000,
  W25QXX_PageSize = 0x100,
  W25QXX_PagesCount = 0xffff,
};

enum w25qxx_sr_no_e : uint32_t { W25QXX_SR0 = 0u, W25QXX_SR1, W25QXX_SR2 };

/* W25qxx helper functions */
static int32_t w25qxx_flock() {
  BaseType_t rc;
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = xSemaphoreTakeRecursive(w25qxx_lock, portIO_MAX_DELAY)) != pdPASS) {
    // errno = ???
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t w25qxx_funlock() {
  BaseType_t rc;
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = xSemaphoreGiveRecursive(w25qxx_lock)) != pdPASS) {
    // errno = ???
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

/* Low level functions */
static int32_t w25qxx_ll_init(const struct drv_model_cmn_s *, int32_t);
static int32_t w25qxx_ll_deinit(const struct drv_model_cmn_s *, int32_t);

static int32_t w25qxx_ll_read_uniq_id(const struct drv_model_cmn_s *, int32_t, uint8_t (&)[8u]);
static int32_t w25qxx_ll_read_id(const struct drv_model_cmn_s *, int32_t, uint16_t *const);

static int32_t w25qxx_ll_read_sr(const struct drv_model_cmn_s *, int32_t, enum w25qxx_sr_no_e, uint16_t *const);
static int32_t w25qxx_ll_write_sr(const struct drv_model_cmn_s *, int32_t, enum w25qxx_sr_no_e, uint8_t);

static int32_t w25qxx_ll_send_byte(const struct drv_model_cmn_s *, int32_t, uint8_t, uint8_t *const);
static int32_t w25qxx_ll_write_enable(const struct drv_model_cmn_s *, int32_t);
static int32_t w25qxx_ll_write_disable(const struct drv_model_cmn_s *, int32_t);
static int32_t w25qxx_ll_wait_for_write_end(const struct drv_model_cmn_s *, int32_t);
static int32_t w25qxx_ll_chip_select(const struct drv_model_cmn_s *, int32_t);
static int32_t w25qxx_ll_chip_unselect(const struct drv_model_cmn_s *, int32_t);

static int32_t w25qxx_ll_erase_sector(const struct drv_model_cmn_s *, int32_t, const void *);
static int32_t w25qxx_ll_erase_bulk(const struct drv_model_cmn_s *, int32_t);
static int32_t w25qxx_ll_erase_block(const struct drv_model_cmn_s *, int32_t, size_t);

static int32_t w25qxx_ll_is_empty_block(const struct drv_model_cmn_s *, int32_t, uint32_t, uint32_t, uint32_t);
static int32_t w25qxx_ll_is_empty_page(const struct drv_model_cmn_s *, int32_t, uint32_t, uint32_t, uint32_t);
static int32_t w25qxx_ll_is_empty_sector(const struct drv_model_cmn_s *, int32_t, uint32_t, uint32_t, uint32_t);

static int32_t w25qxx_ll_read_block(const struct drv_model_cmn_s *, int32_t, uint8_t *, uint32_t, uint32_t, uint32_t);
static int32_t w25qxx_ll_read_byte(const struct drv_model_cmn_s *, int32_t, uint8_t *, uint32_t);
static int32_t w25qxx_ll_read_bytes(const struct drv_model_cmn_s *, int32_t, uint8_t *, uint32_t, uint32_t);
static int32_t w25qxx_ll_read_page(const struct drv_model_cmn_s *, int32_t, uint8_t *, uint32_t, uint32_t, uint32_t);
static int32_t w25qxx_ll_read_sector(const struct drv_model_cmn_s *, int32_t, uint8_t *, uint32_t, uint32_t, uint32_t);

static int32_t w25qxx_ll_write_byte(const struct drv_model_cmn_s *, int32_t, uint8_t, uint32_t);
static int32_t w25qxx_ll_write_page(const struct drv_model_cmn_s *, int32_t, const uint8_t *, uint32_t, uint32_t, uint32_t);
static int32_t w25qxx_ll_write_sector(const struct drv_model_cmn_s *, int32_t, const uint8_t *, uint32_t, uint32_t, uint32_t);
static int32_t w25qxx_ll_write_block(const struct drv_model_cmn_s *, int32_t, const uint8_t *, uint32_t, uint32_t, uint32_t);

// Utility functions
static uint32_t w25qxx_ll_ut_pate_to_sec(uint32_t);
static uint32_t w25qxx_ll_ut_page_to_block(uint32_t);
static uint32_t w25qxx_ll_ut_sec_to_block(uint32_t);
static uint32_t w25qxx_ll_ut_sec_to_page(uint32_t);
static uint32_t w25qxx_ll_ut_block_to_page(uint32_t);

// Device driver file operations
static int32_t w25qxx_drv_open(int32_t, mode_t);
static int32_t w25qxx_drv_ioctl(uint64_t, const void *, size_t);
static int32_t w25qxx_drv_read(void *const, size_t);
static int32_t w25qxx_drv_write(const void *, size_t);
static int32_t w25qxx_drv_close();

// Define driver operations
struct drv_ops_s w25qxx_drv_ops {
  .init = w25qxx_drv_init, .exit = w25qxx_drv_exit
};

/* Driver file operations secification */
struct file_ops_s w25qxx_drv_fops {
  .flock = w25qxx_flock, .funlock = w25qxx_funlock, .open = w25qxx_drv_open, .ioctl = w25qxx_drv_ioctl, .read = w25qxx_drv_read, .write = w25qxx_drv_write, .close = w25qxx_drv_close
};

void w25qxx_drv_init(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *spi;
  int32_t rc, spi_fd;
  struct spi_setup_req_s spi_setup_req {
    .irq_priority = 5u, .bdr_psc = SPI_BDRPSC_2, .chpa = SPI_CPHA_1EDGE, .cpol = SPI_CPOL_LOW, .datasize = SPI_DATASIZE_8B, .direction = SPI_DIR_2L_FD, .endianess = SPI_FIRST_BIT_MSB,
    .mode = SPI_MASTER, .crc_polynomial = 7u
  };

  if (!(spi = drv->dep("spi"))) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((spi_fd = ::open(spi, "spi1", 3, 3u)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_INIT, &spi_setup_req, sizeof(spi_setup_req))) < 0) {
    if ((rc = ::close(spi, spi_fd)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  w25qxx_drv_fops.owner = drv;
  w25qxx_lock = xSemaphoreCreateRecursiveMutex();

  drv->register_blockdev("w25qxx_flash", &w25qxx_drv_fops);
  drv_ptr = drv;
  return;
error:
  w25qxx_drv_exit(drv);
  return;
}

void w25qxx_drv_exit(const struct drv_model_cmn_s *drv) {
  const struct drv_model_cmn_s *spi;
  int32_t rc, spi_fd;
  if (!(spi = drv->dep("spi"))) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((spi_fd = ::open(spi, "spi1", 3, 3u)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_DEINIT, nullptr, 0u)) < 0) {
    if ((rc = ::close(spi, spi_fd)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return;

  w25qxx_drv_fops.owner = nullptr;

  vSemaphoreDelete(w25qxx_lock);
  drv->unregister_blockdev("ads1118_adc");
  drv_ptr = nullptr;
  return;
error:
  return;
}

static int32_t w25qxx_drv_open(int32_t oflags, mode_t mode) {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t w25qxx_drv_ioctl(uint64_t req, const void *buf, size_t size) {
  const struct drv_model_cmn_s *spi;
  int32_t rc, spi_fd;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(spi = drv_ptr->dep("spi"))) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((spi_fd = ::open(spi, "spi1", 3, 3u)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (req) {
  case W25QXX_INIT: {
    if ((rc = w25qxx_ll_init(spi, spi_fd)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case W25QXX_DEINIT: {
    if ((rc = w25qxx_ll_deinit(spi, spi_fd)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case W25QXX_READ_BLOCK: {
    if (!buf) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    const struct w25qxx_rb_req_s *rb_req = reinterpret_cast<const struct w25qxx_rb_req_s *>(buf);
    if ((rc = w25qxx_ll_read_block(spi, spi_fd, reinterpret_cast<uint8_t *>(rb_req->buffer), reinterpret_cast<size_t>(rb_req->block_addr), rb_req->offset, rb_req->size)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case W25QXX_WRITE_BLOCK: {
    if (!buf) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    const struct w25qxx_wb_req_s *wb_req = reinterpret_cast<const struct w25qxx_wb_req_s *>(buf);
    if ((rc = w25qxx_ll_write_block(spi, spi_fd, reinterpret_cast<const uint8_t *>(wb_req->buffer), reinterpret_cast<size_t>(wb_req->block_addr), wb_req->offset, wb_req->size)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case W25QXX_ERASE_BLOCK: {
    if (!buf) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    const struct w25qxx_eb_req_s *eb_req = reinterpret_cast<const struct w25qxx_eb_req_s *>(buf);
    if ((rc = w25qxx_ll_erase_block(spi, spi_fd, reinterpret_cast<size_t>(eb_req->block_addr))) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case W25QXX_ERASE_BULK: {
    if ((rc = w25qxx_ll_erase_bulk(spi, spi_fd)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case W25QXX_GET_CONF: {
    if (!buf) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    const struct w25qxx_cnf_req_s *cnf_req = reinterpret_cast<const struct w25qxx_cnf_req_s *>(buf);
    if (!cnf_req->block_count) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    *cnf_req->block_count = w25qxx_conf.block_count;

    if (!cnf_req->block_size) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    *cnf_req->block_size = w25qxx_conf.block_size;

    if (!cnf_req->page_count) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    *cnf_req->page_count = w25qxx_conf.page_count;

    if (!cnf_req->page_size) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    *cnf_req->page_size = w25qxx_conf.page_size;

    if (!cnf_req->sector_count) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    *cnf_req->sector_count = w25qxx_conf.sector_count;

    if (!cnf_req->sector_size) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    *cnf_req->sector_size = w25qxx_conf.sector_size;
  } break;

  default:
    break;
  }

  if ((rc = ::close(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t w25qxx_drv_read(void *const buf, size_t size) {
  const struct drv_model_cmn_s *spi;
  int32_t rc, spi_fd;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!buf) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(spi = drv_ptr->dep("spi"))) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((spi_fd = ::open(spi, "spi1", 3, 3u)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Read here
  if ((rc = ::close(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t w25qxx_drv_write(const void *buf, size_t size) {
  const struct drv_model_cmn_s *spi;
  int32_t rc, spi_fd;

  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!buf) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (!(spi = drv_ptr->dep("spi"))) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((spi_fd = ::open(spi, "spi1", 3, 3u)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // Write here
  if ((rc = ::close(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t w25qxx_drv_close() {
  /* Check if device driver inited successfully */
  if (!drv_ptr) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return 0;
error:
  return -1;
}

static int32_t w25qxx_ll_init(const struct drv_model_cmn_s *spi, int32_t spi_fd) {
  int32_t rc;
  uint16_t id, uniq_id, sr0, sr1, sr2;

  if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd))) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_read_id(spi, spi_fd, &id)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (id) {
  case W25QXX_W25Q512_DEV_ID: {
    w25qxx_conf.id = W25QXX_W25Q512_DEV_ID;
    w25qxx_conf.block_count = 1024u;
  } break;

  case W25QXX_W25Q256_DEV_ID: {
    w25qxx_conf.id = W25QXX_W25Q256_DEV_ID;
    w25qxx_conf.block_count = 512u;
  } break;

  case W25QXX_W25Q128_DEV_ID: {
    w25qxx_conf.id = W25QXX_W25Q128_DEV_ID;
    w25qxx_conf.block_count = 256u;
  } break;

  case W25QXX_W25Q64_DEV_ID: {
    w25qxx_conf.id = W25QXX_W25Q64_DEV_ID;
    w25qxx_conf.block_count = 128u;
  } break;

  case W25QXX_W25Q32_DEV_ID: {
    w25qxx_conf.id = W25Q32;
    w25qxx_conf.block_count = 64u;
  } break;

  case W25QXX_W25Q16_DEV_ID: {
    w25qxx_conf.id = W25Q16;
    w25qxx_conf.block_count = 32;
  } break;

  case W25QXX_W25Q80_DEV_ID: {
    w25qxx_conf.id = W25QXX_W25Q80_DEV_ID;
    w25qxx_conf.block_count = 16u;
  } break;

  case W25QXX_W25Q40_DEV_ID: {
    w25qxx_conf.id = W25QXX_W25Q40_DEV_ID;
    w25qxx_conf.block_count = 8u;
  } break;

  case W25QXX_W25Q20_DEV_ID: {
    w25qxx_conf.id = W25QXX_W25Q20_DEV_ID;
    w25qxx_conf.block_count = 4u;
  } break;

  case W25QXX_W25Q10_DEV_ID: {
    w25qxx_conf.id = W25QXX_W25Q10_DEV_ID;
    w25qxx_conf.block_count = 2u;
  } break;

  default:
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
    break;
  }

  w25qxx_conf.page_size = 256u;
  w25qxx_conf.sector_size = 0x1000;
  w25qxx_conf.sector_count = w25qxx_conf.block_count * 16u;
  w25qxx_conf.page_count = (w25qxx_conf.sector_count * w25qxx_conf.sector_size) / w25qxx_conf.page_size;
  w25qxx_conf.block_size = w25qxx_conf.sector_size * 16u;
  w25qxx_conf.capacity_kb = (w25qxx_conf.sector_count * w25qxx_conf.sector_size) / 1024u;

  if ((rc = w25qxx_ll_read_uniq_id(spi, spi_fd, w25qxx_conf.uniq_id)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_read_sr(spi, spi_fd, W25QXX_SR0, &sr0)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  w25qxx_conf.status_reg1 = sr0;
  if ((rc = w25qxx_ll_read_sr(spi, spi_fd, W25QXX_SR1, &sr1)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  w25qxx_conf.status_reg2 = sr1;
  if ((rc = w25qxx_ll_read_sr(spi, spi_fd, W25QXX_SR2, &sr2)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  w25qxx_conf.status_reg3 = sr2;
  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_deinit(const struct drv_model_cmn_s *spi, int32_t spi_fd) {
  int32_t rc;

  if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd))) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  w25qxx_conf = {};
  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_read_sr(const struct drv_model_cmn_s *spi, int32_t spi_fd, enum w25qxx_sr_no_e sr_no, uint16_t *const sr) {
  uint8_t recvd_byte;
  int32_t rc;

  if (!sr) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_chip_select(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (sr_no) {
  case W25QXX_SR0: {
    if ((rc = rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_ReadStatusReg1, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_DummyByte, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    *sr = recvd_byte;
    goto success;
  } break;

  case W25QXX_SR1: {
    if ((rc = rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_ReadStatusReg2, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_DummyByte, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    *sr = recvd_byte;
    goto success;
  } break;

  case W25QXX_SR2: {
    if ((rc = rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_ReadStatusReg3, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_DummyByte, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    *sr = recvd_byte;
    goto success;
  } break;
  }

success:
  if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t w25qxx_write_sr(const struct drv_model_cmn_s *spi, int32_t spi_fd, enum w25qxx_sr_no_e sr_no, uint8_t data) {
  int32_t rc;
  uint8_t recvd_byte;

  if ((rc = w25qxx_ll_chip_select(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  switch (sr_no) {
  case W25QXX_SR0: {
    if ((rc = rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_WriteStatusReg1, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case W25QXX_SR1: {
    if ((rc = rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_WriteStatusReg2, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;

  case W25QXX_SR2: {
    if ((rc = rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_WriteStatusReg3, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  } break;
  }

  if ((rc = rc = w25qxx_ll_send_byte(spi, spi_fd, data, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_erase_sector(const struct drv_model_cmn_s *spi, int32_t spi_fd, uint32_t sec_addr) {
  int32_t rc;
  uint8_t recvd_byte;

  if ((rc = w25qxx_ll_wait_for_write_end(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_write_enable(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_chip_select(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_SectorErase, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (w25qxx_conf.id > W25QXX_W25Q256_DEV_ID) {
    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (sec_addr & 0xff000000u) >> 24u, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (sec_addr & 0xff0000u) >> 16u, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (sec_addr & 0xff00u) >> 8u, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, sec_addr & 0xffu, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_wait_for_write_end(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_erase_block(const struct drv_model_cmn_s *spi, int32_t spi_fd, size_t block_addr) {
  int32_t rc;
  uint8_t recvd_byte;

  if ((rc = w25qxx_ll_wait_for_write_end(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  block_addr = block_addr * w25qxx_conf.sector_size * 16u;

  if ((rc = w25qxx_ll_write_enable(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_chip_select(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_BlockErase64, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (w25qxx_conf.id >= W25QXX_W25Q256_DEV_ID) {
    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (block_addr & 0xff000000u) >> 24u, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (block_addr & 0xff0000u) >> 16u, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (block_addr & 0xff00u) >> 8u, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, block_addr & 0xffu, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_wait_for_write_end(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_erase_bulk(const struct drv_model_cmn_s *spi, int32_t spi_fd) {
  int32_t rc;
  uint8_t recvd_byte;

  if ((rc = w25qxx_ll_write_enable(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_chip_select(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_ChipErase, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_wait_for_write_end(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_is_empty_page(const struct drv_model_cmn_s *spi, int32_t spi_fd, uint32_t page_addr, uint32_t offset, uint32_t bytes_to_check) {
  uint8_t buffer[32u], recvd_byte;
  uint32_t work_addr;
  uint32_t i;
  int32_t rc;

  if (((bytes_to_check + offset) > w25qxx_conf.page_size) || (bytes_to_check == 0))
    bytes_to_check = w25qxx_conf.page_size - offset;

  for (i = offset; i < w25qxx_conf.page_size; i += sizeof(buffer)) {

    if ((rc = w25qxx_ll_chip_select(spi, spi_fd)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    work_addr = (i + page_addr * w25qxx_conf.page_size);

    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_FastRead, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if (w25qxx_conf.id >= W25QXX_W25Q256_DEV_ID) {
      if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (work_addr & 0xff000000u) >> 24u, &recvd_byte)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }
    }

    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (work_addr & 0xff0000u) >> 16u, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (work_addr & 0xff00u) >> 8u, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, work_addr & 0xffu, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    for (uint32_t i = 0u; i < sizeof(buffer); i++) {
      if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_DummyByte, &recvd_byte)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      buffer[i] = recvd_byte;
    }

    if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    for (uint8_t x = 0; x < sizeof(buffer); x++) {
      if (buffer[x] != 0xFF)
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((w25qxx_conf.page_size + offset) % sizeof(buffer) != 0) {
    i -= sizeof(buffer);

    for (; i < w25qxx_conf.page_size; i++) {
      if ((rc = w25qxx_ll_chip_select(spi, spi_fd)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      work_addr = (i + page_addr * w25qxx_conf.page_size);

      if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_FastRead, &recvd_byte)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      if (w25qxx_conf.id >= W25QXX_W25Q256_DEV_ID) {
        if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (work_addr & 0xff000000u) >> 24u, &recvd_byte)) < 0) {
          w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
          goto error;
        }
      }

      if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (work_addr & 0xff0000u) >> 16u, &recvd_byte)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (work_addr & 0xff00u) >> 8u, &recvd_byte)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      if ((rc = w25qxx_ll_send_byte(spi, spi_fd, work_addr & 0xffu, &recvd_byte)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_DummyByte, &recvd_byte)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      buffer[0] = recvd_byte;
      if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      if (buffer[0] != 0xff)
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_is_empty_sector(const struct drv_model_cmn_s *spi, int32_t spi_fd, uint32_t sector_addr, uint32_t offset, uint32_t bytes_to_check) {
  if ((bytes_to_check > w25qxx_conf.sector_size) || (bytes_to_check == 0))
    bytes_to_check = w25qxx_conf.sector_size;

  uint8_t buffer[32u], recvd_byte;
  uint32_t work_addr;
  uint32_t i;
  int32_t rc;

  for (i = offset; i < w25qxx_conf.sector_size; i += sizeof(buffer)) {
    if ((rc = w25qxx_ll_chip_select(spi, spi_fd)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    work_addr = (i + sector_addr * w25qxx_conf.sector_size);
    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_FastRead, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if (w25qxx_conf.id >= W25QXX_W25Q256_DEV_ID) {
      if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (work_addr & 0xff000000u) >> 24u, &recvd_byte)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }
    }

    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (work_addr & 0xff0000u) >> 16u, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (work_addr & 0xff00u) >> 8u, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, work_addr & 0xffu, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    for (uint32_t i = 0u; i < sizeof(buffer); i++) {
      if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_DummyByte, &recvd_byte)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      buffer[i] = recvd_byte;
    }

    if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    for (uint8_t x = 0; x < sizeof(buffer); x++) {
      if (buffer[x] != 0xffu)
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((w25qxx_conf.sector_size + offset) % sizeof(buffer) != 0) {
    i -= sizeof(buffer);

    for (; i < w25qxx_conf.sector_size; i++) {
      if ((rc = w25qxx_ll_chip_select(spi, spi_fd)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      work_addr = (i + sector_addr * w25qxx_conf.sector_size);
      if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_FastRead, &recvd_byte)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      if (w25qxx_conf.id >= W25QXX_W25Q256_DEV_ID) {
        if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (work_addr & 0xff000000u) >> 24u, &recvd_byte)) < 0) {
          w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
          goto error;
        }
      }

      if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (work_addr & 0xff0000u) >> 16u, &recvd_byte)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (work_addr & 0xff00u) >> 8u, &recvd_byte)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      if ((rc = w25qxx_ll_send_byte(spi, spi_fd, work_addr & 0xffu, &recvd_byte)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_DummyByte, &recvd_byte)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      buffer[0] = recvd_byte;

      if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      if (buffer[0] != 0xff)
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  return rc;

error:
  return -1;
}

static int32_t w25qxx_ll_is_empty_block(const struct drv_model_cmn_s *spi, int32_t spi_fd, uint32_t block_addr, uint32_t offset, uint32_t bytes_to_check) {
  if ((bytes_to_check > w25qxx_conf.block_size) || (bytes_to_check == 0))
    bytes_to_check = w25qxx_conf.block_size;
  uint8_t buffer[32u], recvd_byte;
  uint32_t work_addr;
  uint32_t i;
  int32_t rc;

  for (i = offset; i < w25qxx_conf.block_size; i += sizeof(buffer)) {
    if ((rc = w25qxx_ll_chip_select(spi, spi_fd)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    work_addr = (i + block_addr * w25qxx_conf.block_size);
    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_FastRead, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if (w25qxx_conf.id >= W25QXX_W25Q256_DEV_ID) {
      if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (work_addr & 0xff000000u) >> 24u, &recvd_byte)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }
    }

    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (work_addr & 0xff0000u) >> 16u, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (work_addr & 0xff00u) >> 8u, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, work_addr & 0xffu, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    for (uint32_t i = 0u; i < sizeof(buffer); i++) {
      if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_DummyByte, &recvd_byte)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      buffer[i] = recvd_byte;
    }

    if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    for (uint8_t x = 0; x < sizeof(buffer); x++) {
      if (buffer[x] != 0xff)
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((w25qxx_conf.block_size + offset) % sizeof(buffer) != 0) {
    i -= sizeof(buffer);

    for (; i < w25qxx_conf.block_size; i++) {
      if ((rc = w25qxx_ll_chip_select(spi, spi_fd)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      work_addr = (i + block_addr * w25qxx_conf.block_size);
      if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_FastRead, &recvd_byte)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      if (w25qxx_conf.id >= W25QXX_W25Q256_DEV_ID) {
        if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (work_addr & 0xff000000u) >> 24u, &recvd_byte)) < 0) {
          w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
          goto error;
        }
      }

      if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (work_addr & 0xff0000u) >> 16u, &recvd_byte)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (work_addr & 0xff00u) >> 8u, &recvd_byte)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      if ((rc = w25qxx_ll_send_byte(spi, spi_fd, work_addr & 0xffu, &recvd_byte)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_DummyByte, &recvd_byte)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      buffer[0] = recvd_byte;

      if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      if (buffer[0] != 0xff)
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }
  return true;
error:
  return false;
}

static int32_t w25qxx_ll_write_byte(const struct drv_model_cmn_s *spi, int32_t spi_fd, uint8_t buffer, uint32_t write_addr) {
  int32_t rc;
  uint8_t recvd_byte;

  if ((rc = w25qxx_ll_wait_for_write_end(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_write_enable(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_chip_select(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_PageProgram, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (w25qxx_conf.id >= W25QXX_W25Q256_DEV_ID) {
    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (write_addr & 0xff000000u) >> 24u, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (write_addr & 0xff0000u) >> 16u, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (write_addr & 0xff00u) >> 8u, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, write_addr & 0xffu, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_wait_for_write_end(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_write_page(const struct drv_model_cmn_s *spi, int32_t spi_fd, const uint8_t *buffer, uint32_t page_addr, uint32_t offset, uint32_t bytes_to_write) {
  int32_t rc;
  uint8_t recvd_byte;

  if (((bytes_to_write + offset) > w25qxx_conf.page_size) || (bytes_to_write == 0)) {
    bytes_to_write = w25qxx_conf.page_size - offset;
  }

  if ((offset + bytes_to_write) > w25qxx_conf.page_size) {
    bytes_to_write = w25qxx_conf.page_size - offset;
  }

  if ((rc = w25qxx_ll_wait_for_write_end(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_write_enable(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_chip_select(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_PageProgram, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  page_addr = (page_addr * w25qxx_conf.page_size) + offset;

  if (w25qxx_conf.id >= W25QXX_W25Q256_DEV_ID) {
    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (page_addr & 0xff000000u) >> 24u, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (page_addr & 0xff0000u) >> 16u, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (page_addr & 0xff00u) >> 8u, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, page_addr & 0xffu, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  for (uint32_t i = 0u; i < bytes_to_write; i++) {
    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, buffer[i], &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_wait_for_write_end(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_write_sector(const struct drv_model_cmn_s *spi, int32_t spi_fd, uint8_t *buffer, uint32_t sector_addr, uint32_t offset, uint32_t bytes_to_write) {
  uint32_t start_page, local_offset;
  int32_t btw, rc;
  uint8_t recvd_byte;

  if ((bytes_to_write > w25qxx_conf.sector_size) || (bytes_to_write == 0)) {
    bytes_to_write = w25qxx_conf.sector_size;
  }

  if (offset >= w25qxx_conf.sector_size) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((offset + bytes_to_write) > w25qxx_conf.sector_size) {
    btw = w25qxx_conf.sector_size - offset;

  } else {
    btw = bytes_to_write;
  }

  start_page = w25qxx_ll_ut_sec_to_page(sector_addr) + (offset / w25qxx_conf.page_size);
  local_offset = offset % w25qxx_conf.page_size;

  do {

    if ((rc = w25qxx_ll_write_page(spi, spi_fd, buffer, start_page, local_offset, btw)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    start_page++;
    btw -= w25qxx_conf.page_size - local_offset;
    buffer += w25qxx_conf.page_size - local_offset;
    local_offset = 0;
  } while (btw > 0);

  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_write_block(const struct drv_model_cmn_s *spi, int32_t spi_fd, const uint8_t *buffer, uint32_t block_addr, uint32_t offset, uint32_t bytes_to_write) {
  uint32_t start_page, local_offset;
  int32_t btw, rc;
  uint8_t recvd_bytes;

  if ((bytes_to_write > w25qxx_conf.block_size) || (bytes_to_write == 0)) {
    bytes_to_write = w25qxx_conf.block_size;
  }

  if (offset >= w25qxx_conf.block_size) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((offset + bytes_to_write) > w25qxx_conf.block_size) {
    btw = w25qxx_conf.block_size - offset;
  } else {
    btw = bytes_to_write;
  }

  start_page = w25qxx_ll_ut_block_to_page(block_addr) + (offset / w25qxx_conf.page_size);
  local_offset = offset % w25qxx_conf.page_size;

  do {

    if ((rc = w25qxx_ll_write_page(spi, spi_fd, buffer, start_page, local_offset, btw)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    start_page++;
    btw -= w25qxx_conf.page_size - local_offset;
    buffer += w25qxx_conf.page_size - local_offset;
    local_offset = 0;
  } while (btw > 0);

  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_read_byte(const struct drv_model_cmn_s *spi, int32_t spi_fd, uint8_t *buffer, uint32_t bytes_addr) {
  int32_t rc;
  uint8_t recvd_byte;

  if ((rc = w25qxx_ll_chip_select(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_FastRead, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (w25qxx_conf.id >= W25QXX_W25Q256_DEV_ID) {
    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (bytes_addr & 0xff000000u) >> 24u, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (bytes_addr & 0xff0000u) >> 16u, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (bytes_addr & 0xff00u) >> 8u, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, bytes_addr & 0xffu, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_DummyByte, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  *buffer = recvd_byte;
  if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_read_bytes(const struct drv_model_cmn_s *spi, int32_t spi_fd, uint8_t *buffer, uint32_t read_addr, uint32_t bytes_to_read) {
  int32_t rc;
  uint8_t recvd_byte;

  if ((rc = w25qxx_ll_chip_select(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_FastRead, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (w25qxx_conf.id >= W25QXX_W25Q256_DEV_ID) {
    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (read_addr & 0xff000000u) >> 24u, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (read_addr & 0xff0000u) >> 16u, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (read_addr & 0xff00u) >> 8u, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, read_addr & 0xffu, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  for (uint32_t i = 0u; i < bytes_to_read; i++) {
    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_DummyByte, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    buffer[i] = recvd_byte;
  }

  if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_read_page(const struct drv_model_cmn_s *spi, int32_t spi_fd, uint8_t *buffer, uint32_t page_addr, uint32_t offset, uint32_t bytes_to_read_up_to_page_size) {
  int32_t rc;
  uint8_t recvd_byte;

  if ((bytes_to_read_up_to_page_size > w25qxx_conf.page_size) || (bytes_to_read_up_to_page_size == 0)) {
    bytes_to_read_up_to_page_size = w25qxx_conf.page_size;
  }

  if ((offset + bytes_to_read_up_to_page_size) > w25qxx_conf.page_size) {
    bytes_to_read_up_to_page_size = w25qxx_conf.page_size - offset;
  }

  page_addr = page_addr * w25qxx_conf.page_size + offset;
  if ((rc = w25qxx_ll_chip_select(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_ReadData, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (w25qxx_conf.id >= W25QXX_W25Q256_DEV_ID) {
    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (page_addr & 0xff000000u) >> 24u, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (page_addr & 0xff0000u) >> 16u, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, (page_addr & 0xff00u) >> 8u, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, page_addr & 0xffu, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  for (uint32_t i = 0u; i < bytes_to_read_up_to_page_size; i++) {
    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_DummyByte, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    buffer[i] = recvd_byte;
  }

  if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_read_sector(const struct drv_model_cmn_s *spi, int32_t spi_fd, uint8_t *buffer, uint32_t sector_addr, uint32_t offset, uint32_t bytes_to_read_up_to_sector_size) {
  uint32_t start_page, local_offset;
  int32_t bytes_to_read, rc;
  uint8_t recvd_byte;

  if ((bytes_to_read_up_to_sector_size > w25qxx_conf.sector_size) || (bytes_to_read_up_to_sector_size == 0)) {
    bytes_to_read_up_to_sector_size = w25qxx_conf.sector_size;
  }

  if (offset >= w25qxx_conf.sector_size) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((offset + bytes_to_read_up_to_sector_size) > w25qxx_conf.sector_size) {
    bytes_to_read = w25qxx_conf.sector_size - offset;
  } else {
    bytes_to_read = bytes_to_read_up_to_sector_size;
  }

  start_page = w25qxx_ll_ut_sec_to_page(sector_addr) + (offset / w25qxx_conf.page_size);
  local_offset = offset % w25qxx_conf.page_size;

  do {

    if ((rc = w25qxx_ll_read_page(spi, spi_fd, buffer, start_page, local_offset, bytes_to_read)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    start_page++;
    bytes_to_read -= w25qxx_conf.page_size - local_offset;
    buffer += w25qxx_conf.page_size - local_offset;
    local_offset = 0u;
  } while (bytes_to_read > 0u);

  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_read_block(const struct drv_model_cmn_s *spi, int32_t spi_fd, uint8_t *buffer, uint32_t block_addr, uint32_t offset, uint32_t bytes_to_read_up_to_block_size) {

  uint32_t start_page, local_offset;
  int32_t bytes_to_read, rc;
  uint8_t recvd_byte;

  if ((bytes_to_read_up_to_block_size > w25qxx_conf.block_size) || (bytes_to_read_up_to_block_size == 0)) {
    bytes_to_read_up_to_block_size = w25qxx_conf.block_size;
  }

  if (offset >= w25qxx_conf.block_size) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((offset + bytes_to_read_up_to_block_size) > w25qxx_conf.block_size) {
    bytes_to_read = w25qxx_conf.block_size - offset;

  } else {
    bytes_to_read = bytes_to_read_up_to_block_size;
  }

  start_page = w25qxx_ll_ut_block_to_page(block_addr) + (offset / w25qxx_conf.page_size);
  local_offset = offset % w25qxx_conf.page_size;

  do {

    if ((rc = w25qxx_ll_read_page(spi, spi_fd, buffer, start_page, local_offset, bytes_to_read)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    start_page++;
    bytes_to_read -= w25qxx_conf.page_size - local_offset;
    buffer += w25qxx_conf.page_size - local_offset;
    local_offset = 0;
  } while (bytes_to_read > 0);

  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_read_id(const struct drv_model_cmn_s *spi, int32_t spi_fd, uint16_t *const id) {
  int32_t rc;
  uint32_t temp = 0u, temp0 = 0u, temp1 = 0u, temp2 = 0u;
  uint8_t recvd_byte;

  if (!id) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_chip_select(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_JedecDeviceId, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_DummyByte, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  temp0 = recvd_byte;

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_DummyByte, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  temp1 = recvd_byte;

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_DummyByte, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  temp2 = recvd_byte;

  if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  *id = (temp0 << 16) | (temp1 << 8) | temp2;
  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_read_uniq_id(const struct drv_model_cmn_s *spi, int32_t spi_fd, uint8_t (&id)[8u]) {
  int32_t rc;
  uint8_t recvd_byte;

  if ((rc = w25qxx_ll_chip_select(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_ReadUniqueId, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  for (uint8_t i = 0u; i < 4u; i++) {
    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_DummyByte, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  for (uint8_t i = 0; i < 8u; i++) {
    if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_DummyByte, &recvd_byte)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    id[i] = recvd_byte;
  }

  if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_write_enable(const struct drv_model_cmn_s *spi, int32_t spi_fd) {
  int32_t rc;
  uint8_t recvd_byte;

  if ((rc = w25qxx_ll_chip_select(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_WriteEnable, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_write_disable(const struct drv_model_cmn_s *spi, int32_t spi_fd) {
  int32_t rc;
  uint8_t recvd_byte;

  if ((rc = w25qxx_ll_chip_select(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_WriteDisable, &recvd_byte)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_wait_for_write_end(const struct drv_model_cmn_s *spi, int32_t spi_fd) {
  uint8_t recvd_byte;
  uint16_t sr0;
  int32_t rc;

  if ((rc = w25qxx_ll_chip_select(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  w25qxx_ll_send_byte(spi, spi_fd, W25QXX_CMD_ReadStatusReg1, &recvd_byte);

  do {
    if ((rc = w25qxx_ll_read_sr(spi, spi_fd, W25QXX_SR0, &sr0)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    w25qxx_conf.status_reg1 = sr0;
  } while ((w25qxx_conf.status_reg1 & W25QXX_CMD_WriteStatusReg1));

  if ((rc = w25qxx_ll_chip_unselect(spi, spi_fd)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_send_byte(const struct drv_model_cmn_s *spi, int32_t spi_fd, uint8_t byte, uint8_t *const recvd) {
  int32_t rc;
  if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_SEND_SEQ, &byte, sizeof(byte))) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (recvd) {
    if ((rc = ::read(spi, spi_fd, recvd, sizeof(*recvd))) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }
  }

  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_chip_select(const struct drv_model_cmn_s *spi, int32_t spi_fd) {
  int32_t rc;
  if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_SELECT, nullptr, 0u)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int32_t w25qxx_ll_chip_unselect(const struct drv_model_cmn_s *spi, int32_t spi_fd) {
  int32_t rc;
  if ((rc = ::ioctl(spi, spi_fd, spi_ioctl_cmd_e::SPI_UNSELECT, nullptr, 0u)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static uint32_t w25qxx_ll_ut_page_t_sec(uint32_t page_addr) { return ((page_addr * w25qxx_conf.page_size) / w25qxx_conf.sector_size); }

static uint32_t w25qxx_ll_ut_page_to_block(uint32_t page_addr) { return ((page_addr * w25qxx_conf.page_size) / w25qxx_conf.block_size); }

static uint32_t w25qxx_ll_ut_sec_to_block(uint32_t sec_addr) { return ((sec_addr * w25qxx_conf.sector_size) / w25qxx_conf.block_size); }

static uint32_t w25qxx_ll_ut_sec_to_page(uint32_t sec_addr) { return (sec_addr * w25qxx_conf.sector_size) / w25qxx_conf.page_size; }

static uint32_t w25qxx_ll_ut_block_to_page(uint32_t block_addr) { return (block_addr * w25qxx_conf.block_size) / w25qxx_conf.page_size; }

int32_t w25qxx_printf(const char *fmt, ...) {
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
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if (strlen) {
    if ((usart_fd = ::open(usart, "usart1", 3, 3u)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, "[w25qxx] : ", std::strlen("[w25qxx] : "))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::write(usart, usart_fd, temp, std::strlen(temp))) < 0) {
      if ((rc = ::close(usart, usart_fd)) < 0) {
        w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
        goto error;
      }

      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
      goto error;
    }

    if ((rc = ::close(usart, usart_fd)) < 0) {
      w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
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
