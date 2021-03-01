#include "lfs/lfs.h"
#include "drivers/io/w25qxx_ioctl.hpp"
#include "drivers/w25qxx_driver.hpp"
#include "fio/unistd.hpp"
#include "sys/system.hpp"

extern struct sys_impl_s &sys;

static int block_device_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) {
  const struct drv_model_cmn_s *w25qxx;
  int32_t rc;

  const struct w25qxx_rb_req_s w25qxx_rb_req { .buffer = buffer, .block_addr = reinterpret_cast<void *>(block), .offset = off, .size = size };

  if (!(w25qxx = sys.drv("w25qxx"))) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::open(&sys, "w25qxx/w25qxx_flash", 3, 3u)) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // W25X_Read((uint8_t *)buffer, (block * c->block_size + off), size);
  if ((rc = ::ioctl(&sys, "w25qxx/w25qxx_flash", w25qxx_ioctl_cmd_e::W25QXX_READ_BLOCK, &w25qxx_rb_req, sizeof(w25qxx_rb_req))) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, "w25qxx/w25qxx_flash")) < 0) {
    w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int block_device_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) {
  const struct drv_model_cmn_s *w25qxx;
  int32_t rc;

  const struct w25qxx_wb_req_s w25qxx_wb_req { .buffer = buffer, .block_addr = reinterpret_cast<void *>(block), .offset = off, .size = size };

  if (!(w25qxx = sys.drv("w25qxx"))) {
	w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::open(&sys, "w25qxx/w25qxx_flash", 3, 3u)) < 0) {
	w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, "w25qxx/w25qxx_flash", w25qxx_ioctl_cmd_e::W25QXX_WRITE_BLOCK, &w25qxx_wb_req, sizeof(w25qxx_wb_req))) < 0) {
	w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, "w25qxx/w25qxx_flash")) < 0) {
	w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int block_device_erase(const struct lfs_config *c, lfs_block_t block) {
  const struct drv_model_cmn_s *w25qxx;
  int32_t rc;

  const struct w25qxx_eb_req_s w25qxx_eb_req { .block_addr = reinterpret_cast<void *>(block) };

  if (!(w25qxx = sys.drv("w25qxx"))) {
	w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::open(&sys, "w25qxx/w25qxx_flash", 3, 3u)) < 0) {
	w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, "w25qxx/w25qxx_flash", w25qxx_ioctl_cmd_e::W25QXX_ERASE_BLOCK, &w25qxx_eb_req, sizeof(w25qxx_eb_req))) < 0) {
	w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, "w25qxx/w25qxx_flash")) < 0) {
	w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  return rc;
error:
  return -1;
}

static int block_device_sync(const struct lfs_config *c) { return 0; }

// variables used by the filesystem
lfs_t lfs;
lfs_file_t file;

// configuration of the filesystem is provided by this struct
struct lfs_config lfs_cfg = {
    .read = block_device_read, .prog = block_device_prog, .erase = block_device_erase, .sync = block_device_sync,

    // block device configuration
    // .read_size = 16,
    // .prog_size = 16,
    // .block_size = 4096,
    // .block_count = 128,
    // .block_cycles = 500,
    // .cache_size = 16,
    // .lookahead_size = 16,
};

int32_t lfs_config() {
  const struct drv_model_cmn_s *w25qxx;
  int32_t rc;
  size_t block_size, page_size, sector_size, block_count, page_count, sector_count;

  struct w25qxx_cnf_req_s w25qxx_cnf_req {
    .block_size = &block_size, .block_count = &block_count, .page_size = &page_size, .page_count = &page_count, .sector_size = &sector_size, .sector_count = &sector_count,
  };

  if (!(w25qxx = sys.drv("w25qxx"))) {
	w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::open(&sys, "w25qxx/w25qxx_flash", 3, 3u)) < 0) {
	w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::ioctl(&sys, "w25qxx/w25qxx_flash", w25qxx_ioctl_cmd_e::W25QXX_GET_CONF, &w25qxx_cnf_req, sizeof(w25qxx_cnf_req))) < 0) {
	w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  if ((rc = ::close(&sys, "w25qxx/w25qxx_flash")) < 0) {
	w25qxx_printf("ERROR: %s:%i\r\n", __FILE__, __LINE__);
    goto error;
  }

  // block device configuration
  lfs_cfg.read_size = 16;
  lfs_cfg.prog_size = 16;
  lfs_cfg.block_size = block_size;
  lfs_cfg.block_count = block_count;
  lfs_cfg.block_cycles = 500;
  lfs_cfg.cache_size = 16;
  lfs_cfg.lookahead_size = 16;

  // lfs_cfg.read_buffer = lfs_read_buf;
  // lfs_cfg.prog_buffer = lfs_prog_buf;
  // lfs_cfg.lookahead_buffer = lfs_lookahead_buf;
  // lfs_cfg.file_buffer = lfs_file_buf;

  return rc;
error:
  return -1;
}
