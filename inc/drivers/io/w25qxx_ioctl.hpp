#ifndef W25QXX_IOCTL_HPP
#define W25QXX_IOCTL_HPP

#include <cstdint>
#include <cstdlib>

enum w25qxx_ioctl_cmd_e : uint32_t {
  W25QXX_INIT = 0u,
  W25QXX_DEINIT,
  W25QXX_READ_BLOCK,
  W25QXX_WRITE_BLOCK,
  W25QXX_ERASE_BLOCK,
  W25QXX_ERASE_BULK,
  W25QXX_GET_CONF,
};

struct w25qxx_rb_req_s {
  void *const buffer;
  void *block_addr;
  size_t offset;
  size_t size;
};

struct w25qxx_wb_req_s {
  const void *buffer;
  void *block_addr;
  size_t offset;
  size_t size;
};

struct w25qxx_eb_req_s {
  void *block_addr;
};

struct w25qxx_cnf_req_s {
  size_t *block_size;
  size_t *block_count;
  size_t *page_size;
  size_t *page_count;
  size_t *sector_size;
  size_t *sector_count;
};

#endif /* W25QXX_IOCTL_HPP */
