#ifndef UNISTD_HPP
#define UNISTD_HPP

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <errno.h>
#include <stdarg.h>
#include <sys/types.h>

#include "sys/system.hpp"
#include "vfs/file.hpp"

/* User space file IO functions */
int32_t open(const struct sys_impl_s *sys, const char *path, int32_t oflags, mode_t mode);

/* Temporary solution while VFS isn't implemented (using path instead of FD) */
int32_t ioctl(const struct sys_impl_s *sys, const char *path, uint64_t req, const void *buf, size_t size);

/* Temporary solution while VFS isn't implemented (using path instead of FD) */
int32_t read(const struct sys_impl_s *sys, const char *path, void *const buf, size_t size);

/* Temporary solution while VFS isn't implemented (using path instead of FD) */
int32_t write(const struct sys_impl_s *sys, const char *path, const void *buf, size_t size);

/* Temporary solution while VFS isn't implemented (using path instead of FD) */
int32_t close(const struct sys_impl_s *sys, const char *path);

/* Driver space file IO functions */
int32_t open(const struct drv_model_cmn_s *drv, const char *dep, const char *dev, int32_t oflags, mode_t mode);
int32_t open(const struct drv_model_cmn_s *drv, const char *dev, int32_t oflags, mode_t mode);
int32_t ioctl(const struct drv_model_cmn_s *drv, int32_t fd, uint64_t req, const void *buf, size_t size);
int32_t read(const struct drv_model_cmn_s *drv, int32_t fd, void *const buf, size_t size);
int32_t write(const struct drv_model_cmn_s *drv, int32_t fd, const void *buf, size_t size);
int32_t close(const struct drv_model_cmn_s *drv, int32_t fd);

#endif /* UNISTD_HPP */
