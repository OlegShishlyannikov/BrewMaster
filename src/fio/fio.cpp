#include "fio/unistd.hpp"

int32_t open(const struct drv_model_cmn_s *drv, const char *dep, const char *dev_name, int32_t oflags, mode_t mode) {
  const struct drv_model_cmn_s *dep_drv;
  const struct file_s *dev_file;
  int32_t rc;

  /* Get dependency module */
  if (!(dep_drv = drv->dep(dep))) {
    // errno = ENOENT
    goto error;
  }

  /* Find device */
  if (!(dev_file = dep_drv->file_tree().find_file(dev_name))) {
    // errno = ENOENT
    goto error;
  }

  /* Lock file */
  if ((rc = dev_file->ops->flock()) < 0) {
    goto error;
  }

  if ((rc = dev_file->ops->open(oflags, mode)) < 0) {
    if ((rc = dev_file->ops->funlock()) < 0) {
      goto error;
    }

    // Opening error
    goto error;
  }

  // File opened, save flags and return fd
  dev_file->oflags = oflags;
  dev_file->mode = mode;

  return dev_file->fd;
error:
  return -1;
}

int32_t open(const struct drv_model_cmn_s *drv, const char *dev_name, int32_t oflags, mode_t mode) {
  const struct file_s *dev_file;
  int32_t rc;

  /* Find device */
  if (!(dev_file = drv->file_tree().find_file(dev_name))) {
    // errno = ENOENT
    goto error;
  }

  /* Lock file */
  if ((rc = dev_file->ops->flock()) < 0) {
    goto error;
  }

  if ((rc = dev_file->ops->open(oflags, mode)) < 0) {
    if ((rc = dev_file->ops->funlock()) < 0) {
      goto error;
    }

    // Opening error
    goto error;
  }

  // File opened, save flags and return fd
  dev_file->oflags = oflags;
  dev_file->mode = mode;

  return dev_file->fd;
error:
  return -1;
}

int32_t ioctl(const struct drv_model_cmn_s *drv, int32_t fd, uint64_t req, const void *buf, size_t size) {
  int32_t rc;
  const struct file_s *file;

  // Search file in registered devices of module
  if (!(file = drv->file_tree().find_file(fd))) {
    goto error;
  }

  // If it isn't opened
  if (!file->oflags && !file->mode) {
    // errno = EBADFD
    goto error;
  }

  if ((rc = file->ops->ioctl(req, buf, size)) < 0) {
    if ((rc = file->ops->funlock()) < 0) {
      // errno = ???
      goto error;
    }

    goto error;
  }

  return rc;
error:
  return -1;
}

int32_t read(const struct drv_model_cmn_s *drv, int32_t fd, void *const buf, size_t size) {
  int32_t rc;
  const struct file_s *file;

  // Search file in registered devices of module
  if (!(file = drv->file_tree().find_file(fd))) {
    goto error;
  }

  // If it isn't opened
  if (!file->oflags && !file->mode) {
    // errno = EBADFD
    goto error;
  }

  if ((rc = file->ops->read(buf, size)) < 0) {
    if ((rc = file->ops->funlock()) < 0) {
      // errno = ???
      goto error;
    }

    goto error;
  }

  return rc;
error:
  return -1;
}

int32_t write(const struct drv_model_cmn_s *drv, int32_t fd, const void *buf, size_t size) {
  int32_t rc;
  const struct file_s *file;

  // Search file in registered devices of module
  if (!(file = drv->file_tree().find_file(fd))) {
    goto error;
  }

  // If it isn't opened
  if (!file->oflags && !file->mode) {
    // errno = EBADFD
    goto error;
  }

  if ((rc = file->ops->write(buf, size)) < 0) {
    if ((rc = file->ops->funlock()) < 0) {
      // errno = ???
      goto error;
    }

    goto error;
  }

  return rc;
error:
  return -1;
}

int32_t close(const struct drv_model_cmn_s *drv, int32_t fd) {
  int32_t rc;
  const struct file_s *file;

  // Search file in registered devices of module
  if (!(file = drv->file_tree().find_file(fd))) {
    goto error;
  }

  // If it isn't opened
  if (!file->oflags && !file->mode) {
    // errno = EBADFD
    goto error;
  }

  if ((rc = file->ops->close()) < 0) {
    if ((rc = file->ops->funlock()) < 0) {
      // errno = ???
      goto error;
    }

    // errno = ???
    goto error;
  }

  // Release lock
  if ((rc = file->ops->funlock()) < 0) {
    // errno = ???
    goto error;
  }

  return rc;
error:
  return -1;
}

/* User space file IO functions */
int32_t open(const struct sys_impl_s *sys, const char *path, int32_t oflags, mode_t mode) {
  int32_t rc;
  const struct file_s *file;

  if (!(file = sys->vfs_get_file(path))) {
    // errno = ENOENT
    goto error;
  }

  if ((rc = file->ops->flock()) < 0) {
    // errno = ???
    goto error;
  }

  if ((rc = file->ops->open(oflags, mode)) < 0) {
    // errno = ???
    if ((rc = file->ops->funlock()) < 0) {
      // errno = ???
      goto error;
    }

    goto error;
  }

  file->oflags = oflags;
  file->mode = mode;

  return file->fd;
error:
  return -1;
}

/* Temporary solution while VFS isn't implemented (using path instead of FD) */
int32_t ioctl(const struct sys_impl_s *sys, const char *path, uint64_t req, const void *buf, size_t size) {
  int32_t rc;
  const struct file_s *file;

  // Search file in registered devices of module
  if (!(file = sys->vfs_get_file(path))) {
    // errno = ENOENT
    goto error;
  }

  // If it isn't opened
  if (!file->oflags && !file->mode) {
    // errno = EBADFD
    goto error;
  }

  if ((rc = file->ops->ioctl(req, buf, size)) < 0) {
    goto error;
  }

  return rc;
error:
  return -1;
}

/* Temporary solution while VFS isn't implemented (using path instead of FD) */
int32_t read(const struct sys_impl_s *sys, const char *path, void *const buf, size_t size) {
  int32_t rc;
  const struct file_s *file;

  // Search file in registered devices of module
  if (!(file = sys->vfs_get_file(path))) {
    // errno = ENOENT
    goto error;
  }

  // If it isn't opened
  if (!file->oflags && !file->mode) {
    // errno = EBADFD
    goto error;
  }

  if ((rc = file->ops->read(buf, size)) < 0) {
    goto error;
  }

  return rc;
error:
  return -1;
}

/* Temporary solution while VFS isn't implemented (using path instead of FD) */
int32_t write(const struct sys_impl_s *sys, const char *path, const void *buf, size_t size) {
  int32_t rc;
  const struct file_s *file;

  // Search file in registered devices of module
  if (!(file = sys->vfs_get_file(path))) {
    // errno = ENOENT
    goto error;
  }

  // If it isn't opened
  if (!file->oflags && !file->mode) {
    // errno = EBADFD
    goto error;
  }

  if ((rc = file->ops->write(buf, size)) < 0) {
    goto error;
  }

  return rc;
error:
  return -1;
}

/* Temporary solution while VFS isn't implemented (using path instead of FD) */
int32_t close(const struct sys_impl_s *sys, const char *path) {
  int32_t rc;
  const struct file_s *file;

  // Search file in registered devices of module
  if (!(file = sys->vfs_get_file(path))) {
    // errno = ENOENT
    goto error;
  }

  // If it isn't opened
  if (!file->oflags && !file->mode) {
    // errno = EBADFD
    goto error;
  }

  if ((rc = file->ops->close()) < 0) {
    if ((rc = file->ops->funlock()) < 0) {
      // errno = ???
      goto error;
    }

    // errno = ???
    goto error;
  }

  // Release lock
  if ((rc = file->ops->funlock()) < 0) {
    // errno = ???
    goto error;
  }

  return rc;
error:
  return -1;
}
