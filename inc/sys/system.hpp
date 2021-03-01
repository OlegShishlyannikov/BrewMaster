#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "drivers/driver_model.hpp"
#include "util/tuple/tuple.hpp"
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <sys/types.h>
#include <type_traits>

// Application descritor structure
struct app_s {
  const char *name;
  void (*entry)(void *);
  uint64_t hash;
};

struct sys_impl_s {
  using this_ta = struct sys_impl_s;

  explicit sys_impl_s(const struct drv_model_cmn_s **const drvs, size_t drvs_n, const struct app_s **apps, size_t apps_n) : drvs_num_(drvs_n), apps_num_(apps_n) {
    drvs_ = reinterpret_cast<const struct drv_model_cmn_s **>(malloc(sizeof(struct drv_model_cmn_s *) * drvs_n));
    std::memcpy(drvs_, drvs, sizeof(struct drv_model_cmn_s *) * drvs_n);

    apps_ = reinterpret_cast<struct app_s **>(malloc(sizeof(struct app_s *) * apps_n));
    std::memcpy(apps_, apps, sizeof(struct app_s *) * apps_n);

    for (uint32_t i = 0u; i < apps_num_; i++) {
      apps_[i]->hash = hash_64_fnv1a(apps_[i]->name);
    }
  }

  virtual ~sys_impl_s() = default;

  /* Deleted copy & move constructors */
  sys_impl_s(const this_ta &) = delete;
  sys_impl_s(this_ta &&) = delete;
  this_ta &operator=(const this_ta &) = delete;
  this_ta &operator=(this_ta &&) = delete;

  /* Init drivers and run OS dependent function - start scheduler */
  template <typename InitFn, typename SchedStartFn> void run(const InitFn &init, const SchedStartFn &sched_start) const {
    /* Init drivers first */
    for (uint32_t i = 0u; i < drvs_num_; i++) {
      drvs_[i]->init();
    }

    init();
    sched_start();
  }

  /* Get driver by name */
  const struct drv_model_cmn_s *drv(const char *name) const {
    const uint64_t hash = hash_64_fnv1a(name);

    for (size_t i = 0u; i < drvs_num_; i++) {
      if (drvs_[i]->hash() == hash) {
        return drvs_[i];
      }
    }

    return nullptr;
  }

  // Get apps array
  struct app_s **const apps() const {
    return apps_;
  }

  // Get apps num
  size_t apps_num() const { return apps_num_; }

  /* Get app by name */
  struct app_s *app(const char *name) const {
    const uint64_t hash = hash_64_fnv1a(name);

    for (size_t i = 0u; i < apps_num_; i++) {
      if (apps_[i]->hash == hash) {
        return apps_[i];
      }
    }

    return nullptr;
  }

  /* TODO later */
  const struct file_s *vfs_get_file(const char *path) const {
    const struct file_s *file;
    const struct drv_model_cmn_s *driver;
    char *path_buffer, *name;
    size_t path_len = std::strlen(path);

    path_buffer = reinterpret_cast<char *>(malloc(path_len + 1u));
    std::strncpy(path_buffer, path, path_len);
    path_buffer[path_len] = '\0';
    name = std::strtok(path_buffer, "/");

    while (name != nullptr) {
      /* Find in drivers first */
      if ((driver = drv(name))) {
        /* Driver found */
        name = std::strtok(nullptr, "/");

        /* Find in registered devices */
        if (name) {
          file = driver->file_tree().find_file(name);

          /* File found */
          if (file) {
            goto success;
          }

          goto fail;
        }

        goto fail;
      }

      name = std::strtok(nullptr, "/");
    }

  success:
    free(path_buffer);
    return file;

  fail:
    free(path_buffer);
    return nullptr;
  }

  const struct file_s *vfs_get_file(int32_t fd) const {
    /* TODO */
    return nullptr;
  }

private:
  const struct drv_model_cmn_s **drvs_;
  const size_t drvs_num_;
  struct app_s **apps_;
  const size_t apps_num_;
};

/* System object singletone generator */
static struct sys_impl_s &make_system(const struct drv_model_cmn_s **const drvs, size_t drvs_n, const struct app_s **apps, size_t apps_n) {
  static const std::unique_ptr<struct sys_impl_s> inst{new sys_impl_s(drvs, drvs_n, apps, apps_n)};
  return *inst;
}

#endif /* SYSTEM_HPP */
