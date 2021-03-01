#ifndef DRIVER_MODEL_HPP
#define DRIVER_MODEL_HPP

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <type_traits>

#include "util/hash/fnv1a.hpp"
#include "util/tuple/tuple.hpp"
#include "vfs/file.hpp"

/* Forward reference */
struct drv_model_cmn_s;
template <uint32_t = 0, typename...> struct drv_model_impl_s {};

/* Init - exit functions */
struct drv_ops_s {
  void (*init)(const struct drv_model_cmn_s *);
  void (*exit)(const struct drv_model_cmn_s *);
};

/* Common driver model. Don't uses template parameters for easy polymorphism */
struct drv_model_cmn_s {
  using this_ta = drv_model_cmn_s;

  /* Get dependencies */
  virtual const this_ta *dep(const char *) const = 0;

  /* Register devices in file tree (it should be registered in file system further but no implemented yet. So we use
   * file_tree and we have access to it from global system structure) */
  struct file_s *register_chardev(const char *name, const struct file_ops_s *const ops) const {
    return files_.add_file(name, ops);
  };

  struct file_s *register_blockdev(const char *name, const struct file_ops_s *ops) const {
    return files_.add_file(name, ops);
  };

  /* Unregister devices in file tree */
  struct file_s *unregister_chardev(const char *name) const {
    return files_.remove_file(name);
  };

  struct file_s *unregister_blockdev(const char *name) const {
    return files_.remove_file(name);
  };

  /* Create list of registered devices */
  const char **create_dev_list() const { return files_.create_file_list(); }

  /* Free list */
  void free_dev_list(const char **list) const { return files_.free_file_list(list); }

  /* Init - exit methods. Should call it driver operations */
  virtual void init() const = 0;
  virtual void exit() const = 0;

  /* Get registered devices */
  const struct file_tree_s &file_tree() const { return files_; }

  /* Get driver name */
  virtual const char *name() const = 0;

  /* Get driver name hash */
  virtual const uint64_t &hash() const = 0;

private:
  /* Registered devices by this driver */
  mutable struct file_tree_s files_;
};

/* Base driver model. Describes IO functions as common for all drivers */
struct drv_model_impl_base_s : public drv_model_cmn_s {
  using this_ta = drv_model_impl_base_s;
  using base_ta = drv_model_cmn_s;

  /* With FIFO buffer methods */
  explicit drv_model_impl_base_s(const struct drv_ops_s *const drv_ops) : drv_ops_(drv_ops){};

  /* Deleted copy & move constructors */
  drv_model_impl_base_s(const this_ta &) = delete;
  drv_model_impl_base_s(this_ta &&) = delete;
  this_ta &operator=(const this_ta &) = delete;
  this_ta &operator=(this_ta &&) = delete;
  virtual ~drv_model_impl_base_s() = default;

  /* Init - exit methods. Call them from driver operations structure */
  virtual void init() const override { drv_ops_->init(this); };
  virtual void exit() const override { drv_ops_->exit(this); };

private:
  const struct drv_ops_s *const drv_ops_;
};

/* Driver dependepcy list concept */
template <typename DriverDepList> concept DriverDepListConcept = std::is_gt(std::tuple_size_v<DriverDepList>);

/* Driver model implementation */
template <uint32_t DepsN, typename DrvNameTreat>
struct drv_model_impl_s<DepsN, DrvNameTreat> : public drv_model_impl_base_s {
  using this_ta = drv_model_impl_s<DepsN, DrvNameTreat>;
  using base_ta = drv_model_impl_base_s;

  explicit drv_model_impl_s(const struct drv_ops_s *const drv_ops, const struct drv_model_cmn_s *const (&deps)[DepsN])
      : base_ta(drv_ops), name_(DrvNameTreat::name), hash_(hash_64_fnv1a(DrvNameTreat::name)) {
    std::memcpy(deps_, deps, sizeof(struct drv_model_cmn_s *) * DepsN);
  };

  /* Deleted copy & move constructors */
  drv_model_impl_s(const this_ta &) = delete;
  drv_model_impl_s(this_ta &&) = delete;
  this_ta &operator=(const this_ta &) = delete;
  this_ta &operator=(this_ta &&) = delete;
  virtual ~drv_model_impl_s() = default;

  /* Get driver name */
  virtual const char *name() const override { return name_; }

  /* Get driver name hash */
  virtual const uint64_t &hash() const override { return hash_; }

  /* Get dependency driver by name */
  virtual const struct drv_model_cmn_s *dep(const char *name) const override {
    const uint64_t hash = hash_64_fnv1a(name);

    /* Visit all tuple elements by lambda expression and find target element by name */
    for (const struct drv_model_cmn_s *drv : deps_) {
      const struct drv_model_cmn_s *res;
      if (drv->hash() == hash) {
        return drv;
      }

      continue;
    }

    return nullptr;
  }

private:
  const char *name_;
  const uint64_t hash_;
  const struct drv_model_cmn_s *deps_[DepsN];
};

/* Driver model implementation with empty dependency list */
template <typename DrvNameTreat> struct drv_model_impl_s<0u, DrvNameTreat> final : public drv_model_impl_base_s {
  using this_ta = drv_model_impl_s<0u, DrvNameTreat>;
  using base_ta = drv_model_impl_base_s;

  /* With FIFO buffer methods */
  explicit drv_model_impl_s(const struct drv_ops_s *const drv_ops)
      : base_ta(drv_ops), name_(DrvNameTreat::name), hash_(hash_64_fnv1a(DrvNameTreat::name)){};

  /* Deleted copy & move constructors */
  drv_model_impl_s(const this_ta &) = delete;
  drv_model_impl_s(this_ta &&) = delete;
  this_ta &operator=(const this_ta &) = delete;
  this_ta &operator=(this_ta &&) = delete;

  virtual ~drv_model_impl_s() = default;

  /* Get driver name */
  virtual const char *name() const override { return name_; }

  /* Get driver name hash */
  virtual const uint64_t &hash() const override { return hash_; }

  /* Get dependency driver by name */
  virtual const struct drv_model_cmn_s *dep(const char *name) const override { return nullptr; }

private:
  const char *name_;
  const uint64_t hash_;
};

#endif /* DRIVER_MODEL_HPP */
