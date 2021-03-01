#ifndef DRIVER_HPP
#define DRIVER_HPP

#include "driver_model.hpp"

/* Driver object singletone generator implementation (dependencies) */
template <typename Driver, uint32_t DepsN>
static const Driver &make_drv_impl(const struct drv_ops_s *drv_ops,
                                   const struct drv_model_cmn_s *const (&deps)[DepsN]) {

  static const std::unique_ptr<Driver> inst{new Driver(drv_ops, deps)};
  return *inst;
}

/* Driver object singletone generator implementation (no dependencies) */
template <typename Driver> static const Driver &make_drv_impl(const struct drv_ops_s *drv_ops) {
  static const std::unique_ptr<Driver> inst{new Driver(drv_ops)};
  return *inst;
}

/* Driver object singletone generator (dependencies) */
template <uint32_t DepsN, typename NameTreat>
static auto &make_drv(const struct drv_ops_s *drv_ops, const struct drv_model_cmn_s *const (&deps)[DepsN]) {
  return make_drv_impl<drv_model_impl_s<DepsN, NameTreat>, DepsN>(drv_ops, deps);
}

/* Driver object singletone generator implementation */
template <typename NameTreat> static auto &make_drv(const struct drv_ops_s *drv_ops) {
  return make_drv_impl<drv_model_impl_s<0u, NameTreat>>(drv_ops);
}

#endif /* DRIVER_HPP */
