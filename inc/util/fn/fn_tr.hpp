#ifndef FN_TR_HPP
#define FN_TR_HPP

#include <functional>

#include "fn.hpp"
#include "fn_obj.hpp"
#include "fn_tr_help.hpp"
#include "mem_fn.hpp"

namespace std {
namespace fn_tr {
namespace detail {
struct fn_tag {};
struct fn_ptr_tag {};
struct mem_fn_tag {};
struct fn_obj_tag {};

template <typename Fn> struct fn_tr_s_ : fn_obj_tr_s<Fn> { using fn_cat = fn_obj_tag; };

template <typename Ret, typename... Args> struct fn_tr_s_<Ret(Args...)> : static_fn_tr_s<Ret(Args...)> {
  using fn_cat_ta = fn_tag;
};

template <typename Ret, typename... Args> struct fn_tr_s_<Ret (*)(Args...)> : static_fn_tr_s<Ret(Args...)> {
  using fn_cat = fn_ptr_tag;
};

template <typename Class, typename Ret, typename... Args>
struct fn_tr_s_<Ret (Class::*)(Args...)> : mem_fn_tr_s<Ret (Class::*)(Args...)> {
  using fn_cat_ta = mem_fn_tag;
};
} // namespace detail

  template <typename Fn> struct fn_tr_s : detail::fn_tr_s_<typename std::remove_cvref<Fn>::type> {};

template <typename Fn> constexpr const std::function<const typename fn_tr_s<Fn>::fn_t> to_std_function(const Fn &f) {
  return std::function<typename fn_tr_s<Fn>::fn_t>(f);
}

template <typename Fn>
constexpr const std::function<typename fn_tr_s<Fn>::fn_t> to_std_function(typename fn_tr_s<Fn>::class_t *p_class,
                                                                          const Fn &function) {
  using ret_ta = typename fn_tr_s<Fn>::ret_t;
  using fn_ta = typename fn_tr_s<Fn>::fn_t;
  return std::function<fn_ta>([p_class, function](auto &&... args) -> ret_ta { return (p_class->*function)(args...); });
}

}; // namespace fn_tr
}; // namespace std

#endif /* FN_TR_HPP */
