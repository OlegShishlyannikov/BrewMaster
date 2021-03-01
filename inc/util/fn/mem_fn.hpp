#ifndef MEM_FN_HPP
#define MEM_FN_HPP

#include "fn.hpp"
#include "fn_tr_help.hpp"

namespace std {
namespace fn_tr {
namespace detail {

struct const_tag_s {};
struct volatile_tag_s {};
struct lref_tag_s {};
struct rref_tag_s {};

template <typename T, typename Fn, typename... Q> struct mem_fn_tr_q_s_ : static_fn_tr_s<Fn> {
  using class_t = T;
  static constexpr bool is_const_v = types_has_s<const_tag_s, Q...>::value;
  static constexpr bool is_volatile_v = types_has_s<volatile_tag_s, Q...>::value;
  static constexpr bool is_lref_v = types_has_s<lref_tag_s, Q...>::value;
  static constexpr bool is_rref_v = types_has_s<rref_tag_s, Q...>::value;
};

template <typename T, typename Fn, typename... Q> const bool mem_fn_tr_q_s_<T, Fn, Q...>::is_const_v;
template <typename T, typename Fn, typename... Q> const bool mem_fn_tr_q_s_<T, Fn, Q...>::is_volatile_v;
template <typename T, typename Fn, typename... Q> const bool mem_fn_tr_q_s_<T, Fn, Q...>::is_lref_v;
template <typename T, typename Fn, typename... Q> const bool mem_fn_tr_q_s_<T, Fn, Q...>::is_rref_v;

template <typename MemFn> struct mem_fn_traits_s_;
template <typename T, typename Ret, typename... Args>
struct mem_fn_traits_s_<Ret (T::*)(Args...)> : mem_fn_tr_q_s_<T, Ret(Args...)> {};

template <typename T, typename Ret, typename... Args>
struct mem_fn_traits_s_<Ret (T::*)(Args...) const> : mem_fn_tr_q_s_<T, Ret(Args...), const_tag_s> {};

template <typename T, typename Ret, typename... Args>
struct mem_fn_traits_s_<Ret (T::*)(Args...) volatile> : mem_fn_tr_q_s_<T, Ret(Args...), volatile_tag_s> {};

template <typename T, typename Ret, typename... Args>
struct mem_fn_traits_s_<Ret (T::*)(Args...) const volatile>
    : mem_fn_tr_q_s_<T, Ret(Args...), const_tag_s, volatile_tag_s> {};

template <typename T, typename Ret, typename... Args>
struct mem_fn_traits_s_<Ret (T::*)(Args...) &> : mem_fn_tr_q_s_<T, Ret(Args...), lref_tag_s> {};

template <typename T, typename Ret, typename... Args>
struct mem_fn_traits_s_<Ret (T::*)(Args...) const &> : mem_fn_tr_q_s_<T, Ret(Args...), const_tag_s, lref_tag_s> {};

template <typename T, typename Ret, typename... Args>
struct mem_fn_traits_s_<Ret (T::*)(Args...) volatile &> : mem_fn_tr_q_s_<T, Ret(Args...), volatile_tag_s, lref_tag_s> {
};

template <typename T, typename Ret, typename... Args>
struct mem_fn_traits_s_<Ret (T::*)(Args...) const volatile &>
    : mem_fn_tr_q_s_<T, Ret(Args...), const_tag_s, volatile_tag_s, lref_tag_s> {};

template <typename T, typename Ret, typename... Args>
struct mem_fn_traits_s_<Ret (T::*)(Args...) &&> : mem_fn_tr_q_s_<T, Ret(Args...), rref_tag_s> {};

template <typename T, typename Ret, typename... Args>
struct mem_fn_traits_s_<Ret (T::*)(Args...) const &&> : mem_fn_tr_q_s_<T, Ret(Args...), const_tag_s, rref_tag_s> {};

template <typename T, typename Ret, typename... Args>
struct mem_fn_traits_s_<Ret (T::*)(Args...) volatile &&> : mem_fn_tr_q_s_<T, Ret(Args...), volatile_tag_s, rref_tag_s> {
};

template <typename T, typename Ret, typename... Args>
struct mem_fn_traits_s_<Ret (T::*)(Args...) const volatile &&>
    : mem_fn_tr_q_s_<T, Ret(Args...), const_tag_s, volatile_tag_s, rref_tag_s> {};
}; // namespace detail

template <typename MemFnPtr> struct mem_fn_tr_s : detail::mem_fn_traits_s_<std::remove_cvref_t<MemFnPtr>> {};
}; // namespace fn_tr
}; // namespace std

#endif /* MEM_FN_HPP */
