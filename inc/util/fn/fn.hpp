#ifndef FN_HPP
#define FN_HPP

#include "fn_tr_help.hpp"
#include <cstddef>
#include <tuple>

namespace std {
namespace fn_tr {
namespace detail {

template <typename Fn> struct static_fn_tr_s {};
template <typename Ret, typename... Args> struct static_fn_tr_s<Ret(Args...)> {
  using ret_ta = Ret;
  using fn_ta = Ret(Args...);
  static constexpr uint64_t argc = types_cnt_s<Args...>::value;
  template <uint64_t N> using arg_ta = typename types_n_s<N, Args...>::type;
  using args_ta = std::tuple<Args...>;
  static constexpr bool is_void_v = std::is_void_v<Ret>;
};

template <typename Ret, typename... Args> const std::uint64_t static_fn_tr_s<Ret(Args...)>::argc;
} // namespace detail

template <typename Fn> struct static_fn_tr_s : detail::static_fn_tr_s<std::remove_cvref_t<Fn>> {};
template <typename Fn> struct static_fn_tr_s<Fn *> : detail::static_fn_tr_s<std::remove_cvref_t<Fn>> {};

}; // namespace fn_tr
}; // namespace std

#endif /* FN_HPP */
