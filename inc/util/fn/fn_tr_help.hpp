#ifndef FN_TR_HELP_HPP
#define FN_TR_HELP_HPP

#include <cstdint>
#include <utility>

namespace std {
namespace fn_tr {
template <int N, typename... Ts> struct get_s;
template <int N, typename T, typename... Ts> struct get_s<N, std::tuple<T, Ts...>> {
  using type = typename get_s<N - 1u, std::tuple<Ts...>>::type;
};

template <typename T, typename... Ts> struct get_s<0u, std::tuple<T, Ts...>> { using type = T; };
template <typename... Types> struct types_cnt_s;
template <> struct types_cnt_s<> { static constexpr std::uint64_t value = 0; };

template <typename Type, typename... Types> struct types_cnt_s<Type, Types...> {
  static constexpr std::uint64_t value = types_cnt_s<Types...>::value + 1;
};

template <std::uint64_t n, typename... Types> struct types_n_s;
template <std::uint64_t N, typename Type, typename... Types>
struct types_n_s<N, Type, Types...> : types_n_s<N - 1, Types...> {};
template <typename Type, typename... Types> struct types_n_s<0, Type, Types...> { typedef Type type; };

template <typename Q, typename... Ts> struct types_has_s {};
template <typename Q> struct types_has_s<Q> { static constexpr bool value = false; };

template <typename Q, typename... Ts> struct types_has_s<Q, Q, Ts...> { static constexpr bool value = true; };

template <typename Q, typename T, typename... Ts> struct types_has_s<Q, T, Ts...> : types_has_s<Q, Ts...> {};
}; // namespace fn_tr
}; // namespace std

#endif /* FN_TR_HELP_HPP */
