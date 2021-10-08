#ifndef TUPLE_UTILS_HPP
#define TUPLE_UTILS_HPP

#include <cstdint>
#include <stdexcept>
#include <tuple>
#include <variant>
#include <type_traits>

namespace std {
namespace {
template <int32_t Low, int32_t High, int32_t Mid = (Low + High) / 2, typename = void> struct visit_at_;
  template <int32_t Low, int32_t High, int32_t Mid> struct visit_at_<Low, High, Mid, std::enable_if_t<(Low > High)>> {
  template <typename... T> static decltype(auto) apply_(int32_t, T &&...) { throw std::out_of_range("visit_at"); }
};

template <int32_t Mid> struct visit_at_<Mid, Mid, Mid> {
  template <typename Tuple, typename F> static decltype(auto) apply_(int32_t n, F &&f, Tuple &&tp) {
    if (n != Mid)
      throw std::out_of_range("visit_at");
    return std::forward<F>(f)(std::get<Mid>(std::forward<Tuple>(tp)));
  }
};

template <int32_t Low, int32_t High, int32_t Mid> struct visit_at_<Low, High, Mid, std::enable_if_t<(Low < High)>> {
  template <typename... T> static decltype(auto) apply_(int32_t n, T &&... t) {
    if (n < Mid) {
      return visit_at_<Low, Mid - 1>::apply_(n, std::forward<T>(t)...);
    } else if (n == Mid) {
      return visit_at_<Mid, Mid>::apply_(n, std::forward<T>(t)...);
    } else {
      return visit_at_<Mid + 1, High>::apply_(n, std::forward<T>(t)...);
    }
  }
};

template <typename V, typename T, size_t I> auto get_getter() {
  return [](T const &t) { return V{std::in_place_index_t<I>{}, std::get<I>(t)}; };
}

template <typename Tuple, typename Indices = std::make_index_sequence<std::tuple_size<Tuple>::value>>
struct runtime_get_func_table;

template <typename Tuple, size_t... Indices> struct runtime_get_func_table<Tuple, std::index_sequence<Indices...>> {
  using return_type = typename std::tuple_element<0, Tuple>::type &;
  using get_func_ptr = return_type (*)(Tuple &);
  static constexpr get_func_ptr table[std::tuple_size<Tuple>::value] = {&std::get<Indices>...};
};

template <typename Tuple, size_t... Indices>
constexpr typename runtime_get_func_table<Tuple, std::index_sequence<Indices...>>::get_func_ptr
    runtime_get_func_table<Tuple, std::index_sequence<Indices...>>::table[std::tuple_size<Tuple>::value];

template <typename... Args, std::size_t... I> auto tuple_getters_impl(std::index_sequence<I...>) {
  using V = std::variant<Args...>;
  using T = std::tuple<Args...>;
  using F = V (*)(T const &);
  std::array<F, sizeof...(Args)> array = {{get_getter<V, T, I>()...}};
  return array;
}

template <typename... Args> auto tuple_getters(std::tuple<Args...>) {
  return tuple_getters_impl<Args...>(std::index_sequence_for<Args...>{});
}

template <std::size_t Ofst, typename Tuple, std::size_t... I>
constexpr auto slice_impl(Tuple &&t, std::index_sequence<I...>) {
  return std::forward_as_tuple(std::get<I + Ofst>(std::forward<Tuple>(t))...);
}

template <typename Tp, typename F, std::size_t... I> constexpr F for_each_impl(Tp &&t, F &&f, std::index_sequence<I...>) {
  return std::initializer_list<int>{(std::forward<F>(f)(std::get<I>(std::forward<Tp>(t))), 0)...}, f;
}
}; // namespace

template <typename Tp, typename F> constexpr F for_each(Tp &&t, F &&f) {
  return for_each_impl(std::forward<Tp>(t), std::forward<F>(f),
                       std::make_index_sequence<std::tuple_size<std::remove_cvref_t<Tp>>::value>{});
}

template <typename Tp, typename Pred> constexpr size_t find_if(Tp &&tuple, Pred pred) {
  uint32_t index = std::tuple_size<std::remove_cvref_t<Tp>>::value;
  uint32_t curr_index = 0u;
  bool found = false;

  for_each(tuple, [&](auto &&value) -> void {
    if (!found && pred(value)) {

      index = curr_index;
      found = true;
    }

    ++curr_index;
  });

  return index;
}

template <std::size_t I1, std::size_t I2, typename Cont> constexpr auto tuple_slice(Cont &&t) {
  static_assert(I2 >= I1, "Invalid slice");
  static_assert(std::tuple_size<std::decay_t<Cont>>::value >= I2, "Slice index out of bounds");
  return slice_impl<I1>(std::forward<Cont>(t), std::make_index_sequence<I2 - I1>{});
}

template <typename Tp>
constexpr typename std::tuple_element<0u, typename std::remove_reference<Tp>::type>::type &runtime_get(Tp &&t,
                                                                                                      size_t index) {
  using tuple_type = typename std::remove_reference<Tp>::type;
  if (index >= std::tuple_size<tuple_type>::value)
    throw std::runtime_error("Out of range");
  return runtime_get_func_table<tuple_type>::table[index](t);
}

template <typename Tp> struct tuple_size {
  static constexpr std::size_t size = Tp::size;
  template <typename ConvType> constexpr operator ConvType() { return ConvType(size); }
};

template <typename Tp, typename F> static decltype(auto) visit_at(int n, F &&f, Tp &&tp) {
  return visit_at_<0, int(std::tuple_size<std::decay_t<Tp>>{}) - 1>::apply_(n, std::forward<F>(f),
                                                                            std::forward<Tp>(tp));
}

template <typename ValueType, std::size_t N, typename Indices = std::make_index_sequence<N>>
static decltype(array_to_tuple_(std::array<ValueType, N>(), Indices{}))
array_to_tuple(const std::array<ValueType, N> &array) {
  return array_to_tuple_(array, Indices{});
}

template <typename ValueType, std::size_t N, typename Indices = std::make_index_sequence<N>>
static decltype(array_to_tuple_(std::array<ValueType, N>(), Indices{}))
array_to_tuple(const std::array<ValueType, N> &&array) {
  return array_to_tuple_(array, Indices{});
}

template <typename ValueType, std::size_t N, typename Indices = std::make_index_sequence<N>>
static decltype(array_to_tuple_(std::array<ValueType, N>(), Indices{})) array_to_tuple(const ValueType (&array)[N]) {
  return array_to_tuple_(array, Indices{});
}

template <typename ValueType, std::size_t N, typename Indices = std::make_index_sequence<N>>
static decltype(array_to_tuple_(std::array<ValueType, N>(), Indices{})) array_to_tuple(const ValueType(&&array)[N]) {
  return array_to_tuple_(array, Indices{});
}

template <typename... Args>
constexpr std::array<std::variant<Args...>, sizeof...(Args)> tuple_to_array(std::tuple<Args...> const &tuple) {
  return tuple_to_array_impl(tuple, std::index_sequence_for<Args...>{});
}

template <std::size_t, typename T> using T_ = T;
template <typename T, std::size_t... Is> auto make_tuple(std::index_sequence<Is...>) {
  return std::tuple<T_<Is, T>...>{};
}

template <typename T, std::size_t N> auto make_tuple() { return make_tuple<T>(std::make_index_sequence<N>{}); }
}; // namespace std

#endif /* TUPLE_UTILS_HPP */
