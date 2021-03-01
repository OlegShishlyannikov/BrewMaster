#ifndef SHA256_HPP
#define SHA256_HPP

#include <array>
#include <cstdint>
#include <cstring>

namespace sha256 {
template <typename T, uint32_t N> struct sha256_hash_t : std::array<T, N> {
  using this_t = sha256_hash_t<T, N>;
  using base_s = std::array<T, N>;

  constexpr this_t operator^(const base_s &data) { return xor_(static_cast<const base_s>(*this), data); }
  constexpr this_t operator&(const base_s &data) { return and_(static_cast<const base_s>(*this), data); }
  constexpr this_t operator|(const base_s &data) { return or_(static_cast<const base_s>(*this), data); }

  constexpr this_t operator^=(const base_s &data) { return xor_(static_cast<const base_s>(*this), data); }
  constexpr this_t operator&=(const base_s &data) { return and_(static_cast<const base_s>(*this), data); }
  constexpr this_t operator|=(const base_s &data) { return or_(static_cast<const base_s>(*this), data); }

  constexpr this_t operator^(const T &data) { return xor_(static_cast<const base_s>(*this), data); }
  constexpr this_t operator&(const T &data) { return and_(static_cast<const base_s>(*this), data); }
  constexpr this_t operator|(const T &data) { return or_(static_cast<const base_s>(*this), data); }

  constexpr this_t operator^=(const T &data) { return xor_(static_cast<const base_s>(*this), data); }
  constexpr this_t operator&=(const T &data) { return and_(static_cast<const base_s>(*this), data); }
  constexpr this_t operator|=(const T &data) { return or_(static_cast<const base_s>(*this), data); }

  constexpr this_t operator~() { return not_(*this); }

private:
  template <int32_t... Is> struct seq_ {};
  template <int32_t I, int32_t... Is> struct gen_seq_ : gen_seq_<I - 1u, I - 1u, Is...> {};
  template <int32_t... Is> struct gen_seq_<0u, Is...> : seq_<Is...> {};

  template <int32_t... Is> constexpr this_t xor_impl_(const base_s &lhs, const base_s &rhs, seq_<Is...>) {
    return {{static_cast<const uint8_t>(std::get<Is>(lhs) ^ std::get<Is>(rhs))...}};
  }

  template <int32_t... Is> constexpr this_t or_impl_(const base_s &lhs, const base_s &rhs, seq_<Is...>) {
    return {{static_cast<uint8_t>(std::get<Is>(lhs) | std::get<Is>(rhs))...}};
  }

  template <int32_t... Is> constexpr this_t and_impl_(const base_s &lhs, const base_s &rhs, seq_<Is...>) {
    return {{static_cast<uint8_t>(std::get<Is>(lhs) & std::get<Is>(rhs))...}};
  }

  template <int32_t... Is> constexpr this_t not_impl_(const base_s &data, seq_<Is...>) {
    return {{static_cast<uint8_t>(~std::get<Is>(data))...}};
  }

  template <int32_t... Is> constexpr this_t xor_impl_(const base_s &lhs, const T &rhs, seq_<Is...>) {
    return {{static_cast<const uint8_t>(std::get<Is>(lhs) ^ rhs)...}};
  }

  template <int32_t... Is> constexpr this_t or_impl_(const base_s &lhs, const T &rhs, seq_<Is...>) {
    return {{static_cast<uint8_t>(std::get<Is>(lhs) | rhs)...}};
  }

  template <int32_t... Is> constexpr this_t and_impl_(const base_s &lhs, const T &rhs, seq_<Is...>) {
    return {{static_cast<uint8_t>(std::get<Is>(lhs) & rhs)...}};
  }

  constexpr auto xor_(const base_s &lhs, const base_s &rhs) -> decltype(xor_impl_(lhs, rhs, gen_seq_<N>{})) {
    return xor_impl_(lhs, rhs, gen_seq_<N>{});
  }

  constexpr auto or_(const base_s &lhs, const base_s &rhs) -> decltype(or_impl_(lhs, rhs, gen_seq_<N>{})) {
    return or_impl_(lhs, rhs, gen_seq_<N>{});
  }

  constexpr auto and_(const base_s &lhs, const base_s &rhs) -> decltype(and_impl_(lhs, rhs, gen_seq_<N>{})) {
    return and_impl_(lhs, rhs, gen_seq_<N>{});
  }

  constexpr auto not_(const base_s &data) -> decltype(not_impl_(data, gen_seq_<N>{})) {
    return not_impl_(data, gen_seq_<N>{});
  }

  constexpr auto xor_(const base_s &lhs, const T &rhs) -> decltype(xor_impl_(lhs, rhs, gen_seq_<N>{})) {
    return xor_impl_(lhs, rhs, gen_seq_<N>{});
  }

  constexpr auto or_(const base_s &lhs, const T &rhs) -> decltype(or_impl_(lhs, rhs, gen_seq_<N>{})) {
    return or_impl_(lhs, rhs, gen_seq_<N>{});
  }

  constexpr auto and_(const base_s &lhs, const T &rhs) -> decltype(and_impl_(lhs, rhs, gen_seq_<N>{})) {
    return and_impl_(lhs, rhs, gen_seq_<N>{});
  }
};

using sha256_hash_type = sha256_hash_t<uint8_t, 32u>;
sha256_hash_type compute(const void *const data, const size_t size);
sha256_hash_type sha256_from_array(const void *const data, size_t size);

} // namespace sha256

#endif /* SHA256_HPP */
