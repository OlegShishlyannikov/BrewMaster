#ifndef SHA256_LITERAL_H
#define SHA256_LITERAL_H

#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace sha256_literal {

static constexpr uint32_t SHA256_K[64u] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

using StateType = std::array<uint32_t, 8u>;
using BlockType = std::array<uint32_t, 16u>;
using WType = std::array<uint32_t, 64u>;
using HashType = std::array<uint8_t, sizeof(StateType)>;

namespace details {

template <typename F, typename T, size_t N, typename... Args>
static constexpr auto map_zip(F const f, std::array<T, N> const data, Args const... Arrays) {
  std::array<decltype(f(data[0u], Arrays[0u]...)), N> out = {0};
  auto *itOut = &std::get<0u>(out);
  for (size_t i = 0u; i < N; ++i)
    itOut[i] = f(data[i], Arrays[i]...);
  return out;
}

template <typename F, typename T, size_t N, typename... Args>
static constexpr auto map(F const f, std::array<T, N> const data, Args const... args) {
  using RetType = decltype(f(std::declval<T>(), args...));
  std::array<RetType, N> out = {0u};
  auto *itOut = &std::get<0>(out);
  for (size_t i = 0u; i < N; ++i)
    itOut[i] = f(data[i], args...);
  return out;
}

template <typename F, typename T, typename... Args>
static constexpr auto map(F const f, std::array<T, 0> const, Args const... args) {
  using RetType = decltype(f(std::declval<T>(), args...));
  return std::array<RetType, 0u>{};
}

// BlockType constexpr helpers

static constexpr uint32_t xor_(uint32_t a, uint32_t b) { return a ^ b; }
static constexpr BlockType blocktype_xor(BlockType const A, uint8_t const B) {
  const uint32_t B32 = (uint32_t)B;
  const uint32_t B32x4 = B32 | (B32 << 8) | (B32 << 16) | (B32 << 24);
  return map(xor_, A, B32x4);
}

static constexpr uint32_t u8x4_to_be_u32(uint8_t const a, uint8_t const b, uint8_t const c, uint8_t const d) {
  return ((uint32_t)d) | (((uint32_t)c) << 8u) | (((uint32_t)b) << 16u) | (((uint32_t)a) << 24u);
}

// SHA256 routines
// Based on code from https://github.com/thomdixon/pysha2/blob/master/sha2/sha256.py

static constexpr uint32_t rotr(uint32_t const v, int off) { return (v >> off) | (v << (32u - off)); }
static constexpr uint32_t sum(uint32_t const a, uint32_t const b) { return a + b; }
static constexpr StateType transform(StateType const S, BlockType const data) {
  WType W = {0u};
  auto *ItW = &std::get<0u>(W);
  auto const *CItW = &std::get<0u>(W);
  for (size_t i = 0u; i < data.size(); ++i) {
    ItW[i] = data[i];
  }

  for (size_t i = 16u; i < 64u; ++i) {
    const uint32_t s0 = rotr(CItW[i - 15u], 7u) ^ rotr(CItW[i - 15u], 18u) ^ (CItW[i - 15u] >> 3u);
    const uint32_t s1 = rotr(CItW[i - 2u], 17u) ^ rotr(CItW[i - 2u], 19u) ^ (CItW[i - 2u] >> 10u);
    ItW[i] = (CItW[i - 16u] + s0 + CItW[i - 7u] + s1);
  }

  StateType InS = S;
  auto const *CInS = &std::get<0u>(InS);
  for (size_t i = 0u; i < 64u; ++i) {
    const uint32_t s0 = rotr(CInS[0u], 2u) ^ rotr(CInS[0u], 13u) ^ rotr(CInS[0u], 22u);
    const uint32_t maj = (CInS[0u] & CInS[1u]) ^ (CInS[0u] & CInS[2u]) ^ (CInS[1u] & CInS[2u]);
    const uint32_t t2 = s0 + maj;
    const uint32_t s1 = rotr(CInS[4u], 6u) ^ rotr(CInS[4u], 11u) ^ rotr(CInS[4u], 25u);
    const uint32_t ch = (CInS[4u] & CInS[5u]) ^ ((~CInS[4u]) & CInS[6u]);
    const uint32_t t1 = CInS[7u] + s1 + ch + SHA256_K[i] + CItW[i];

    InS = {t1 + t2, CInS[0u], CInS[1u], CInS[2u], CInS[3u] + t1, CInS[4u], CInS[5u], CInS[6u]};
  }

  return map_zip(sum, S, InS);
}

static auto constexpr u8_to_block(uint8_t const *It) {
  BlockType B = {0u};
  auto *ItB = &std::get<0u>(B);
  for (size_t i = 0u; i < std::tuple_size<BlockType>(); i++) {
    ItB[i] = u8x4_to_be_u32(It[i * sizeof(uint32_t)], It[i * sizeof(uint32_t) + 1], It[i * sizeof(uint32_t) + 2],
                            It[i * sizeof(uint32_t) + 3]);
  }
  return B;
}

template <uint64_t BlockCount, typename Ar>
constexpr std::enable_if_t<BlockCount != 0, std::array<BlockType, BlockCount>> u8_to_blocks_(Ar const Data) {
  std::array<BlockType, BlockCount> Ret = {{0u}};
  auto *ItRet = &std::get<0u>(Ret);
  for (uint64_t i = 0u; i < BlockCount; ++i) {
    ItRet[i] = u8_to_block(&Data[i * sizeof(BlockType)]);
  }
  return Ret;
}

template <uint64_t BlockCount, typename Ar>
constexpr std::enable_if_t<BlockCount == 0u, std::array<BlockType, 0u>> u8_to_blocks_(Ar const __attribute__((unused))
                                                                                      Data) {
  return std::array<BlockType, 0u>{};
}

template <size_t Len_> constexpr auto u8_to_blocks(std::array<uint8_t, Len_> const Data) {
  constexpr uint64_t Len = Len_;
  constexpr uint64_t BlockCount = Len / sizeof(BlockType);
  return u8_to_blocks_<BlockCount>(Data);
}

static constexpr HashType state_to_hash(StateType const S) {
  HashType Ret = {0u};
  auto *ItRet = &std::get<0u>(Ret);
  for (size_t i = 0u; i < std::tuple_size<StateType>(); ++i) {
    uint32_t const V = S[i];
    ItRet[i * sizeof(uint32_t)] = (V >> 24u) & 0xffu;
    ItRet[i * sizeof(uint32_t) + 1u] = (V >> 16u) & 0xffu;
    ItRet[i * sizeof(uint32_t) + 2u] = (V >> 8u) & 0xffu;
    ItRet[i * sizeof(uint32_t) + 3u] = (V)&0xffu;
  }
  return Ret;
}

template <typename InputIt, typename OutputIt>
static constexpr OutputIt constexpr_copy(InputIt first, InputIt last, OutputIt d_first) {
  for (; first != last; ++first) {
    *d_first = *first;
    ++d_first;
  }
  return d_first;
}

template <typename T, size_t N> constexpr auto *get_array_it(std::array<T, N> const &Data) {
  return &std::get<0>(Data);
}
template <typename T> constexpr T *get_array_it(std::array<T, 0> const &) {
  // Do not use nullptr here, because it is of type "nullptr_t", and this will
  // give erros for the pointer arithmetics done in sha256.
  // This would be much easier with "if constexpr" in C++17!
  return 0u;
}

template <size_t Len_> static constexpr HashType sha256(std::array<uint8_t, Len_> const Data) {
  constexpr StateType StateOrg = {0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
                                  0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u};

  StateType State = StateOrg;
  constexpr uint64_t Len = Len_;
  constexpr uint64_t BlockCount = Len / sizeof(BlockType);

  auto const Blocks = u8_to_blocks(Data);

  for (size_t i = 0u; i < Blocks.size(); ++i) {
    State = transform(State, Blocks[i]);
  }

  struct {
    uint8_t Data[64u];
  } LastB = {0u};

  auto *const ItLastBBegin = &LastB.Data[0u];
  auto *ItLastB = ItLastBBegin;

  auto *ItData = get_array_it(Data);
  if (ItData != 0u) {
    ItLastB = constexpr_copy(ItData + BlockCount * sizeof(BlockType), ItData + Len, ItLastB);
  }
  *ItLastB = 0x80u;

  constexpr uint64_t Rem = Len - BlockCount * sizeof(BlockType);
  if (Rem >= 56u) {
    BlockType const LastB_ = u8_to_block(ItLastBBegin);
    State = transform(State, LastB_);
    LastB = {0u};
  }
  constexpr uint64_t Len3 = Len << 3u;
  for (size_t i = 0u; i < sizeof(uint64_t); ++i) {
    LastB.Data[56u + i] = (Len3 >> (56u - (i * 8u))) & 0xffu;
  }
  BlockType const LastB_ = u8_to_block(ItLastBBegin);
  State = transform(State, LastB_);

  return state_to_hash(State);
}

static constexpr uint8_t char_to_u8(char const v) { return v; }
template <size_t N, typename T, size_t N_, size_t... I>
static constexpr auto get_array(T const (&Data)[N_], std::index_sequence<I...>) {
  return std::array<T, N>{Data[I]...};
}

template <size_t N, typename T, size_t N_> static constexpr auto get_array(T const (&Data)[N_]) {
  return get_array<N>(Data, std::make_index_sequence<N>());
}

template <typename T, size_t N> static constexpr auto get_array(T const (&Data)[N]) { return get_array<N>(Data); }

} // namespace details

template <size_t N> static constexpr auto compute(std::array<uint8_t, N> const Data) { return details::sha256(Data); }

template <size_t N> static constexpr auto compute(char const (&Data)[N]) {
  auto const Ar = details::get_array(Data);
  return details::sha256(details::map(details::char_to_u8, Ar));
}

template <size_t N> static constexpr auto compute_str(char const (&Data)[N]) {
  auto const Ar = details::get_array<N - 1u>(Data);
  return details::sha256(details::map(details::char_to_u8, Ar));
}

} // namespace sha256_literal

template <typename CharT, CharT... Cs> static constexpr auto operator"" _sha256() {
  static_assert(std::is_same<CharT, char>::value, "only support 8-bit strings");
  const char Data[] = {Cs...};
  return sha256_literal::compute(Data);
}

#endif
