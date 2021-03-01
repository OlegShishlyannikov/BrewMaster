#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "util/hash/intmem.hpp"
#include "util/hash/sha256.hpp"

static const uint32_t SHA256_K[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

using StateType = std::array<uint32_t, 8>;
using BlockType = std::array<uint32_t, 16>;
using WType = std::array<uint32_t, 64>;

static inline uint32_t rotr(uint32_t const v, int off) { return (v >> off) | (v << (32u - off)); }
static inline void transform(StateType &S, uint8_t const *Data) {
  WType W = {0};
#pragma unroll
  for (size_t i = 0u; i < 16u; ++i) {
    W[i] = intmem::loadu_be<uint32_t>(&Data[i * sizeof(uint32_t)]);
  }

  for (size_t i = 16u; i < 64u; ++i) {
    const uint32_t s0 = rotr(W[i - 15u], 7) ^ rotr(W[i - 15u], 18u) ^ (W[i - 15u] >> 3u);
    const uint32_t s1 = rotr(W[i - 2u], 17u) ^ rotr(W[i - 2u], 19u) ^ (W[i - 2u] >> 10u);
    W[i] = (W[i - 16u] + s0 + W[i - 7u] + s1);
  }

  StateType InS = S;
  for (size_t i = 0u; i < 64u; ++i) {
    uint32_t s0 = rotr(InS[0u], 2u) ^ rotr(InS[0u], 13u) ^ rotr(InS[0u], 22u);
    uint32_t maj = (InS[0u] & InS[1u]) ^ (InS[0u] & InS[2u]) ^ (InS[1u] & InS[2u]);
    uint32_t t2 = s0 + maj;
    uint32_t s1 = rotr(InS[4u], 6u) ^ rotr(InS[4u], 11u) ^ rotr(InS[4u], 25u);
    uint32_t ch = (InS[4u] & InS[5u]) ^ ((~InS[4u]) & InS[6u]);
    uint32_t t1 = InS[7u] + s1 + ch + SHA256_K[i] + W[i];

    InS[7u] = InS[6u];
    InS[6u] = InS[5u];
    InS[5u] = InS[4u];
    InS[4u] = (InS[3u] + t1);
    InS[3u] = InS[2u];
    InS[2u] = InS[1u];
    InS[1u] = InS[0u];
    InS[0u] = (t1 + t2);
  }

  for (size_t i = 0u; i < std::tuple_size<StateType>(); ++i) {
    S[i] += InS[i];
  }
}

sha256::sha256_hash_type sha256::compute(const void *const data, const size_t size) {
  StateType State = {0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
                     0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u};
  const uint64_t BlockCount = size / sizeof(BlockType);
  for (uint64_t i = 0u; i < BlockCount; ++i) {
    transform(State, &reinterpret_cast<const uint8_t *>(data)[i * sizeof(BlockType)]);
  }

  const uint64_t Rem = size - BlockCount * sizeof(BlockType);

  uint8_t LastBlock[sizeof(BlockType)];
  std::memset(&LastBlock, 0u, sizeof(LastBlock));
  std::memcpy(&LastBlock[0u], &reinterpret_cast<const uint8_t *>(data)[BlockCount * sizeof(BlockType)], Rem);
  LastBlock[Rem] = 0x80u;

  if (Rem >= 56u) {
    ::transform(State, LastBlock);
    std::memset(&LastBlock, 0u, sizeof(LastBlock));
  }

  intmem::storeu_be(&LastBlock[56u], static_cast<uint64_t>(size << 3u));
  ::transform(State, LastBlock);

  sha256::sha256_hash_type Ret;
  static_assert(sizeof(sha256::sha256_hash_type) == sizeof(StateType), "bad definition of HashType");

  for (size_t i = 0u; i < 8u; ++i) {
    intmem::storeu_be(&Ret[i * sizeof(uint32_t)], State[i]);
  }

  return Ret;
}

sha256::sha256_hash_type sha256::sha256_from_array(const void *const data, size_t size) {
  sha256_hash_type ret;
  std::memcpy(ret.data(), reinterpret_cast<const uint8_t *>(data), ret.size());
  return std::move(ret);
}
