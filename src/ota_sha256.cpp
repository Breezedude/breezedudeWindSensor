#include "ota_sha256.h"

namespace {

constexpr uint32_t kRoundConstants[64] = {
  0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
  0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
  0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
  0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
  0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
  0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
  0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
  0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u, 0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u,
};

inline uint32_t rotr32(uint32_t value, uint8_t bits) {
  return (value >> bits) | (value << (32u - bits));
}

inline uint32_t choose(uint32_t e, uint32_t f, uint32_t g) {
  return (e & f) ^ (~e & g);
}

inline uint32_t majority(uint32_t a, uint32_t b, uint32_t c) {
  return (a & b) ^ (a & c) ^ (b & c);
}

inline uint32_t sigma0(uint32_t x) {
  return rotr32(x, 2) ^ rotr32(x, 13) ^ rotr32(x, 22);
}

inline uint32_t sigma1(uint32_t x) {
  return rotr32(x, 6) ^ rotr32(x, 11) ^ rotr32(x, 25);
}

inline uint32_t gamma0(uint32_t x) {
  return rotr32(x, 7) ^ rotr32(x, 18) ^ (x >> 3);
}

inline uint32_t gamma1(uint32_t x) {
  return rotr32(x, 17) ^ rotr32(x, 19) ^ (x >> 10);
}

void ota_sha256_transform(ota_sha256_ctx_t *ctx, const uint8_t block[64]) {
  uint32_t w[64] = {0};
  for(uint8_t i = 0; i < 16; ++i) {
    const uint8_t j = i * 4u;
    w[i] = ((uint32_t)block[j] << 24) |
           ((uint32_t)block[j + 1] << 16) |
           ((uint32_t)block[j + 2] << 8) |
           (uint32_t)block[j + 3];
  }
  for(uint8_t i = 16; i < 64; ++i) {
    w[i] = gamma1(w[i - 2]) + w[i - 7] + gamma0(w[i - 15]) + w[i - 16];
  }

  uint32_t a = ctx->state[0];
  uint32_t b = ctx->state[1];
  uint32_t c = ctx->state[2];
  uint32_t d = ctx->state[3];
  uint32_t e = ctx->state[4];
  uint32_t f = ctx->state[5];
  uint32_t g = ctx->state[6];
  uint32_t h = ctx->state[7];

  for(uint8_t i = 0; i < 64; ++i) {
    const uint32_t t1 = h + sigma1(e) + choose(e, f, g) + kRoundConstants[i] + w[i];
    const uint32_t t2 = sigma0(a) + majority(a, b, c);
    h = g;
    g = f;
    f = e;
    e = d + t1;
    d = c;
    c = b;
    b = a;
    a = t1 + t2;
  }

  ctx->state[0] += a;
  ctx->state[1] += b;
  ctx->state[2] += c;
  ctx->state[3] += d;
  ctx->state[4] += e;
  ctx->state[5] += f;
  ctx->state[6] += g;
  ctx->state[7] += h;
}

}  // namespace

void ota_sha256_init(ota_sha256_ctx_t *ctx) {
  ctx->datalen = 0;
  ctx->bitlen = 0;
  ctx->state[0] = 0x6a09e667u;
  ctx->state[1] = 0xbb67ae85u;
  ctx->state[2] = 0x3c6ef372u;
  ctx->state[3] = 0xa54ff53au;
  ctx->state[4] = 0x510e527fu;
  ctx->state[5] = 0x9b05688cu;
  ctx->state[6] = 0x1f83d9abu;
  ctx->state[7] = 0x5be0cd19u;
}

void ota_sha256_update(ota_sha256_ctx_t *ctx, const uint8_t *data, size_t len) {
  for(size_t i = 0; i < len; ++i) {
    ctx->data[ctx->datalen++] = data[i];
    if(ctx->datalen == sizeof(ctx->data)) {
      ota_sha256_transform(ctx, ctx->data);
      ctx->bitlen += 512u;
      ctx->datalen = 0;
    }
  }
}

void ota_sha256_final(ota_sha256_ctx_t *ctx, uint8_t hash[32]) {
  uint32_t i = ctx->datalen;

  ctx->data[i++] = 0x80u;
  if(i > 56u) {
    while(i < 64u) {
      ctx->data[i++] = 0x00u;
    }
    ota_sha256_transform(ctx, ctx->data);
    i = 0;
  }

  while(i < 56u) {
    ctx->data[i++] = 0x00u;
  }

  ctx->bitlen += (uint64_t)ctx->datalen * 8u;
  ctx->data[56] = (uint8_t)(ctx->bitlen >> 56);
  ctx->data[57] = (uint8_t)(ctx->bitlen >> 48);
  ctx->data[58] = (uint8_t)(ctx->bitlen >> 40);
  ctx->data[59] = (uint8_t)(ctx->bitlen >> 32);
  ctx->data[60] = (uint8_t)(ctx->bitlen >> 24);
  ctx->data[61] = (uint8_t)(ctx->bitlen >> 16);
  ctx->data[62] = (uint8_t)(ctx->bitlen >> 8);
  ctx->data[63] = (uint8_t)(ctx->bitlen);
  ota_sha256_transform(ctx, ctx->data);

  for(i = 0; i < 4u; ++i) {
    hash[i]      = (uint8_t)((ctx->state[0] >> (24u - i * 8u)) & 0xFFu);
    hash[i + 4]  = (uint8_t)((ctx->state[1] >> (24u - i * 8u)) & 0xFFu);
    hash[i + 8]  = (uint8_t)((ctx->state[2] >> (24u - i * 8u)) & 0xFFu);
    hash[i + 12] = (uint8_t)((ctx->state[3] >> (24u - i * 8u)) & 0xFFu);
    hash[i + 16] = (uint8_t)((ctx->state[4] >> (24u - i * 8u)) & 0xFFu);
    hash[i + 20] = (uint8_t)((ctx->state[5] >> (24u - i * 8u)) & 0xFFu);
    hash[i + 24] = (uint8_t)((ctx->state[6] >> (24u - i * 8u)) & 0xFFu);
    hash[i + 28] = (uint8_t)((ctx->state[7] >> (24u - i * 8u)) & 0xFFu);
  }
}
