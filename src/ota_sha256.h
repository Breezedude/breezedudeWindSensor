#pragma once

#include <Arduino.h>

typedef struct {
  uint8_t data[64];
  uint32_t datalen;
  uint64_t bitlen;
  uint32_t state[8];
} ota_sha256_ctx_t;

void ota_sha256_init(ota_sha256_ctx_t *ctx);
void ota_sha256_update(ota_sha256_ctx_t *ctx, const uint8_t *data, size_t len);
void ota_sha256_final(ota_sha256_ctx_t *ctx, uint8_t hash[32]);
