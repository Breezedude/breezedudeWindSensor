// Host stub: the parts of radio.h that status.cpp reads.
#pragma once
#include <stdint.h>
void radio_rx_duty_get(uint32_t *rx_ms, uint32_t *total_ms);
bool noise_stats(int16_t *min_dbm, int16_t *avg_dbm, int16_t *max_dbm, uint32_t *samples);
