#pragma once
extern bool undervoltage;
extern bool deep_undervoltage;
extern bool reduced_interval;
void power_duty_get(uint32_t *awake_ms, uint32_t *total_ms);
void power_status_window_take(uint32_t *awake_ms, uint32_t *total_ms);
