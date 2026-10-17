#pragma once
extern bool undervoltage;
extern bool deep_undervoltage;
extern bool reduced_interval;
void power_duty_get(uint32_t *awake_ms, uint32_t *total_ms);
