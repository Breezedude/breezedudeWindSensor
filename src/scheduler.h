#pragma once
#include <Arduino.h>

// FANET message scheduling: when each message type is due, sending it, and
// working out how long the station may sleep afterwards.
// Split out of main.cpp together with power.cpp.

extern uint32_t next_tx_time;            // time() of the next planned transmission
extern uint32_t broadcast_scale_factor;  // interval multiplier, raised when the battery is low
extern uint32_t last_fnet_send;          // time() of the last packet handed to the radio
extern uint32_t fanet_cooldown;          // minimum gap between two packets, ms
extern bool force_wait_for_wdata;        // one-shot short sleep while waiting for wind data

// Milliseconds until the station has to be awake again.
uint32_t calc_time_to_sleep();

// Start every transmission that is currently due.
void tx_run_jobs();

// Handle TX completion and the send timeout.
void tx_service_active_send();

// True while a packet is in the radio and the station must stay awake.
bool tx_send_in_progress();

// Mark a transmission as started (used for forwarded packets, which are sent
// from radio.cpp rather than from a scheduled job).
void tx_note_send_started();
