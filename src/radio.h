#include <Arduino.h>
#include <RadioLib.h>

#ifndef BREEZEDUDE_RADIO_SX1276
  #define BREEZEDUDE_RADIO_SX1276 1
#endif
#ifndef BREEZEDUDE_RADIO_SX1262
  #define BREEZEDUDE_RADIO_SX1262 1
#endif
#ifndef BREEZEDUDE_RADIO_LLCC68
  #define BREEZEDUDE_RADIO_LLCC68 1
#endif

#include "tools.h"
#include "fanet.h"
#include "defines.h"
#include "logging.h"


extern LORA_MODULE lora_module;
#define LORA_SYNCWORD 0xF1 //SX1262: 0xF4 0x14 https://blog.classycode.com/lora-sync-word-compatibility-between-sx127x-and-sx126x-460324d1787a is handled by RadioLib

extern volatile int wakeup_source;


extern PhysicalLayer* radio_phy;
extern volatile bool transmittedFlag;
extern volatile bool loraReceivedFlag;

// Runtime counters reset on boot, exposed via HW Info debug type 3
extern uint16_t fanet_forward_counter;
extern uint16_t fanet_rx_counter;


void set_fanet_send_flag(void);

void irq_lora_rec();

void en_rx_sleep();

void dis_rx_sleep();


void radio_sleep();

// Listen-Before-Talk channel check. Returns true when safe to transmit.
// Call after dis_rx_sleep(). max_attempts: number of CAD retries (default 5).
bool lbt_channel_free(int max_attempts = 5);

bool fanet_rx();
// Returns ms until the queued forward packet should be sent, 0 if none pending
// or if the scheduled time has already passed. Use to reschedule RTC wakeup.
uint32_t fanet_forward_delay_ms();
// Transmits a queued FANET forward packet. Call from main loop when no TX is active.
// Returns true when transmission was started (caller should set send_active).
bool fanet_forward_check();
bool radio_init();