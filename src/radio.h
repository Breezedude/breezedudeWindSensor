#include <Arduino.h>
#include <RadioLib.h>
#include "tools.h"
#include "fanet.h"
#include "defines.h"
#include "logging.h"


extern LORA_MODULE lora_module;
#define LORA_SYNCWORD 0xF1 //SX1262: 0xF4 0x14 https://blog.classycode.com/lora-sync-word-compatibility-between-sx127x-and-sx126x-460324d1787a is handled by RadioLib

extern int wakeup_source;


extern PhysicalLayer* radio_phy;
extern bool transmittedFlag;
extern bool loraReceivedFlag;


void set_fanet_send_flag(void);

void irq_lora_rec();

void en_rx_sleep();

void dis_rx_sleep();


void radio_sleep();

void fanet_rx();
bool radio_init();