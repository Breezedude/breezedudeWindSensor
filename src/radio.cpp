#include "radio.h"
#include "sensors.h"
#include "fanet_sched.h"

LORA_MODULE lora_module = LORA_NONE;

// Forward queue: one packet buffer to relay received FANET packets
static bool     forward_pending    = false;
static uint8_t  forward_buf[255]   = {0};
static int      forward_len        = 0;
static uint32_t forward_after_time = 0;


#define FANET_FORWARD_RSSI_THRESHOLD -92 // dBm, only forward packets with weaker signal to extend range of distant nodes

// Dedup cache: suppress re-forwarding the same packet within TTL. The key is
// the sender together with the packet type. Keyed on the sender alone it threw
// away everything a station sent after its first packet: scheduler.cpp holds
// each of a station's own transmissions apart by fanet_cooldown (4 s), so its
// name, weather and HWInfo packets arrive within the TTL of each other, and
// only the first of them was passed on. Measured on the rig: name at 191.8 s
// forwarded, weather 4.5 s later dropped, HWInfo 3.9 s after that dropped.
#define FWD_DEDUP_LEN 16
#define FWD_DEDUP_TTL 10000u  // ms
struct FwdCacheEntry { uint32_t key; uint32_t last_seen; };
static FwdCacheEntry fwd_cache[FWD_DEDUP_LEN] = {};
static uint8_t fwd_cache_idx = 0;

// A FANET sender id is 24 bits (vendor << 16 | address), so the packet type
// fits above it and the entry stays two words.
static inline uint32_t fwd_key(uint32_t sender_id, uint8_t type) {
  return (sender_id & 0x00FFFFFFu) | ((uint32_t)type << 24);
}

// Runtime counters, reset on boot, exposed via HW Info debug type 3
uint16_t fanet_forward_counter = 0;
uint16_t fanet_rx_counter      = 0;

// Receiver bookkeeping for the repeater report: whether RX was left on at the
// last radio_sleep(), since when, and how much of the time since the last
// reset it has been on. rx_armed says the chip is in RX right now, which is
// when its instantaneous RSSI reads the channel.
static bool rx_on = false;
static bool rx_armed = false;
static uint32_t rx_on_since = 0;
static uint32_t rx_on_ms = 0;
static uint32_t rx_duty_since = 0;

static void rx_account(uint32_t now){
  if(rx_on) rx_on_ms += now - rx_on_since;
  rx_on_since = now;
}
void radio_rx_duty_get(uint32_t *rx_ms, uint32_t *total_ms){
  const uint32_t now = time();
  rx_account(now);
  *rx_ms = rx_on_ms;
  *total_ms = now - rx_duty_since;
}
void radio_rx_duty_reset(){
  const uint32_t now = time();
  rx_account(now);
  rx_on_ms = 0;
  rx_duty_since = now;
}

static int16_t noise_min = 0, noise_max = 0;
static int32_t noise_sum = 0;
static uint32_t noise_n = 0;
void noise_note(int16_t rssi_dbm){
  if(rssi_dbm > -20 || rssi_dbm < -150) return;   // not a reading of the channel
  if(!noise_n || rssi_dbm < noise_min) noise_min = rssi_dbm;
  if(!noise_n || rssi_dbm > noise_max) noise_max = rssi_dbm;
  if(noise_n == 0xFFFFFFu){ noise_sum = 0; noise_n = 0; }  // long before the sum could overflow
  noise_sum += rssi_dbm;
  noise_n++;
}
bool noise_stats(int16_t *min_dbm, int16_t *avg_dbm, int16_t *max_dbm, uint32_t *samples){
  if(!noise_n) return false;
  *min_dbm = noise_min;
  *max_dbm = noise_max;
  *avg_dbm = (int16_t)(noise_sum / (int32_t)noise_n);
  *samples = noise_n;
  return true;
}

static bool fwd_dedup_check_and_add(uint32_t sender_id, uint8_t type) {
  const uint32_t key = fwd_key(sender_id, type);
  uint32_t now = time();
  for (int i = 0; i < FWD_DEDUP_LEN; i++) {
    if (fwd_cache[i].key == key &&
        (now - fwd_cache[i].last_seen) < FWD_DEDUP_TTL) {
      return false; // this packet was already forwarded
    }
  }
  fwd_cache[fwd_cache_idx] = {key, now};
  fwd_cache_idx = (fwd_cache_idx + 1) % FWD_DEDUP_LEN;
  return true;
}

#if BREEZEDUDE_RADIO_SX1276
SX1276 radio_sx1276 = new Module(PIN_LORA_CS, PIN_LORA_DIO0, PIN_LORA_RESET, PIN_LORA_DIO1);
#endif
#if BREEZEDUDE_RADIO_SX1262
SX1262 radio_sx1262 = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RESET, PIN_LORA_DIO2);
#endif
#if BREEZEDUDE_RADIO_LLCC68
LLCC68 radio_llcc68 = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RESET, PIN_LORA_DIO2);
#endif

PhysicalLayer* radio_phy = nullptr;
volatile bool transmittedFlag = false;
volatile bool loraReceivedFlag = false;


void set_fanet_send_flag(void) {
  transmittedFlag = true;
}

void irq_lora_rec(){
  loraReceivedFlag = true;
  wakeup_source = WAKEUP_LORA; // is a special case of WAKEUP_EIC
  // NOTE: do not call fanet_rx() or DEBUGSER here – ISR context; SPI may be
  // unstable immediately after deep-sleep wakeup. fanet_rx() is called from
  // the main loop and also from go_sleep() after the wakeup path.
}

void en_rx_sleep(){
    loraReceivedFlag = false;
    if((lora_module == LORA_SX1262) || (lora_module == LORA_LLCC68)){
        // Use plain continuous receive – duty-cycle auto mode was unreliable
        // on LLCC68 with FANET's SF7/BW250 preamble timing.
        static_cast<SX126x*>(radio_phy)->clearIrqFlags(0xFFFF);
        int err = radio_phy->startReceive();
        if (err != RADIOLIB_ERR_NONE){
            log_w("en_rx_sleep: startReceive failed, code=", err);
            return;
        }
        static_cast<SX126x*>(radio_phy)->clearDio1Action();
        attachInterruptWakeup(PIN_LORA_DIO1, irq_lora_rec, RISING, false);
        rx_armed = true;
        //log_i("Radio: RX mode active (LLCC68/SX1262)\r\n");
    } else if(lora_module == LORA_SX1276){
        int err = radio_phy->startReceive();
        if(err != RADIOLIB_ERR_NONE){
            log_w("en_rx_sleep: startReceive SX1276 failed, code=", err);
            return;
        }
        attachInterruptWakeup(PIN_LORA_DIO0, irq_lora_rec, RISING, false); // SX1276: RX done on DIO0
        //log_i("Radio: RX mode active (SX1276)\r\n");
    }
}

void dis_rx_sleep(){
  // Always detach interrupt unconditionally: the interrupt was attached based on
  // lora_module, not on lora_smart_rcv. If we gate the detach on the setting,
  // the interrupt stays armed when conditions change (e.g. battery drops below
  // threshold) and keeps waking the MCU → 6mA constant drain in deep sleep.
  if(lora_module == LORA_SX1276){
    detachInterrupt(PIN_LORA_DIO0);
  } else {
    detachInterrupt(PIN_LORA_DIO1);
  }
  radio_phy->standby();
  rx_armed = false;
  // NOTE: setPacketSentAction() is NOT called here. Calling it here re-arms
  // the DIO1 MCU interrupt right before sleep(), which is the root cause of
  // the 6mA drain bug: a pending LLCC68 IRQ fires immediately after attach,
  // setting transmittedFlag, causing finishTransmit() to wake the radio from
  // sleep, then radio_sleep() with stale batt_perc puts it back in RX.
  // setPacketSentAction() is instead called in each TX path just before
  // startTransmit(), where the interrupt is actually needed.
}


// Listen-Before-Talk: RSSI-based channel check.
// Returns true if channel is clear (RSSI below threshold).
// Active RSSI measurement (enters RX briefly to measure current channel noise).
// Continuously samples RSSI and uses average value for decision.
// Must be called AFTER dis_rx_sleep() (radio in standby).
bool lbt_channel_free(int max_attempts) {
  if (!radio_phy) {
    log_v("LBT: skip RSSI scan, radio not initialized\r\n");
    return true;
  }

  if((lora_module == LORA_SX1262) || (lora_module == LORA_LLCC68)){

    for (int i = 0; i < max_attempts; i++) {
      if(i > 0){
        led_error(1);
        state.lbt_counter++;
      }
      // Start RX briefly to get active RSSI measurement
      radio_phy->startReceive();
      
      // Continuously sample RSSI over time window
      int32_t rssi_sum = 0;
      const int num_samples = 10;  // ~1ms per sample = 10ms total
      for (int s = 0; s < num_samples; s++) {
        int16_t rssi = static_cast<SX126x*>(radio_phy)->getRSSI(false); // instantaneous RSSI
        rssi_sum += rssi;
        delayMicroseconds(100);  // 0.1ms between samples
      }
      int16_t rssi_avg = (int16_t)(rssi_sum / num_samples);
      noise_note(rssi_avg);
      
      radio_phy->standby();  // Back to standby
      
      log_v("LBT: attempt ");
      log_write(i + 1);
      log_v("/");
      log_write(max_attempts);
      log_v(" RSSI avg ");
      log_write(rssi_avg);
      log_v(" dBm (threshold ");
      log_write(settings.lora_rssi_threshold);
      log_v(" dBm)\r\n");
      
      if (rssi_avg < settings.lora_rssi_threshold) {
        led_error(0);
        return true;  // Channel clear
      }

      // Channel busy – short random backoff before retry
      if (i < max_attempts - 1) {
        delay(5 + (millis() % 20)); // 5-24 ms pseudo-random
      }
    }

    log_w("LBT: channel not free after retries\r\n");
    led_error(0);
    return false; // channel still busy after all attempts
  } else {
    log_v("LBT: RSSI scan not supported for this module\r\n");
    return true; // Assume channel is free if RSSI check not supported
  }
}

// The scheduled repeater governs the receiver itself, see fanet_sched.h - but
// not while the battery is full and the sun is on it. There is nothing to save
// then, and hearing everything is worth more than the milliamps.
bool repeater_sched_active(){
  extern bool undervoltage;
  if(!settings.repeater || !settings.repeater_sched || undervoltage){ return false; }
  const bool power_to_spare = sensor.batt_perc > 95 && (sensor.pv_charging || sensor.pv_done);
  return !power_to_spare;
}

void radio_sleep(){
  // Repeater: always-on RX except during undervoltage, or when the schedule
  // says the next packet is not due yet.
  // Smart-rcv: only when battery is sufficiently charged and solar is active.
  extern bool undervoltage;
  const uint32_t now = time();
  rx_account(now);
  // A receiver that sat in RX through the sleep reads the channel as it is
  // between packets - one free noise sample per wakeup.
  if(rx_armed && ((lora_module == LORA_SX1262) || (lora_module == LORA_LLCC68))){
    noise_note((int16_t)static_cast<SX126x*>(radio_phy)->getRSSI(false));
  }
  const bool scheduled = repeater_sched_active();
  const bool en_rx = (settings.repeater && !undervoltage && (!scheduled || sched_rx_wanted(now))) ||
                     (settings.lora_smart_rcv && sensor.batt_perc >= 90 && (sensor.pv_charging || sensor.pv_done));
  rx_on = en_rx;
  if(en_rx){
    en_rx_sleep();
  } else {
    // dis_rx_sleep() detaches the EIC interrupt and puts radio in standby first,
    // then sleep() drives it into lowest-power PHY sleep (~1µA on SX126x).
    dis_rx_sleep();
    if((lora_module == LORA_SX1262) || (lora_module == LORA_LLCC68)){
      // Clear all pending IRQ flags so DIO1 is driven LOW before sleep.
      // If flags are left set, DIO1 stays HIGH during sleep and any re-attached
      // interrupt would fire immediately on the next en_rx_sleep or TX transition.
      radio_phy->clearIrqFlags(0xFFFF);
      static_cast<SX126x*>(radio_phy)->sleep(true); // warm start: retains config, lowest current draw
    } else {
      radio_phy->sleep();
    }
  }
}

bool fanet_rx(){
  // When the packet arrived, as far as the station can tell. The way from the
  // DIO1 interrupt through the wakeup to here is short and always the same, so
  // the schedule in fanet_sched.h can measure periods with it.
  const uint32_t t_rx = time();
  if(loraReceivedFlag) {
    loraReceivedFlag = false;

    int numBytes = radio_phy->getPacketLength();
    // Fixed buffer instead of a VLA: getPacketLength() is unvalidated radio
    // input, 0 would form a zero-length array and 255 is a large stack
    // allocation on the path right after a deep-sleep wakeup.
    if(numBytes <= 0 || numBytes > (int)sizeof(forward_buf)) {
      log_w("FANET RX: bad packet length, dropped len=", numBytes);
      return true; // packet consumed, caller re-arms RX
    }
    uint8_t byteArr[sizeof(forward_buf)];
    int state = radio_phy->readData(byteArr, numBytes);
    // readData() leaves the radio in standby; caller must re-arm RX when safe.
    rx_armed = false;

    if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      // One line a minute, not one per packet: a noisy band produced three of
      // these a second, and a log full of them says no more than one with a
      // count. The first error of a quiet minute is reported at once, the rest
      // of that minute is summed up with the next one.
      static uint32_t crc_window = 0, crc_count = 0;
      crc_count++;
      if(!crc_window || (time() - crc_window) >= 60000u){
        if(crc_count > 1){ log_w("FANET RX: CRC errors since the last line: ", crc_count - 1); }
        log_w("FANET RX: CRC error\r\n");
        crc_window = time();
        crc_count = 1;
      }
      return true; // packet consumed, re-arm is caller's responsibility
    } else if (state != RADIOLIB_ERR_NONE) {
      log_w("FANET RX: read failed, code=", state);
      return true;
    }

    fanet_header *header = (fanet_header *)byteArr;
    fanet_rx_counter++;

    float rssi = radio_phy->getRSSI();
    float snr  = radio_phy->getSNR();
    log_i("FANET RX #");
    log_write(fanet_rx_counter);
    log_i(": type=0x");
    log_write_hex(header->type, 2);
    log_i(" vendor=0x");
    log_write_hex(header->vendor, 2);
    log_i(" addr=0x");
    log_write_hex(header->address, 4);
    log_i(" fwd=");
    log_write((int)header->forward);
    log_i(" len=");
    log_write(numBytes);
    log_i(" RSSI=");
    log_write(rssi);
    log_i(" SNR=");
    log_write(snr);
    log_i("\r\n");
    log_v_hex_dump(byteArr, numBytes); // raw packet bytes when USB_VERBOSE=1

    // Forward any packet with the forward bit set (spec: clear forward bit on re-TX
    // to prevent cascaded forwarding). Skip if a forward is already queued or
    // we recently forwarded from this sender (dedup).
    const bool rssi_ok = settings.repeater || (rssi < FANET_FORWARD_RSSI_THRESHOLD);
    const uint32_t sender_id = ((uint32_t)header->vendor << 16) | header->address;
    // The schedule learns the rhythm of every station that asks to be
    // forwarded - the weather packets it exists for, and the HWInfo report of
    // the same station as a second rhythm. This happens even when this packet
    // itself is not forwarded because one is already queued. The classic
    // repeater learns too, without acting on it: its repeater report then
    // shows the periods and misses per station, and whether the rhythms are
    // stable enough to switch to REPEATER_SCHED.
    if (settings.repeater && header->forward) {
      if (header->type == FANET_PCK_TYPE_WEATHER) {
        sched_note_weather(sender_id, t_rx);
      } else if (header->type == FANET_PCK_TYPE_HWINFO) {
        sched_note_hwinfo(sender_id, t_rx);
      }
    }
    if (header->forward && !forward_pending && rssi_ok) { // repeater: forward all; smart_rcv: only weak signals
      if (fwd_dedup_check_and_add(sender_id, header->type)) {
        byteArr[0] &= ~(1u << 6); // clear forward bit
        if (numBytes <= (int)sizeof(forward_buf)) {
          memcpy(forward_buf, byteArr, numBytes);
          forward_len     = numBytes;
          forward_pending = true;
          uint32_t delay_ms = 500u + (millis() % 1000u);
          forward_after_time = time() + delay_ms;
          log_i("FANET: queued fwd type=0x");
          log_write_hex(header->type, 2);
          log_i(" from ");
          log_write_hex(header->vendor, 2);
          log_i(":");
          log_write_hex(header->address, 4);
          log_i(" len=");
          log_write(numBytes);
          log_i(" delay=");
          log_write(delay_ms);
          log_i("ms\r\n");
        }
      } else {
        log_d("FANET: skip fwd (dedup) from ");
        log_write_hex(header->vendor, 2);
        log_d(":");
        log_write_hex(header->address, 4);
        log_d("\r\n");
      }
    }
    return true; // packet consumed
  }
  return false; // nothing received
}

uint32_t fanet_forward_delay_ms() {
  if (!forward_pending) return 0;
  if (time_reached(forward_after_time)) return 0;
  return forward_after_time - time();
}

// Call from main loop (when no TX is active) to transmit a queued forward packet.
bool fanet_forward_check() {
  if(!forward_pending || !time_reached(forward_after_time)) {
    return false;
  }
  forward_pending = false;
  dis_rx_sleep();
  if(settings.lora_lbt && !lbt_channel_free()) {
    // Channel busy – reschedule with a fresh random delay
    forward_pending = true;
    forward_after_time = time() + 500 + (millis() % 1500);
    radio_sleep();
    log_w("FANET fwd: channel busy, deferred\r\n");
    return false;
  }
  // Arm TX-done callback just before transmit (not in dis_rx_sleep, to avoid
  // spurious interrupt while the radio is being put to PHY sleep).
  radio_phy->setPacketSentAction(set_fanet_send_flag);
  int state = radio_phy->startTransmit(forward_buf, forward_len);
  if(state != RADIOLIB_ERR_NONE) {
    log_w("FANET fwd: TX failed, code=", state);
    radio_sleep();
    return false;
  }
  fanet_forward_counter++;
  log_i("FANET fwd #");
  log_write(fanet_forward_counter);
  log_i(": TX started len=");
  log_write(forward_len);
  log_i("\r\n");
  return true;
}


bool radio_init(){
  if(state.skip_lora){return false;}

  if(zone_not_eu()){ // check GPS coordinates for freuency selection
    settings.lora_freq = LORA_FREQ_NA;
    settings.lora_bw = LORA_BW_NA;
  }

  #if BREEZEDUDE_RADIO_SX1276
  if(radio_sx1276.begin(settings.lora_freq, settings.lora_bw, settings.lora_sf, settings.lora_cr, LORA_SYNCWORD, 10, 12, 0) == RADIOLIB_ERR_NONE){
    radio_phy = (PhysicalLayer*)&radio_sx1276;
    log_i("Radio: LoRa SX1276\r\n");
    lora_module = LORA_SX1276;
    return true;
  }
  #endif

  #if BREEZEDUDE_RADIO_LLCC68
  const int llcc68_state = radio_llcc68.begin(settings.lora_freq, settings.lora_bw, settings.lora_sf, settings.lora_cr, LORA_SYNCWORD, 10, 12);
  if(llcc68_state == RADIOLIB_ERR_NONE){
    radio_phy = (PhysicalLayer*)&radio_llcc68;
    log_i("Radio: LoRa LLCC68\r\n");
    lora_module = LORA_LLCC68;
    radio_phy->setOutputPower(22);
    static_cast<SX126x*>(radio_phy)->setRegulatorDCDC(); // DC-DC saves ~1-2mA in RX vs LDO default
    return true;
  }
  #endif

  #if BREEZEDUDE_RADIO_SX1262
  if(radio_sx1262.begin(settings.lora_freq, settings.lora_bw, settings.lora_sf, settings.lora_cr, LORA_SYNCWORD, 10, 12) == RADIOLIB_ERR_NONE){
    // NiceRF SX1262 issue https://github.com/jgromes/RadioLib/issues/689
    radio_phy = (PhysicalLayer*)&radio_sx1262;
    log_i("Radio: LoRa SX1262\r\n");
    lora_module = LORA_SX1262;
    radio_phy->setOutputPower(22);
    static_cast<SX126x*>(radio_phy)->setRegulatorDCDC(); // DC-DC saves ~1-2mA in RX vs LDO default
    return true;
  }
  #endif

  // The code tells a missing chip (-2) from a refused parameter, such as a
  // bandwidth the module does not have (-12). "Not found" alone hid that.
  #if BREEZEDUDE_RADIO_LLCC68
  log_w("Radio: LLCC68 begin() code=", llcc68_state);
  #endif
  log_e("Radio: No LoRa found\r\n");
  return false;
}