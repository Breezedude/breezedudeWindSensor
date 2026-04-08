#include "radio.h"

LORA_MODULE lora_module = LORA_NONE;

SX1276 radio_sx1276 = new Module(PIN_LORA_CS, PIN_LORA_DIO0, PIN_LORA_RESET, PIN_LORA_DIO1);
SX1262 radio_sx1262 = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RESET, PIN_LORA_DIO2);
LLCC68 radio_llcc68 = new Module(PIN_LORA_CS, PIN_LORA_DIO1, PIN_LORA_RESET, PIN_LORA_DIO2);

PhysicalLayer* radio_phy = nullptr;
bool transmittedFlag = false;
bool loraReceivedFlag = false;


void set_fanet_send_flag(void) {
  transmittedFlag = true;
}

void irq_lora_rec(){
  loraReceivedFlag = true;
  wakeup_source = WAKEUP_LORA; // is a special case of WAKEUP_EIC
  DEBUGSER.println(F("Got FANET message"));
  fanet_rx();
}

void en_rx_sleep(){
    bool smart_mode = true;

    if(!smart_mode){
        radio_phy->startReceive();
    } else {
        radio_phy->standby();
        RadioLibIrqFlags_t irqFlags = RADIOLIB_IRQ_RX_DONE | RADIOLIB_IRQ_TX_DONE;
        RadioLibIrqFlags_t irqMask  = RADIOLIB_IRQ_RX_DONE | RADIOLIB_IRQ_TX_DONE | RADIOLIB_IRQ_TIMEOUT;
        if((lora_module == LORA_SX1262) || (lora_module == LORA_LLCC68)){
            radio_phy->clearIrqFlags(radio_phy->getIrqFlags() & (RADIOLIB_IRQ_SYNC_WORD_VALID | RADIOLIB_IRQ_PREAMBLE_DETECTED));
            int err = static_cast<SX126x*>(radio_phy)->startReceiveDutyCycleAuto(8,4, irqFlags,irqMask ); // workaround as virtual function is missing in PysicalLayer class
            if (err != RADIOLIB_ERR_NONE){
                log_i ("Faild to set to rx mode, ", err);
            }
            static_cast<SX126x*>(radio_phy)->clearDio1Action();
            

        } else if(lora_module == LORA_SX1276){
        // not supported yet
        //radio_phy->sleep();
        radio_phy->startReceive();
        return;
        }
    }
    //radio_phy->setPacketReceivedAction(irq_lora_rec); see line below
    //static_cast<SX126x*>(radio_phy)->setDio1Action(irq_lora_rec);
    attachInterruptWakeup(PIN_LORA_DIO1, irq_lora_rec, RISING, false);
    DEBUGSER.println(F("Set Radio to smart RX mode"));
    // sleep
}

void dis_rx_sleep(){
  if(settings.lora_smart_rcv){
    detachInterrupt(PIN_LORA_DIO1);
  }
  radio_phy->standby();
  // Restore TX-done callback: clearDio1Action() in en_rx_sleep() removed the MCU
  // DIO1 interrupt, and startTransmit() does NOT re-attach it automatically.
  // Without this, transmittedFlag is never set and every TX times out.
  radio_phy->setPacketSentAction(set_fanet_send_flag);
  //DEBUGSER.println(F("Set Radio to idle"));
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
        settings.lbt_counter++;
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
      
      radio_phy->standby();  // Back to standby
      
      log_if("LBT: attempt %d/%d RSSI avg %d dBm (threshold %d dBm)\r\n", 
            i + 1, max_attempts, rssi_avg, settings.lora_rssi_threshold);
      
      if (rssi_avg < settings.lora_rssi_threshold) {
        led_error(0);
        return true;  // Channel clear
      }

      // Channel busy – short random backoff before retry
      if (i < max_attempts - 1) {
        delay(5 + (millis() % 20)); // 5-24 ms pseudo-random
      }
    }

    log_i("LBT: channel not free after retries\r\n");
    led_error(0);
    return false; // channel still busy after all attempts
  } else {
    log_v("LBT: RSSI scan not supported for this module\r\n");
    return true; // Assume channel is free if RSSI check not supported
  }
}

void radio_sleep(){
if(settings.lora_smart_rcv){
  en_rx_sleep();
} else {
  radio_phy->sleep();
}
}

void fanet_rx(){
  if(loraReceivedFlag) {
    loraReceivedFlag = false;

    int numBytes = radio_phy->getPacketLength();
    byte byteArr[numBytes];
    int state = radio_phy->readData(byteArr, numBytes);

    if (state == RADIOLIB_ERR_NONE) {

      DEBUGSER.println(F("[SX1262] Received packet!"));
       DEBUGSER.print(F("[SX1262] Data:\t\t"));
       char buf [4];
       for ( int i =0; i< numBytes; i++){
         sprintf(buf,"%02X ", byteArr[i]);
         DEBUGSER.print(buf);
       }
       DEBUGSER.println();

    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      DEBUGSER.println(F("CRC error!"));
      return;
    } else {
      DEBUGSER.print(F("failed, code "));
      DEBUGSER.println(state);
      return;
    }

    fanet_header *header = (fanet_header *)byteArr;
    if(header->type == FANET_PCK_TYPE_WEATHER){
      if(header->forward){
        // queque package for forwarding
      }
    }
  }
}


bool radio_init(){
  if(settings.skip_lora){return false;}

  if(zone_not_eu()){ // check GPS coordinates for freuency selection
    settings.lora_freq = LORA_FREQ_NA;
    settings.lora_bw = LORA_BW_NA;
  }



    if(radio_sx1276.begin(settings.lora_freq, settings.lora_bw, settings.lora_sf, settings.lora_cr, LORA_SYNCWORD, 10, 12, 0) == RADIOLIB_ERR_NONE){
      radio_phy = (PhysicalLayer*)&radio_sx1276;
      log_i("Found LoRa SX1276\r\n");
      lora_module = LORA_SX1276;
      return true;
    } 
    if(radio_llcc68.begin(settings.lora_freq, settings.lora_bw, settings.lora_sf, settings.lora_cr, LORA_SYNCWORD, 10, 12) == RADIOLIB_ERR_NONE){
      radio_phy = (PhysicalLayer*)&radio_llcc68;
      log_i("Found LoRa LLCC68\r\n");
      lora_module = LORA_LLCC68;
      radio_phy->setOutputPower(22);
      return true;
    }
    if(radio_sx1262.begin(settings.lora_freq, settings.lora_bw, settings.lora_sf, settings.lora_cr, LORA_SYNCWORD, 10, 12) == RADIOLIB_ERR_NONE){
      // NiceRF SX1262 issue https://github.com/jgromes/RadioLib/issues/689
      radio_phy = (PhysicalLayer*)&radio_sx1262;
      log_i("Found LoRa SX1262\r\n");
      lora_module = LORA_SX1262;
      radio_phy->setOutputPower(22);
      return true;
    }  else {
        log_i("No LoRa found\r\n");
        return false;
    }
}