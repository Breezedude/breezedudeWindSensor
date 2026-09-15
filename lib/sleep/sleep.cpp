#include "sleep.h"
#include "wiring_private.h"




void WDT_Handler(void) {
  // ISR for watchdog early warning, DO NOT RENAME!
  WDT->CTRL.bit.ENABLE = 0; // Disable watchdog
  while (WDT->STATUS.bit.SYNCBUSY)
    ; // Sync CTRL write
  WDT->INTFLAG.bit.EW = 1; // Clear interrupt flag
}

// Change BOD33 action from RESET to INTERRUPT so that a brownout below ~2.8 V
// triggers our ISR instead of resetting the MCU into the DFU bootloader.
// Must be called early in setup() – ideally before any other initialisation.
void setup_BOD33_interrupt() {
  SYSCTRL->BOD33.bit.ENABLE   = 0;            // Disable while reconfiguring
  SYSCTRL->BOD33.bit.ACTION   = 2;            // 0=None, 1=Reset (default), 2=Interrupt
  SYSCTRL->BOD33.bit.RUNSTDBY = 0;            // Off during standby – saves ~30µA; bootloader timeout handles stuck-in-DFU
  SYSCTRL->BOD33.bit.HYST     = 1;            // Hysteresis – prevents rapid re-trigger
  SYSCTRL->BOD33.bit.ENABLE   = 1;
  while (!SYSCTRL->PCLKSR.bit.BOD33RDY);      // Wait until BOD33 has settled

  SYSCTRL->INTENSET.bit.BOD33DET = 1;         // Enable BOD33 detect interrupt
  NVIC_ClearPendingIRQ(SYSCTRL_IRQn);
  NVIC_SetPriority(SYSCTRL_IRQn, 0);          // Highest priority
  NVIC_EnableIRQ(SYSCTRL_IRQn);
}

// Emergency brownout ISR.
// Fires instead of a CPU reset when VDD falls below the BOD33 threshold.
// Immediately isolates the UART TX pin (idles HIGH → leaks ~10 mA into the
// WS85 sensor via ESD diodes) and cuts sensor VCC, then returns so the
// normal sleep/UV logic can put the MCU back to standby.
extern "C" void SYSCTRL_Handler(void) {
  if (SYSCTRL->INTFLAG.bit.BOD33DET) {
    SYSCTRL->INTFLAG.bit.BOD33DET = 1;        // Clear interrupt flag

    // Disable UART TX – PA10 (Arduino pin 1 = PIN_TX).
    // Clearing PMUXEN disconnects the pin from SERCOM; setting as hi-Z input
    // removes the idle-HIGH drive that leaks current into the sensor.
    PORT->Group[0].PINCFG[10].reg = 0;        // clear PMUXEN + INEN
    PORT->Group[0].DIRCLR.reg     = (1u << 10);

    // Disable UART RX – PA11 (Arduino pin 0 = PIN_RX), symmetric isolation.
    PORT->Group[0].PINCFG[11].reg = 0;
    PORT->Group[0].DIRCLR.reg     = (1u << 11);

    // Assert sensor power-off gate: PB03 = PIN_PS_WS (active-high → VCC off).
    PORT->Group[1].DIRSET.reg = (1u << 3);
    PORT->Group[1].OUTSET.reg = (1u << 3);

    // ISR returns normally. The CPU resumes after __WFI() in deepsleep() and
    // the main loop's UV path will enter 30-minute standby sleep cycles until
    // the battery has recovered enough for normal operation.
  }
}


bool set_cpu_div(int divisor){
static int div = 0;
if(div != divisor){
    div = divisor;
    GCLK->GENDIV.reg = GCLK_GENDIV_DIV(divisor) |    // Divide the 48MHz clock source by divisor 48: 48MHz/48=1MHz
                       GCLK_GENDIV_ID(0);            // Select Generic Clock (GCLK) 0
    while (GCLK->STATUS.bit.SYNCBUSY){};             // Wait for synchronization
    return true;
    }
return false;
}

#ifdef WS85_TRACE
// WS85_TRACE debug build only, see the overview above ws85_trace_report() in
// src/sensors.cpp. What SERCOM0 did from enable_sercom0_int() until the report:
// start-of-frame (RXS), receive (RXC) and error interrupts, and when each of the
// first WS85_TRACE_RX_TIMES bytes arrived. Those arrival times are what showed
// the watchdog holding the CPU in the middle of a frame. Taking micros() in the
// interrupt costs time there, one more reason to keep this out of the release.
volatile uint32_t sercom0_rxs_count = 0;
volatile uint32_t sercom0_rxc_count = 0;
volatile uint32_t sercom0_err_count = 0;
volatile uint32_t sercom0_rx_us[WS85_TRACE_RX_TIMES]; // micros() at each RXC
volatile uint8_t  sercom0_rx_n = 0;                   // entries used in sercom0_rx_us
#endif

// SERCOM0_Handler overrides the weak default from variants/itsybitsy_m0/variant.cpp
void SERCOM0_Handler(void) {
#ifdef WS85_TRACE
    // Read the flags before anything clears them: RXS is cleared below, RXC and
    // ERROR by the core's handler.
    {
      const uint8_t f = SERCOM0->USART.INTFLAG.reg;
      if (f & SERCOM_USART_INTFLAG_RXS)   { sercom0_rxs_count++; }
      if (f & SERCOM_USART_INTFLAG_RXC) {
        sercom0_rxc_count++;
        if (sercom0_rx_n < WS85_TRACE_RX_TIMES) { sercom0_rx_us[sercom0_rx_n++] = micros(); }
      }
      if (f & SERCOM_USART_INTFLAG_ERROR) { sercom0_err_count++; }
    }
#endif
    if (SERCOM0->USART.INTFLAG.bit.RXS) {
        SERCOM0->USART.INTFLAG.reg = SERCOM_USART_INTFLAG_RXS; // clear start bit flag
    // Mark a real UART wakeup so USB IRQs can be ignored in CDC test mode.
    mark_uart_wakeup();
    
    }
    Serial1.IrqHandler();  // let Arduino core handle RXC
}



// The debug UART. Same shape as SERCOM0_Handler above: the start-of-frame flag
// has to be cleared by hand, and it is the only place that can tell a genuine
// wakeup on the console from a stray USB interrupt.
// Counters for diagnosing UART_WAKE.
//
// The question they answer now is what a wakeup with nobody sending actually
// was. RXS only says a 1-to-0 edge was seen on RxD; it does not say whether a
// valid frame followed. So RXC and ERROR are counted alongside it:
//
//   rxs up, rxc up, err flat  -> a real byte arrived, someone did send
//   rxs up, rxc flat, err up  -> the "start bit" was a glitch, the frame that
//                                followed was not a frame - line noise
//   rxs up, rxc flat, err flat-> an edge with nothing behind it at all
//
// STATUS has to be read here, before Serial2's handler clears it, or the
// distinction between a framing error and an overflow is gone by the time
// anything can log it.
volatile uint32_t sercom1_irq_count = 0;
volatile uint32_t sercom1_rxs_count = 0;
volatile uint32_t sercom1_rxc_count = 0;
volatile uint32_t sercom1_err_count = 0;
volatile uint32_t sercom1_last_status = 0;

void SERCOM1_Handler(){
  sercom1_irq_count++;
  const uint8_t flags = SERCOM1->USART.INTFLAG.reg;

  if (flags & SERCOM_USART_INTFLAG_RXC) { sercom1_rxc_count++; }
  if (flags & SERCOM_USART_INTFLAG_ERROR) {
    sercom1_err_count++;
    sercom1_last_status = SERCOM1->USART.STATUS.reg;
  }
  if (flags & SERCOM_USART_INTFLAG_RXS) {
    sercom1_rxs_count++;
    SERCOM1->USART.INTFLAG.reg = SERCOM_USART_INTFLAG_RXS;
    // Deliberately its own reason, not the sensor's: a console command has to
    // end the sleep window, whereas a byte from the sensor must not.
    mark_console_wakeup();
  }
  Serial2.IrqHandler();
}


// GCLK2 as a low power generator on OSCULP32K, set up exactly once.
//
// Configuring it once is hygiene rather than a fix: writing GENCTRL takes the
// generator down for a moment, and with a UART sensor enable_sercom0_int()
// attaches SERCOM0 to GCLK2 on every single sleep, so there is no reason to
// keep re-writing it underneath a running peripheral.
//
// It was once believed to be the cause of "Wakeup: 266, source NONE" repeating
// forever. It was not - that was a fresh boot each time, from a watchdog reset,
// and the note above enable_sercom0_int() has the real story.
static void ensure_gclk2_ulp(void) {
  static bool configured = false;
  if (configured) return;

  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(0);
  while (GCLK->STATUS.bit.SYNCBUSY);

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) |
                      GCLK_GENCTRL_SRC_OSCULP32K |
                      GCLK_GENCTRL_RUNSTDBY |
                      GCLK_GENCTRL_GENEN |
                      GCLK_GENCTRL_IDC |
                      GCLK_GENCTRL_DIVSEL;
  while (GCLK->STATUS.bit.SYNCBUSY);
  configured = true;
}

// GCLK3 as the fast start-up oscillator for SERCOM cores in standby.
//
// The Arduino core startup already puts OSC8M on generator 3 at 8 MHz and
// already clears OSC8M.ONDEMAND, so the oscillator keeps running instead of
// being started on request. Only RUNSTDBY is missing.
//
// This is what the datasheet asks for. AT11626 section 4.5 is explicit: "In
// standby sleep mode, the internal fast start-up oscillator must be selected
// as the GCLK_SERCOMx_CORE source." A 32 kHz generator is not that oscillator,
// and Microchip's own SOF example clears OSC8M.ONDEMAND for exactly one stated
// reason - "to avoid the loss of first character which wake up the device from
// sleep mode". Losing the first byte is therefore not a quirk to live with, it
// is the symptom of running the wrong clock.
// Two separate bits are needed, and missing the second one is subtle. The core
// clears OSC8M.ONDEMAND, which means "always on while enabled" - but that says
// nothing about standby. OSC8M.RUNSTDBY is what decides there: with 0 the
// oscillator is stopped in standby unless a peripheral is asking for it, and a
// SERCOM woken by a start bit asks too late for the first frame. Both bits are
// set here, plus RUNSTDBY on the generator that carries the clock.
static bool gclk3_osc8m_running = false;

static void ensure_gclk3_osc8m_stdby(void) {
  if (gclk3_osc8m_running) return;

  SYSCTRL->OSC8M.bit.ONDEMAND = 0;
  SYSCTRL->OSC8M.bit.RUNSTDBY = 1;

  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(3) |
                      GCLK_GENCTRL_SRC_OSC8M |
                      GCLK_GENCTRL_RUNSTDBY |
                      GCLK_GENCTRL_GENEN;
  while (GCLK->STATUS.bit.SYNCBUSY);
  gclk3_osc8m_running = true;
}

/**
 * Hand the 8 MHz oscillator back when nothing needs it.
 *
 * The Arduino core startup puts OSC8M on generator 3 with GENEN and clears
 * OSC8M.ONDEMAND, so from reset the oscillator runs the whole time the MCU is
 * awake - roughly 60-70 uA - even though nothing in this project consumes
 * GCLK3. Standby was never affected: generator 3 has no RUNSTDBY there, stops,
 * and stops requesting its source. This is purely an awake-time cost, and with
 * UART_WAKE off it buys nothing at all.
 *
 * Disabling the generator and handing ONDEMAND back means OSC8M only starts if
 * some peripheral actually asks for it.
 */
static void release_gclk3_osc8m(void) {
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(3);   // GENEN clear: generator off
  while (GCLK->STATUS.bit.SYNCBUSY);

  SYSCTRL->OSC8M.bit.RUNSTDBY = 0;
  SYSCTRL->OSC8M.bit.ONDEMAND = 1;
  gclk3_osc8m_running = false;
}

// A note on BAUD, because getting this wrong cost a long hunt.
//
// SERCOM::initUART() in the Arduino core programs the rate in **fractional**
// mode - it writes BAUD.FRAC.BAUD and BAUD.FRAC.FP and selects the mode with
// CTRLA.SAMPR. Recomputing the register with the arithmetic-mode formula
// therefore does not produce the same number at all: for 115200 at 48 MHz the
// arithmetic value is 63019, which read as a fractional register means
// BAUD=5675, FP=7 - about **528 baud**. The correct fractional value is 26.
//
// On SERCOM0 that never showed, because SENSOR_UART.begin() right after the
// wake reprograms the register anyway. On SERCOM1 nothing did, so the debug
// console dropped to 528 baud after the first sleep. That was the burst of
// garbage on the UART, and it is why log_flush() then sat there long enough
// for the 2500 ms watchdog to reset the station - "Reset cause: 0x20 [WDT]",
// every 37 s, which looked for a long time like a standby problem.
//
// The lesson is to use the same format the core uses, which is what
// standby_baud_reg() below does. SERCOM1's rate is now set once, for the clock
// it actually runs on, and never switched again.
//
// SERCOM0 had the same arithmetic value in disable_sercom0_int(), hidden for a
// long time by a begin() after every wakeup. It no longer writes the register.

/** The clock the debug UART runs on once configure_debug_uart_standby() ran. */
#define SERCOM_STANDBY_HZ   8000000UL
#define SERCOM_STANDBY_BPS  115200UL

/**
 * Baud register for the 8 MHz core clock, in the core's fractional format.
 *
 * Rounded rather than truncated, which is worth a word: the exact quotient is
 * 34.72, and SERCOM::initUART() takes the integer division, so the core would
 * program 34 - BAUD = 4, FP = 2, giving 117647 baud, 2.1 % fast. Rounding to
 * 35 gives BAUD = 4, FP = 3 and 114286 baud, 0.79 % slow. An 8N1 frame carries
 * roughly 3 %, so the rounded value leaves three times the margin for free.
 */
static uint16_t standby_baud_reg(void) {
  const uint32_t num = SERCOM_STANDBY_HZ * 8UL;
  const uint32_t den = 16UL * SERCOM_STANDBY_BPS;
  const uint32_t baud_times_8 = (num + den / 2UL) / den;
  return (uint16_t)(((baud_times_8 % 8UL) << 13) | (baud_times_8 / 8UL));
}

void enable_sercom0_int(void) {
#ifdef WS85_TRACE
    // WS85_TRACE: this sleep's interrupts and byte times count from here.
    sercom0_rxs_count = 0;
    sercom0_rxc_count = 0;
    sercom0_err_count = 0;
    sercom0_rx_n = 0;
#endif
    // 1) Make sure SERCOM0 APBC clock is enabled
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;

    // 2) Stop USART while we reconfigure
    SERCOM0->USART.CTRLA.bit.ENABLE = 0;
    while (SERCOM0->USART.SYNCBUSY.reg != 0); // wait any sync

    // 3) GCLK2 as OSCULP32K with RUNSTDBY, once. See ensure_gclk2_ulp().
    ensure_gclk2_ulp();

    // 4) Attach GCLK2 to SERCOM0 core and slow clocks (use CLKCTRL on SAMD21)
    // core
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM0_GCLK_ID_CORE) |
                        GCLK_CLKCTRL_GEN_GCLK2 |
                        GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.bit.SYNCBUSY);

    // slow (important for wake/detection)
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM0_GCLK_ID_SLOW) |
                        GCLK_CLKCTRL_GEN_GCLK2 |
                        GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.bit.SYNCBUSY);

    // 5) Configure SERCOM CTRLA (MODE + RUNSTDBY) and wait for sync
    SERCOM0->USART.CTRLA.reg = (SERCOM0->USART.CTRLA.reg & ~SERCOM_USART_CTRLA_MODE_Msk)
                              | SERCOM_USART_CTRLA_MODE_USART_INT_CLK; // keep other bits
    // set RUNSTDBY
    SERCOM0->USART.CTRLA.bit.RUNSTDBY = 1;
    while (SERCOM0->USART.SYNCBUSY.reg != 0); // wait until all SERCOM sync complete

    // 6) CTRLB: enable RX/TX and SFDE (start-frame detection)
    SERCOM0->USART.CTRLB.reg = SERCOM_USART_CTRLB_CHSIZE(0x0) |
                               SERCOM_USART_CTRLB_TXEN |
                               SERCOM_USART_CTRLB_RXEN |
                               SERCOM_USART_CTRLB_SFDE;
    while (SERCOM0->USART.SYNCBUSY.bit.CTRLB);

    // 7) Enable interrupts (RX start + RX complete)
    SERCOM0->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXS | SERCOM_USART_INTENSET_RXC;

    // 8) Enable NVIC for SERCOM0
    NVIC_ClearPendingIRQ(SERCOM0_IRQn);
    NVIC_SetPriority(SERCOM0_IRQn, 0);
    NVIC_EnableIRQ(SERCOM0_IRQn);

    // 9) Re-enable USART
    SERCOM0->USART.CTRLA.bit.ENABLE = 1;
    while (SERCOM0->USART.SYNCBUSY.bit.ENABLE);
}


void disable_sercom0_int() {
  // The baud register is left as Uart::begin() set it. enable_sercom0_int()
  // never changes it, so it is right again as soon as the core clock is back
  // on GCLK0. This used to rewrite it from the arithmetic formula (63019), but
  // the core runs the USART in fractional mode, where that value means about
  // 550 baud: nothing was received until a begin() overwrote it again.

  // Disable USART
  SERCOM0->USART.CTRLA.bit.ENABLE = 0;
  while (SERCOM0->USART.SYNCBUSY.bit.ENABLE);

   // Reconnect SERCOM0 core to 48 MHz (GCLK0)
   GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM0_GCLK_ID_CORE) |
                       GCLK_CLKCTRL_GEN_GCLK0 |
                       GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.bit.SYNCBUSY); // save time
  
  // Seems not required
  //  // Reconnect slow to GCLK0 too
  //  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM0_GCLK_ID_SLOW) |
  //                      GCLK_CLKCTRL_GEN_GCLK0 |
  //                      GCLK_CLKCTRL_CLKEN;
  //  while (GCLK->STATUS.bit.SYNCBUSY);

    // Disable RUNSTDBY now (clear)
    SERCOM0->USART.CTRLA.bit.RUNSTDBY = 0;
    // Start-of-frame detection off again, while the USART is disabled (SFDE is
    // enable-protected). It only matters for waking from standby, and
    // enable_sercom0_int() sets it again before every sleep.
    SERCOM0->USART.CTRLB.bit.SFDE = 0;
    while (SERCOM0->USART.SYNCBUSY.bit.CTRLB);

    // Disable RXSTART interrupt, keep RXC
    // A full register write, not INTENCLR.bit.RXS = 1: reading INTENCLR returns
    // every enabled interrupt, so the bitfield form wrote RXC back as well and
    // switched off the receive interrupt. Measured: INTENSET 0x00 after the
    // wakeup, bytes arriving but none reaching the buffer. The begin() that
    // used to follow switched RXC back on and hid it.
    SERCOM0->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_RXS;
    SERCOM0->USART.INTFLAG.reg = SERCOM_USART_INTFLAG_RXS; // no stale start condition

    // Re-enable USART
    SERCOM0->USART.CTRLA.bit.ENABLE = 1;
    while (SERCOM0->USART.SYNCBUSY.bit.ENABLE);
    // The error flags are cleared, but nothing is read from DATA: the parser
    // syncs on the frame's length byte, and that could be the one dropped.
    SERCOM0->USART.STATUS.reg = SERCOM_USART_STATUS_BUFOVF |
                                SERCOM_USART_STATUS_FERR |
                                SERCOM_USART_STATUS_PERR;
    SERCOM0->USART.INTFLAG.reg = SERCOM_USART_INTFLAG_ERROR;
    NVIC_SetPriority(SERCOM0_IRQn, SERCOM_NVIC_PRIORITY); // save time
}
// https://github.com/adafruit/ArduinoCore-samd/blob/05e83bcb71232684dc259131ac5c5d624db191f5/cores/arduino/SERCOM.cpp#L849C46-L849C66



// GCLK6: OSCULP32K / 4 = 8192 Hz, running in standby. It clocks the RTC, both
// timers (TC3 pulse counter, TC4 time counter), the watchdog and the EIC.
//
// It used to run at 1024 Hz. Every synchronised write or read of those
// peripherals then waited about 5 ms, and those waits made up most of a wake:
// 69 ms per DAVIS wake even after the software had been trimmed. At 8192 Hz a
// wait is about 0.6 ms.
//
// The price is sleep current. Measured on the rig, clocking the RTC or a timer
// faster than 1024 Hz costs about 13.5 uA - the same at 8192 Hz as at
// 32.768 kHz, and the RTC alone as much as all of them - and READREQ.RCONT on
// a single timer at 1024 Hz costs just as much. It is one standing cost either
// way, so the software changes in rtc_sleep_cfg(), elapsed_time_counter() and
// pulse_count_since_arm() come along for free. At this clock rate their gain
// is within the noise of the measurement.
//
// DAVIS mode, 120 s each; time above 1 mA per wake (TX included), sleep floor
// and mean current:
//
//   GCLK6 at 1024 Hz                                   69 ms  83.9 uA  222 uA
//   GCLK6 at 1024 Hz, only the software changes        53 ms  97.4 uA  226 uA
//   RTC, TC3, TC4 on a second generator at 8192 Hz     35 ms  97.7 uA  214 uA
//   GCLK6 at 8192 Hz                                   26 ms  98.1 uA  208 uA
//   GCLK6 at 8192 Hz with the software changes         27 ms  97.8 uA  208 uA
//
// The trade pays while the station wakes more often than about every 24 s. A
// DAVIS cycle is 12 s, and a WS85 wakes for each of its frames. A station that
// sleeps much longer is better off at 1024 Hz without RCONT.
//
// 8192 Hz is the highest rate the watchdog can live with: its period counts
// cycles of this clock and tops out at 16384 of them, which is 2 s.
void configGCLK6(bool en_rtc){

  //Set Clock divider for GCLK6
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) |         //Select clock divisor: 2 ^ (1 + 1) = 4, so 8192 Hz
                     GCLK_GENDIV_ID(6);           //GCLK6
  while (GCLK->STATUS.bit.SYNCBUSY);              //Wait for the settings to be synchronized

  GCLK->GENCTRL.reg = //GCLK_GENCTRL_OE |         // Test: enable GCLK output (on a selected pin)
                      GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                      GCLK_GENCTRL_GENEN |        // Enable GCLK6
                      GCLK_GENCTRL_SRC_OSCULP32K| // Set the internal 32.768kHz clock source (GCLK_GENCTRL_SRC_OSCULP32K)
                      GCLK_GENCTRL_RUNSTDBY |
                      GCLK_GENCTRL_DIVSEL |
                      GCLK_GENCTRL_ID(6);         // Select GCLK6
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization 

if(en_rtc){
// Connect GCLK6 output to RTC
 GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK6 |  // Select GCLK6
                     GCLK_CLKCTRL_ID_RTC |     // Connect to the RTC
                     GCLK_CLKCTRL_CLKEN;       // Enable the channel
 while (GCLK->STATUS.bit.SYNCBUSY);            // Wait for synchronization
}

//Connect GCLK6 output to WDT
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK6 |  // Select GCLK6
                      GCLK_CLKCTRL_ID_WDT |     // Connect to the WDT
                      GCLK_CLKCTRL_CLKEN;       // Enable GCLK6
  while (GCLK->STATUS.bit.SYNCBUSY);            // Wait for synchronization

//Connect GCLK6 output to External Interrupt Controller (EIC)
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_GEN_GCLK6 |  // Select GCLK6
                      GCLK_CLKCTRL_ID_EIC |     // Connect to the EIC
                      GCLK_CLKCTRL_CLKEN;       // Enable GCLK6
  while (GCLK->STATUS.bit.SYNCBUSY);            // Wait for synchronization

// Enable WDT early-warning interrupt
  NVIC_DisableIRQ(WDT_IRQn);
  NVIC_ClearPendingIRQ(WDT_IRQn);
  NVIC_SetPriority(WDT_IRQn, 0); // Top priority
  NVIC_EnableIRQ(WDT_IRQn);
}


// Pulsecounter

uint32_t rtc_sleep_cfg(uint32_t milliseconds){
  uint16_t counts =  milliseconds/8;

  // GCLK6 has to be routed to the RTC only once, nothing turns that routing
  // off again. configGCLK6() rewrites the generator and three clock channels,
  // five synchronisation waits, and this runs before every sleep.
  static bool rtc_clock_ready = false;
  if(!rtc_clock_ready){
    configGCLK6(true);
    rtc_clock_ready = true;
  }
  // Is 16 bit counter only
  if( milliseconds/8 > 0xFFFF){
    counts = 0xFFFF;
  }

  // After the first full configuration the RTC stays enabled in 16-bit counter
  // mode, and the interrupt, NVIC and sleep settings stay in place. Only the
  // period changes, and the count has to start from zero: two synchronised
  // writes instead of disable, software reset, configuration and enable.
  // COUNT goes first, so the counter cannot be past a shorter new period.
  static bool rtc_configured = false;
  static uint16_t running_counts = 0xFFFF;
  if(rtc_configured && RTC->MODE1.CTRL.bit.ENABLE){
    // Already counting this period: nothing to do. The RTC runs free, so its
    // next overflow comes one period after the last one - the sleep window then
    // starts at the wakeup rather than at the end of the wake phase, which keeps
    // the cycle exactly periodic. Saves both synchronised writes (about 11 ms
    // with GCLK6 at 1024 Hz).
    if(running_counts == counts){
      return (counts+1)*8;
    }
    RTC->MODE1.COUNT.reg = 0;
    while (RTC->MODE1.STATUS.bit.SYNCBUSY);
    RTC->MODE1.PER.reg = RTC_MODE1_PER_PER(counts);
    while (RTC->MODE1.STATUS.bit.SYNCBUSY);
    RTC->MODE1.INTFLAG.reg = RTC_MODE1_INTFLAG_OVF; // an overflow of the old period must not end this one
    running_counts = counts;
    return (counts+1)*8;
  }
  // RTC configuration (rtc.h)--------------------------------------------------                                              
    RTC->MODE1.CTRL.bit.ENABLE = 0;                       // Disable the RTC
    while (RTC->MODE1.STATUS.bit.SYNCBUSY);               // Wait for synchronization

    RTC->MODE1.CTRL.bit.SWRST = 1;                       // Software reset the RTC
    while (RTC->MODE1.STATUS.bit.SYNCBUSY);              // Wait for synchronization
    
    RTC->MODE1.CTRL.reg |= RTC_MODE1_CTRL_PRESCALER_DIV64 |    // 8192 Hz / 64 = 128 Hz, the rate the sleep times assume
                           RTC_MODE1_CTRL_MODE_COUNT16;           // Set RTC to mode 1, 16-bit timer                         
    RTC->MODE1.PER.reg = RTC_MODE1_PER_PER(counts);            /// Interrupt time in s: 1Hz/(seconds + 1)
    while (RTC->MODE1.STATUS.bit.SYNCBUSY);                       // Wait for synchronization

  // Configure RTC interrupts ------------------------------------------
    RTC->MODE1.INTENSET.reg = RTC_MODE1_INTENSET_OVF;             // Enable RTC overflow interrupts

    NVIC_SetPriority(RTC_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for RTC
    NVIC_EnableIRQ(RTC_IRQn);         // Connect RTC to Nested Vector Interrupt Controller (NVIC)

  // Enable Deep Sleep Mode--------------------------------------------------------------
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;// | SCB_SCR_SLEEPONEXIT_Msk;  // Put the SAMD21 in deep sleep upon executing the __WFI() function
    NVMCTRL->CTRLB.reg |= NVMCTRL_CTRLB_SLEEPPRM_DISABLED;        // Disable auto power reduction during sleep - SAMD21 Errata 1.14.2


  // get silicon revision
  uint32_t rev = DSU->DID.reg;
  rev &= DSU_DID_REVISION_Msk;
  rev = rev >> DSU_DID_REVISION_Pos;
  #define _SYSTEM_MCU_REVISION_D 3

  if (rev < _SYSTEM_MCU_REVISION_D) {
      /* Errata 13140: Make sure that the Flash does not power all the way down
        * when in sleep mode. https://forum.microchip.com/s/topic/a5C3l000000UfKhEAK/t171994*/
      NVMCTRL->CTRLB.bit.SLEEPPRM = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val;
  }

  // Enable RTC--------------------------------------------------------------
    RTC->MODE1.CTRL.bit.ENABLE = 1;                       // Enable the RTC
    while (RTC->MODE1.STATUS.bit.SYNCBUSY);               // Wait for synchronization

    rtc_configured = true;
    running_counts = counts;
    return (counts+1)*8;
    //return milliseconds;
  }
  

void wdt_reset(){
  // A clear that is still being synchronised resets the counter just the same,
  // so there is nothing to wait for. Waiting cost a full synchronisation - about
  // 4 ms with GCLK6 at 1024 Hz - whenever two clears came close together, and
  // loop() does one at its end and the next one at its start.
  if(WDT->STATUS.bit.SYNCBUSY){
    return;
  }
  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
}

void wdt_disable(){
  // Wait for a clear or an enable still being synchronised before writing CTRL:
  // a write during synchronisation stalls the bus. Waiting here lets interrupts
  // run meanwhile.
  while (WDT->STATUS.bit.SYNCBUSY){};
  WDT->CTRL.bit.ENABLE = 0; // Stop watchdog
  while (WDT->STATUS.bit.SYNCBUSY){};
}


// Cycles per second of the clock the watchdog counts, see configGCLK6().
static const long WDT_CLK_HZ = 8192L;

int wdt_enable(int maxPeriodMS, bool isForSleep) {
    static bool wdt_initialized = false;
  // Enable the watchdog with a period up to the specified max period in
  // milliseconds.

  // Review the watchdog section from the SAMD21 datasheet section 17:
  // http://www.atmel.com/images/atmel-42181-sam-d21_datasheet.pdf

  int cycles;
  uint8_t bits;

  if(!wdt_initialized){
    configGCLK6(false);
    wdt_initialized = true;
  }

  // Same configuration as last time, and the WDT is off: CONFIG, the window
  // mode and the early-warning interrupt are all still set - wdt_disable() only
  // clears ENABLE - so clearing the counter and enabling is all that is left.
  // That saves the disable and the configuration write, two synchronisation
  // waits on every wakeup.
  static int last_period = -1;
  static bool last_for_sleep = false;
  static int last_result = 0;
  if(maxPeriodMS == last_period && isForSleep == last_for_sleep && !WDT->CTRL.bit.ENABLE){
    // One synchronised write, and no waiting for it: the enable synchronises in
    // the background. What stood here before - wdt_reset(), then this CTRL
    // write while the CLEAR was still being synchronised - stalled the
    // peripheral bus with no interrupt served, measured about 740 us with GCLK6
    // at 8192 Hz. The WS85 UART lost 5 to 7 bytes in the middle of every frame
    // to it. Nothing clears the counter now; should it survive wdt_disable(), it
    // holds only the wake phases since the last wdt_reset(), which are short
    // against the period.
    WDT->CTRL.bit.ENABLE = 1; // Start watchdog now!
    return last_result;
  }

  WDT->CTRL.reg = 0; // Disable watchdog for config
  while (WDT->STATUS.bit.SYNCBUSY)
    {};

  // The watchdog counts cycles of GCLK6, which runs at WDT_CLK_HZ from the
  // low-power 32.768 kHz oscillator, so the periods are not precise. 16384
  // cycles is the longest period the hardware offers, 2 s at 8192 Hz: anything
  // longer is clamped to that.
  if ((maxPeriodMS >= (16384L * 1000L) / WDT_CLK_HZ) || !maxPeriodMS) {
    cycles = 16384;
    bits = 0xB;
  } else {
    cycles = (maxPeriodMS * WDT_CLK_HZ + 500) / 1000; // ms -> WDT cycles
    if (cycles >= 8192) {
      cycles = 8192;
      bits = 0xA;
    } else if (cycles >= 4096) {
      cycles = 4096;
      bits = 0x9;
    } else if (cycles >= 2048) {
      cycles = 2048;
      bits = 0x8;
    } else if (cycles >= 1024) {
      cycles = 1024;
      bits = 0x7;
    } else if (cycles >= 512) {
      cycles = 512;
      bits = 0x6;
    } else if (cycles >= 256) {
      cycles = 256;
      bits = 0x5;
    } else if (cycles >= 128) {
      cycles = 128;
      bits = 0x4;
    } else if (cycles >= 64) {
      cycles = 64;
      bits = 0x3;
    } else if (cycles >= 32) {
      cycles = 32;
      bits = 0x2;
    } else if (cycles >= 16) {
      cycles = 16;
      bits = 0x1;
    } else {
      cycles = 8;
      bits = 0x0;
    }
  }

  if (isForSleep) {
    WDT->INTENSET.bit.EW = 1;      // Enable early warning interrupt
    WDT->CONFIG.bit.PER = 0xB;     // Period = max
    WDT->CONFIG.bit.WINDOW = bits; // Set time of interrupt
    WDT->CTRL.bit.WEN = 1;         // Enable window mode
    while (WDT->STATUS.bit.SYNCBUSY){}; // Sync CTRL write
  } else {
    WDT->INTENCLR.bit.EW = 1;   // Disable early warning interrupt
    WDT->CONFIG.bit.PER = bits; // Set period for chip reset
    WDT->CTRL.bit.WEN = 0;      // Disable window mode
    while (WDT->STATUS.bit.SYNCBUSY){}; // Sync CTRL write
  }

  // No wdt_reset() right before the enable: a CLEAR still being synchronised
  // makes the CTRL write stall the bus, see the fast path above.
  WDT->CTRL.bit.ENABLE = 1; // Start watchdog now!
  while (WDT->STATUS.bit.SYNCBUSY){};
  last_period = maxPeriodMS;
  last_for_sleep = isForSleep;
  last_result = (cycles * 1000L + WDT_CLK_HZ / 2) / WDT_CLK_HZ; // WDT cycles -> ms
  return last_result;
}

void deepsleep(bool light) {
  // Enable standby sleep mode (deepest sleep) and activate.
  // Insights from Atmel ASF library.

// Errata: Make sure that the Flash does not power all the way down when in sleep mode. Only until Rev. C
// NVMCTRL->CTRLB.bit.SLEEPPRM = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val; // 3

  if(light){
  // Idele Modes. see P.138 of https://ww1.microchip.com/downloads/en/DeviceDoc/SAM_D21_DA1_Family_DataSheet_DS40001882F.pdf
   PM->SLEEP.reg |= PM_SLEEP_IDLE_CPU;  // Enable Idle0 mode - sleep CPU clock only
   // PM->SLEEP.reg |= PM_SLEEP_IDLE_AHB; // Idle1 - sleep CPU and AHB clocks
   // PM->SLEEP.reg |= PM_SLEEP_IDLE_APB; // Idle2 - sleep CPU, AHB, and APB clocks
   SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
  } else {
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk; // deepsleep
  }
  // Due to a hardware bug on the SAMD21, the SysTick interrupts become
  // active before the flash has powered up from sleep, causing a hard fault.
  // To prevent this the SysTick interrupts are disabled before entering sleep
  // mode.

  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk; // Disable SysTick interrupts
  __DSB(); // Data sync to ensure outgoing memory accesses complete
  __WFI(); // Wait for interrupt (places device in sleep mode)
  // Wakeup from timer or pinInterrupt
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk; // Enable SysTick interrupts
  

  // Code resumes here on wake (WDT early warning interrupt).
  // Bug: the return value assumes the WDT has run its course;
  // incorrect if the device woke due to an external interrupt.
  // Without an external RTC there's no way to provide a correct
  // sleep period in the latter case...but at the very least,
  // might indicate said condition occurred by returning 0 instead
  // (assuming we can pin down which interrupt caused the wake).

// https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-42248-SAM-D20-Power-Measurements_ApplicationNote_AT04188.pdf
}

void setup_PM(bool en_pulse_counter, bool en_rtc_counter){
// We use this one { PORTB,  8, PIO_ANALOG, (PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel2, PWM4_CH0, TC4_CH0, EXTERNAL_INT_8 }, // ADC/AIN[2]
// disable Clock ADC/DAC for Analog
  //PM->APBCMASK.reg &= ~PM_APBCMASK_ADC;
  
  PM->APBCMASK.reg &= ~PM_APBCMASK_DAC;

  // The DMAC, which nothing in this project uses. Both of its clocks have to go,
  // and both live in registers of their own.
  //
  // What used to stand here was "PM->APBCMASK.reg &= ~PM_APBBMASK_DMAC" - an
  // APBB bit written into the APBC register. PM_APBBMASK_DMAC is bit 4, and bit
  // 4 of APBCMASK is SERCOM2, so the line switched off SERCOM2 (which the block
  // below does anyway) and never touched the DMAC at all.
  PM->AHBMASK.reg  &= ~PM_AHBMASK_DMAC;
  PM->APBBMASK.reg &= ~PM_APBBMASK_DMAC;

  // Bd adafruit default all sercoms are enabled, disable the one we dont use
  //PM->APBCMASK.reg &= ~PM_APBCMASK_SERCOM1; // Serial2
  PM->APBCMASK.reg &= ~PM_APBCMASK_SERCOM2;
  PM->APBCMASK.reg &= ~PM_APBCMASK_SERCOM5;

  // Disable Clock TC/TCC for Pulse and Analog
  PM->APBCMASK.reg &= ~PM_APBCMASK_TCC0;
  PM->APBCMASK.reg &= ~PM_APBCMASK_TCC1;
  //PM->APBCMASK.reg &= ~PM_APBCMASK_TCC2;
  //PM->APBCMASK.reg &= ~PM_APBCMASK_TC3;

  if(en_pulse_counter){
    PM->APBCMASK.reg |= PM_APBCMASK_TC3; // used for pulsecounter, 16bit mode
    PM->APBCMASK.reg |= PM_APBCMASK_TCC2; // todo: required?
  } else {
    PM->APBCMASK.reg &= ~PM_APBCMASK_TC3;
  }

  if(en_rtc_counter){
    PM->APBCMASK.reg |= PM_APBCMASK_TC4; // used for rtc counting sleeptime
    PM->APBCMASK.reg |= PM_APBCMASK_TC5; // 32 bit
  } else {
    PM->APBCMASK.reg &= ~PM_APBCMASK_TC4;
    PM->APBCMASK.reg &= ~PM_APBCMASK_TC5;
  }

}



/***************************************************************************
  TC3 External Pulse Counter (16-bit)
  Target MCU : ATSAMD21G18
  Function   : Count falling edges on PA04 (EXTINT4)
               using EIC -> EVSYS -> TC3
               Runs in STANDBY sleep mode
***************************************************************************/

void setup_pulse_counter()
{
  uint32_t ulPin = 17;          // Arduino pin 17 = PA04 = EXTINT4

  // Configure GCLK6 (OSCULP32K / 4 = 8192 Hz) for low-power peripherals
  configGCLK6(true);

  /***********************************************************************
    1) ROUTE GENERIC CLOCK TO TC3
       TC3 shares clock ID with TCC2 (NOT with TC4/5!)
  ***********************************************************************/
  GCLK->CLKCTRL.reg =
      GCLK_CLKCTRL_CLKEN |
      GCLK_CLKCTRL_GEN_GCLK6 |        // Use GCLK6 (8192 Hz)
      GCLK_CLKCTRL_ID_TCC2_TC3;       // Clock ID for TC3
  while (GCLK->STATUS.bit.SYNCBUSY);

  /***********************************************************************
    2) PORT CONFIGURATION
       Configure PA04 as EIC (external interrupt controller) input
  ***********************************************************************/
  EPortType port = g_APinDescription[ulPin].ulPort;
  uint32_t  pin  = g_APinDescription[ulPin].ulPin;
  uint32_t  pinMask = (1ul << pin);

  // Enable peripheral multiplexer + input buffer
  PORT->Group[port].PINCFG[pin].reg =
      PORT_PINCFG_PMUXEN | PORT_PINCFG_INEN;

  // Make pin input
  PORT->Group[port].DIRCLR.reg = pinMask;

  // Connect pin to EIC peripheral (MUX function A)
  if (pin & 1)
      PORT->Group[port].PMUX[pin >> 1].bit.PMUXO = PORT_PMUX_PMUXO_A;
  else
      PORT->Group[port].PMUX[pin >> 1].bit.PMUXE = PORT_PMUX_PMUXE_A;

  /***********************************************************************
    3) EXTERNAL INTERRUPT CONTROLLER (EIC)
       Configure EXTINT4 to:
       - Detect falling edge
       - Generate event (not interrupt)
  ***********************************************************************/
  EExt_Interrupts in = g_APinDescription[ulPin].ulExtInt;

  uint32_t config = (in > 7) ? 1 : 0;
  uint32_t pos    = (in > 7) ? (in - 8) * 4 : in * 4;

  PM->APBAMASK.reg |= PM_APBAMASK_EIC;   // Enable EIC clock

  // Disable EIC before configuration
  EIC->CTRL.bit.ENABLE = 0;
  while (EIC->STATUS.bit.SYNCBUSY);

  // Configure falling edge detection
  EIC->CONFIG[config].reg &= ~(0xF << pos);
  EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_FALL_Val << pos;

  // Enable event output on this EXTINT line
  EIC->EVCTRL.reg |= (1 << in);

  // Disable CPU interrupt (we only want events)
  EIC->INTENCLR.reg = (1 << in);

  // Re-enable EIC
  EIC->CTRL.bit.ENABLE = 1;
  while (EIC->STATUS.bit.SYNCBUSY);

  /***********************************************************************
    4) EVENT SYSTEM
       Route EIC EXTINT4 -> TC3
       Path is asynchronous so it works in STANDBY
  ***********************************************************************/
  PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;

  // Connect event channel 0 to TC3
  EVSYS->USER.reg =
      EVSYS_USER_CHANNEL(1) |             // Channel 0 = n+1
      EVSYS_USER_USER(EVSYS_ID_USER_TC3_EVU);

  // Configure channel 0:
  // Generator = EXTINT4
  // Asynchronous path
  EVSYS->CHANNEL.reg =
      EVSYS_CHANNEL_CHANNEL(0) |
      EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_4) |
      EVSYS_CHANNEL_PATH_ASYNCHRONOUS |
      EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT;

  /***********************************************************************
    5) TIMER COUNTER TC3
       Configure as 16-bit event counter
  ***********************************************************************/
  PM->APBCMASK.reg |= PM_APBCMASK_TC3;

  // Software reset (important!)
  TC3->COUNT16.CTRLA.bit.SWRST = 1;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
  while (TC3->COUNT16.CTRLA.bit.SWRST);

  // Count one increment per incoming event
  TC3->COUNT16.EVCTRL.reg =
      TC_EVCTRL_TCEI |            // Enable event input
      TC_EVCTRL_EVACT_COUNT;      // Increment on event

  // Configure timer:
  // - 16-bit mode
  // - Run in standby
  // - No prescaling
  // - Prescaler sync mode
  TC3->COUNT16.CTRLA.reg =
      TC_CTRLA_MODE_COUNT16 |
      TC_CTRLA_RUNSTDBY |
      TC_CTRLA_PRESCALER_DIV1 |
      TC_CTRLA_PRESCSYNC_PRESC;

  // Enable TC3
  TC3->COUNT16.CTRLA.bit.ENABLE = 1;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

  // Continuous read synchronization
  TC3->COUNT16.READREQ.reg =
      TC_READREQ_RCONT |
      TC_READREQ_ADDR(TC_COUNT16_COUNT_OFFSET);

  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
}

// Counter value at the last arm_pulse_counter(), see pulse_count_since_arm().
static uint16_t pulse_offset = 0;

// Read current 16-bit counter value
uint16_t read_pulse_counter()
{
  // setup_pulse_counter() leaves the counter in continuous read synchronisation
  // (READREQ.RCONT), so COUNT can be read straight away. The read request this
  // used to write cleared RCONT again and cost a synchronisation (about 6 ms
  // with GCLK6 at 1024 Hz).
  return TC3->COUNT16.COUNT.reg;
}

// Pulses since arm_pulse_counter(). Counting from an offset instead of zeroing
// the counter saves another synchronised write. The counter is 16 bit and wraps
// with the subtraction; a window holds far fewer pulses than that.
uint16_t pulse_count_since_arm()
{
  return (uint16_t)(read_pulse_counter() - pulse_offset);
}

// Reset counter to zero
void reset_pulse_counter()
{
  TC3->COUNT16.COUNT.reg = 0;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
}

/***************************************************************************
  Function   : Prepare the pulse counter for the next integration window
***************************************************************************/
// Runs at the start of every DAVIS cycle. Normally the counter is still set up
// from the cycle before, and only its current value has to be taken as the
// start of the window.
// setup_pulse_counter() rewrites GCLK6, disables and re-enables the EIC and
// software resets TC3 - eleven synchronisation waits. It still runs on the
// first cycle, and whenever something undid the configuration: a pinMode(),
// pinDisable() or attachInterrupt() on PA04 (the analog test mode does that),
// or the sleep quiet mask. The checks are plain register reads.
void arm_pulse_counter()
{
  static bool configured = false;
  uint32_t ulPin = 17;          // Arduino pin 17 = PA04 = EXTINT4
  EPortType port = g_APinDescription[ulPin].ulPort;
  uint32_t  pin  = g_APinDescription[ulPin].ulPin;
  EExt_Interrupts in = g_APinDescription[ulPin].ulExtInt;
  uint32_t config = (in > 7) ? 1 : 0;
  uint32_t pos    = (in > 7) ? (in - 8) * 4 : in * 4;

  const uint8_t pmux = (pin & 1) ? PORT->Group[port].PMUX[pin >> 1].bit.PMUXO
                                 : PORT->Group[port].PMUX[pin >> 1].bit.PMUXE;
  const bool pin_ready = PORT->Group[port].PINCFG[pin].bit.PMUXEN &&
                         pmux == PORT_PMUX_PMUXE_A_Val;
  const bool eic_ready = EIC->CTRL.bit.ENABLE &&
                         (EIC->EVCTRL.reg & (1ul << in)) &&
                         ((EIC->CONFIG[config].reg >> pos) & 0xF) == EIC_CONFIG_SENSE0_FALL_Val;
  const bool tc_ready  = TC3->COUNT16.CTRLA.bit.ENABLE &&
                         (TC3->COUNT16.EVCTRL.reg & TC_EVCTRL_TCEI);

  if(configured && pin_ready && eic_ready && tc_ready){
    pulse_offset = read_pulse_counter();
    return;
  }
  setup_pulse_counter();
  pulse_offset = read_pulse_counter();
  configured = true;
}

// Stop counter
void stop_pulse_counter()
{
  TC3->COUNT16.CTRLBSET.reg = TC_CTRLBSET_CMD_STOP;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
}

 // https://forum.arduino.cc/t/samd21-not-waking-up-using-timer-interrupt-from-deep-sleep/662100/2

// https://github.com/adafruit/ArduinoCore-samd/blob/21ac7624c9d7374878d2c2029caad203ed34326d/cores/arduino/USB/SAMR21_USBDevice.h#L50
//
//  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE; //Disable USB port (to disconnect correctly from host
//  USB->DEVICE.CTRLA.bit.ENABLE = 0; // Disable the USB peripheral
//  USB->DEVICE.CTRLA.bit.RUNSTDBY = 0; // Deactivate run on standby

//SYSCTRL->VREG.bit.RUNSTDBY = 1;  // Keep the voltage regulator in normal configuration during run stanby




// AHBMASK:  CLK_HPBA_AHB CLK_HPBB_AHB CLK_HPBC_AHB CLK_DSU_AHB CLK_NVMCTRL_AHB CLK_DMAC_AHB CLK_USB_AHB
// APBAMASK:  CLK_PAC0_APB CLK_PM_APB CLK_SYSCTRL_APB CLK_GCLK_APB CLK_WDT_APB CLK_RTC_APB CLK_EIC_APB
// APBBMASK:  CLK_PAC1_APB CLK_DSU_APB CLK_NVMCTRL_APB CLK_PORT_APB CLK_DMAC_APB CLK_USB_APB
// APBCMASK:  CLK_SERCOM0_APB CLK_SERCOM1_APB CLK_SERCOM2_APB CLK_SERCOM3_APB CLK_SERCOM4_APB CLK_SERCOM5_APB CLK_TCC0_APB CLK_TCC1_APB CLK_TCC2_APB CLK_TC3_APB CLK_TC4_APB CLK_TC5_APB CLK_ADC_APB CLK_DAC_APB

//PM->AHBMASK.reg &= ~PM_AHBMASK_USB; // this kills I2C after sleep
//PM->APBCMASK.reg &= ~PM_APBBMASK_USB; // this kills I2C

//PM->AHBMASK.reg &= ~PM_AHBMASK_DMAC;
//PM->APBCMASK.reg &= ~PM_APBBMASK_DMAC;
//PM->AHBMASK.reg &= ~PM_AHBMASK_DSU;
//PM->APBCMASK.reg &= ~PM_APBBMASK_DSU;

// point all unused peripherals to dead clocks like so: // https://forum.arduino.cc/t/reducing-power-consumption/412345/18
// GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_USB | GCLK_CLKCTRL_GEN_GCLK4;
// GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_EVSYS_0 | GCLK_CLKCTRL_GEN_GCLK4;


void setup_rtc_time_counter(){
  // Ensure GCLK6 is initialized before routing it to TC4/TC5. Without this,
  // startup can stall here on cold boot when WDT setup has been skipped.
  configGCLK6(false);
  // https://forum.arduino.cc/t/arduino-zero-sam-d21-hardware-counter-as-simple-input-counter-intialization/630306/2
  // Generic Clock /////////////////////////////////////////////////////////////////////////
 
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |        // Enable the generic clock...
                      GCLK_CLKCTRL_GEN_GCLK6 |    // On GCLK6 at 8192 Hz
                      GCLK_CLKCTRL_ID_TC4_TC5;    // Route GCLK6 to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
                              
  // Timer Counter TC4 /////////////////////////////////////////////////////////////////////

  PM->APBCMASK.reg |= PM_APBCMASK_TC4; // PM enable TC4 & TC5 for 32 bit counter
  PM->APBCMASK.reg |= PM_APBCMASK_TC5;

 
  //TC4->COUNT32.EVCTRL.reg |= TC_EVCTRL_TCEI |              // Enable asynchronous events on the TC timer
  //                           TC_EVCTRL_EVACT_COUNT;        // Increment the TC timer each time an event is received

  TC4->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32 |         // Configure TC4 (if 32: together with TC5 to operate in 32-bit mode)
                           TC_CTRLA_RUNSTDBY |             // Set timer to run in standby
                           TC_CTRLA_PRESCALER_DIV8 |       // 8192 Hz / 8 = 1024 Hz, the rate callers expect (~1 tick per ms)
                           TC_CTRLA_PRESCSYNC_GCLK;        // Reload or reset the counter on next generic clock 
                      
  TC4->COUNT32.CTRLA.bit.ENABLE = 1;                       // Enable TC4
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);                // Wait for synchronization

  TC4->COUNT32.READREQ.reg = TC_READREQ_RCONT |            // Enable a continuous read request
                             TC_READREQ_ADDR(0x10);        // Offset of the 32-bit COUNT register
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);                // Wait for synchronization

}

uint32_t read_time_counter(){
  // ADDR has to name COUNT: writing RREQ alone also sets ADDR to 0, and in the
  // TC only COUNT and CCx can be read-synchronised.
  TC4->COUNT32.READREQ.reg = TC_READREQ_RREQ | TC_READREQ_ADDR(TC_COUNT32_COUNT_OFFSET);         // Request a read synchronization
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);           // Wait for read synchronization
  return TC4->COUNT32.COUNT.reg;
}

// Ticks since the last call. The counter free-runs and is never reset: keeping
// the previous value in software saves the synchronised retrigger before every
// sleep (about 5 ms with GCLK6 at 1024 Hz). The subtraction wraps with the
// counter.
uint32_t elapsed_time_counter(){
  static uint32_t last = 0;
  const uint32_t now = read_time_counter();
  const uint32_t delta = now - last;
  last = now;
  return delta;
}

void reset_time_counter(){
  // RETRIGGER restarts an up-counter from zero on its own. The COUNT = 0 write
  // that came before it was a second synchronisation wait for the same result.
  TC4->COUNT32.CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;  // Retrigger the TC4 timer
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);               // Wait for synchronization
}



// The debug UART in standby ---------------------------------------------------
//
// The point of UART_WAKE: the console stays reachable while the station sleeps.
// Without it the console is only usable for about two seconds after a reset -
// the SERCOM clocks stop in standby, so anything arriving while the station is
// asleep is lost outright. Measured on the rig: of sixty requests sent one per
// second, two were answered, and not one of the other fifty-eight ever produced
// a late reply.
//
// Nothing is switched on going to sleep or on waking, and that is the design.
//
// The first attempt did switch: it moved the core clock to a low-power
// generator before the sleep and back afterwards, which meant disabling the
// USART at both ends. That window is where the bytes went. The SAMD21 SERCOM
// has no receive FIFO - AT11626 lists "single transmit buffer and double
// receive buffers", so two bytes, 174 us at 115200 - and a receiver that is
// switched off buffers nothing at all. Measured: four bytes lost on every
// wake, regardless of how much padding the host sent ahead of the command.
//
// So the peripheral is configured once and then left alone. The start bit
// wakes the CPU through SFDE while the SERCOM, still clocked, samples that very
// frame; RXC then fires and Serial2's handler puts the byte into the 350 byte
// ring buffer, exactly as it does while awake. Reception no longer depends on
// what the main code happens to be doing at that moment.
//
// The core clock has to be OSC8M for this. AT11626 section 4.5: "In standby
// sleep mode, the internal fast start-up oscillator must be selected as the
// GCLK_SERCOMx_CORE source." A 32 kHz generator cannot carry 115200 at all -
// 16x oversampling would need 1.84 MHz - so the low-power generator was only
// ever good for being woken, never for receiving.
//
// Cost, measured in the field state: about +0.10 mA on a 0.77 mA baseline.
// AT11491 puts a UART in standby at 28.6 uA and the rest is OSC8M. DEBUG is
// normally on in the field, so this is a real running cost rather than a
// lab-only one, which is why it stays behind UART_WAKE and off by default.
void configure_debug_uart_standby(bool enable){
  if(!enable){
    // Nothing here wants the 8 MHz oscillator, so let it go - see
    // release_gclk3_osc8m(). This is the default path: UART_WAKE is off.
    release_gclk3_osc8m();
    return;
  }

  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM1;

  // The USART is stopped before the generator is touched, not after: the
  // SYNCBUSY spin below needs the core clock it is currently running on, and
  // taking that away first would hang here.
  SERCOM1->USART.CTRLA.bit.ENABLE = 0;
  while (SERCOM1->USART.SYNCBUSY.reg != 0);

  ensure_gclk2_ulp();
  ensure_gclk3_osc8m_stdby();

  // The fast start-up oscillator, permanently - awake and asleep alike.
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM1_GCLK_ID_CORE) |
                      GCLK_CLKCTRL_GEN_GCLK3 |
                      GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY);

  // The slow clock is one single channel for every SERCOM on this chip -
  // SERCOM0_GCLK_ID_SLOW and SERCOM1_GCLK_ID_SLOW are both 19 - so this moves
  // SERCOM0's as well. Both want GCLK2, so it makes no difference.
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM1_GCLK_ID_SLOW) |
                      GCLK_CLKCTRL_GEN_GCLK2 |
                      GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY);

  SERCOM1->USART.CTRLA.bit.RUNSTDBY = 1;
  while (SERCOM1->USART.SYNCBUSY.reg != 0);

  SERCOM1->USART.BAUD.reg = standby_baud_reg();

  SERCOM1->USART.CTRLB.reg = SERCOM_USART_CTRLB_CHSIZE(0x0) |
                             SERCOM_USART_CTRLB_TXEN |
                             SERCOM_USART_CTRLB_RXEN |
                             SERCOM_USART_CTRLB_SFDE;
  while (SERCOM1->USART.SYNCBUSY.bit.CTRLB);

  // Clear whatever the reconfiguration left standing before arming anything:
  // __WFI returns at once while an enabled interrupt is pending, and a stale
  // flag would turn the very first sleep into a spin. RXC does not clear by
  // writing INTFLAG, it goes away by reading DATA.
  (void)SERCOM1->USART.DATA.reg;
  SERCOM1->USART.STATUS.reg = SERCOM_USART_STATUS_BUFOVF |
                              SERCOM_USART_STATUS_FERR |
                              SERCOM_USART_STATUS_PERR;
  SERCOM1->USART.INTFLAG.reg = SERCOM_USART_INTFLAG_ERROR |
                               SERCOM_USART_INTFLAG_RXS |
                               SERCOM_USART_INTFLAG_TXC;

  // RXS wakes the CPU, RXC delivers the byte, ERROR is the one the Arduino
  // core owns and its handler clears.
  SERCOM1->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXS |
                                SERCOM_USART_INTENSET_RXC |
                                SERCOM_USART_INTENSET_ERROR;

  NVIC_ClearPendingIRQ(SERCOM1_IRQn);
  NVIC_SetPriority(SERCOM1_IRQn, SERCOM_NVIC_PRIORITY);
  NVIC_EnableIRQ(SERCOM1_IRQn);

  SERCOM1->USART.CTRLA.bit.ENABLE = 1;
  while (SERCOM1->USART.SYNCBUSY.bit.ENABLE);
}
