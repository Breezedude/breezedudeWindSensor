#include "sleep.h"
#include "wiring_private.h"




void WDT_Handler(void) {
  // ISR for watchdog early warning, DO NOT RENAME!
  WDT->CTRL.bit.ENABLE = 0; // Disable watchdog
  while (WDT->STATUS.bit.SYNCBUSY)
    ; // Sync CTRL write
  WDT->INTFLAG.bit.EW = 1; // Clear interrupt flag
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

// add void SERCOM0_Handler() __attribute__((weak)); in variant.cpp to overwrite 
void SERCOM0_Handler(void) {
    if (SERCOM0->USART.INTFLAG.bit.RXS) {
        SERCOM0->USART.INTFLAG.reg = SERCOM_USART_INTFLAG_RXS; // clear start bit flag
        // woke up
    }
    Serial1.IrqHandler();  // let Arduino core handle RXC
}



void SERCOM1_Handler(){
  Serial2.IrqHandler();
}


void enable_sercom0_int(void) {
    // 1) Make sure SERCOM0 APBC clock is enabled
    PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;

    // 2) Stop USART while we reconfigure
    SERCOM0->USART.CTRLA.bit.ENABLE = 0;
    while (SERCOM0->USART.SYNCBUSY.reg != 0); // wait any sync

    // 3) Configure GCLK2 as OSCULP32K and RUNSTDBY (generator setup)
    //    (assumes nothing else uses GCLK2 — adapt if it does)
    GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(0); // DIV=0 => no divide
    while (GCLK->STATUS.bit.SYNCBUSY);

    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) |
                        GCLK_GENCTRL_SRC_OSCULP32K |
                        GCLK_GENCTRL_RUNSTDBY |
                        GCLK_GENCTRL_GENEN |
                        GCLK_GENCTRL_IDC |
                        GCLK_GENCTRL_DIVSEL; // DIVSEL not necessary with DIV 0 but OK
    while (GCLK->STATUS.bit.SYNCBUSY);

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


void disable_sercom0_int(uint32_t baud) {
    // Disable USART
    SERCOM0->USART.CTRLA.bit.ENABLE = 0;
    while (SERCOM0->USART.SYNCBUSY.bit.ENABLE);

    // Reconnect SERCOM0 core to 48 MHz (GCLK0)
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM0_GCLK_ID_CORE) |
                        GCLK_CLKCTRL_GEN_GCLK0 |
                        GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.bit.SYNCBUSY);

    // Reconnect slow to GCLK0 too
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM0_GCLK_ID_SLOW) |
                        GCLK_CLKCTRL_GEN_GCLK0 |
                        GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.bit.SYNCBUSY);

    // Reprogram baud for 48 MHz and 115200
    SERCOM0->USART.BAUD.reg = (uint16_t)(65536.0f * (1.0f - (16.0f * (float)baud / 48000000.0f)));

    // Disable RUNSTDBY now (clear)
    SERCOM0->USART.CTRLA.bit.RUNSTDBY = 0;

    // Disable RXSTART interrupt, keep RXC
    SERCOM0->USART.INTENCLR.bit.RXS = 1;

    // Re-enable USART
    SERCOM0->USART.CTRLA.bit.ENABLE = 1;
    while (SERCOM0->USART.SYNCBUSY.bit.ENABLE);

    NVIC_SetPriority(SERCOM0_IRQn, SERCOM_NVIC_PRIORITY);
}
// https://github.com/adafruit/ArduinoCore-samd/blob/05e83bcb71232684dc259131ac5c5d624db191f5/cores/arduino/SERCOM.cpp#L849C46-L849C66



// Config GCLK6 for WDT and External Interrup to run at 1024Hz
void configGCLK6(bool en_rtc){

  //Set Clock divider for GCLK6
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(4) |         //Select clock divisor to divide by 32 = (2 ^ (4 + 1)) , ~1024 Hz clock
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
                     GCLK_CLKCTRL_CLKEN;       // Enable GCLK6
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

  configGCLK6(true);
  // Is 16 bit counter only
  if( milliseconds/8 > 0xFFFF){
    counts = 0xFFFF;
  }

  // RTC configuration (rtc.h)--------------------------------------------------                                              
    RTC->MODE1.CTRL.bit.ENABLE = 0;                       // Disable the RTC
    while (RTC->MODE1.STATUS.bit.SYNCBUSY);               // Wait for synchronization

    RTC->MODE1.CTRL.bit.SWRST = 1;                       // Software reset the RTC
    while (RTC->MODE1.STATUS.bit.SYNCBUSY);              // Wait for synchronization
    
    RTC->MODE1.CTRL.reg |= RTC_MODE1_CTRL_PRESCALER_DIV8 |     // Set prescaler to 1024
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

    return (counts+1)*8;
    //return milliseconds;
  }



void attachInterruptWakeup(uint32_t pin, voidFuncPtr callback, uint32_t mode, bool en_rtc) {
	EExt_Interrupts in = g_APinDescription[pin].ulExtInt;
	if (in == NOT_AN_INTERRUPT || in == EXTERNAL_INT_NMI){
    return;
  }
	attachInterrupt(pin, callback, mode);
	configGCLK6(en_rtc);
	// Enable wakeup capability on pin in case being used during sleep
	EIC->WAKEUP.reg |= (1 << in);
}

void wdt_reset(){
  while (WDT->STATUS.bit.SYNCBUSY){};
  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
}

void wdt_disable(){
  WDT->CTRL.bit.ENABLE = 0; // Stop watchdog
  while (WDT->STATUS.bit.SYNCBUSY){};
}


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

  WDT->CTRL.reg = 0; // Disable watchdog for config
  while (WDT->STATUS.bit.SYNCBUSY)
    {};

  // You'll see some occasional conversion here compensating between
  // milliseconds (1000 Hz) and WDT clock cycles (~1024 Hz).  The low-
  // power oscillator used by the WDT ostensibly runs at 32,768 Hz with
  // a 1:32 prescale, thus 1024 Hz, though probably not super precise.

  if ((maxPeriodMS >= 16000) || !maxPeriodMS) {
    cycles = 16384;
    bits = 0xB;
  } else {
    cycles = (maxPeriodMS * 1024L + 500) / 1000; // ms -> WDT cycles
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

  // Clear watchdog interval
  wdt_reset();             
  WDT->CTRL.bit.ENABLE = 1; // Start watchdog now!
  while (WDT->STATUS.bit.SYNCBUSY){};
  return (cycles * 1000L + 512) / 1024; // WDT cycles -> ms
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

void PM_sleep(){
  PM->APBCMASK.reg &= ~PM_APBCMASK_ADC;
  PM->APBCMASK.reg &= ~PM_APBBMASK_DMAC;
  PM->APBCMASK.reg &= ~PM_APBBMASK_DSU;
  PM->APBCMASK.reg &= ~PM_APBBMASK_USB; // this kills I2C. No

  PM->APBCMASK.reg &= ~PM_APBBMASK_PAC1;
  PM->APBCMASK.reg &= ~PM_APBBMASK_HMATRIX;

  PM->AHBMASK.reg  &= ~PM_AHBMASK_DMAC;
  PM->AHBMASK.reg  &= ~PM_AHBMASK_DSU;
  PM->AHBMASK.reg  &= ~PM_AHBMASK_USB; // this kills I2C after sleep

  PM->AHBMASK.reg  &= ~PM_APBCMASK_I2S;
  PM->AHBMASK.reg  &= ~PM_APBCMASK_PTC;
  PM->AHBMASK.reg  &= ~PM_APBCMASK_DAC;
  PM->AHBMASK.reg  &= ~PM_APBCMASK_AC;
  PM->AHBMASK.reg  &= ~PM_APBCMASK_ADC;

  PM->APBCMASK.reg &= ~PM_APBCMASK_SERCOM0;
  PM->APBCMASK.reg &= ~PM_APBCMASK_SERCOM1;
  PM->APBCMASK.reg &= ~PM_APBCMASK_SERCOM2;
  PM->APBCMASK.reg &= ~PM_APBCMASK_SERCOM3;
  PM->APBCMASK.reg &= ~PM_APBCMASK_SERCOM4;
  PM->APBCMASK.reg &= ~PM_APBCMASK_SERCOM5;

  PM->APBCMASK.reg &= ~PM_APBCMASK_TCC0;
  PM->APBCMASK.reg &= ~PM_APBCMASK_TCC1;
  PM->APBCMASK.reg &= ~PM_APBCMASK_TCC2;
  PM->APBCMASK.reg &= ~PM_APBCMASK_TC3;
  
  PM->APBCMASK.reg &= ~PM_APBCMASK_TC5;
  PM->APBCMASK.reg &= ~PM_APBCMASK_TC6;
  PM->APBCMASK.reg &= ~PM_APBCMASK_TC7;


  USB->DEVICE.CTRLA.bit.ENABLE = 0;
  USB->DEVICE.CTRLA.bit.RUNSTDBY = 0;
  while(USB->DEVICE.SYNCBUSY.bit.ENABLE){};
}

void PM_wakeup(){
  PM->APBCMASK.reg |= PM_APBCMASK_ADC;

  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0; // Serial1
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM1; // Serial2
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM3;
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM4;
}

void setup_PM(bool en_counter){
// We use this one { PORTB,  8, PIO_ANALOG, (PIN_ATTR_PWM|PIN_ATTR_TIMER), ADC_Channel2, PWM4_CH0, TC4_CH0, EXTERNAL_INT_8 }, // ADC/AIN[2]
// disable Clock ADC/DAC for Analog
  //PM->APBCMASK.reg &= ~PM_APBCMASK_ADC;
  
  PM->APBCMASK.reg &= ~PM_APBCMASK_DAC;
  PM->APBCMASK.reg &= ~PM_APBBMASK_DMAC;

  // Bd adafruit default all sercoms are enabled, disable the one we dont use
  //PM->APBCMASK.reg &= ~PM_APBCMASK_SERCOM1; // Serial2
  PM->APBCMASK.reg &= ~PM_APBCMASK_SERCOM2;
  PM->APBCMASK.reg &= ~PM_APBCMASK_SERCOM5;

  // Disable Clock TC/TCC for Pulse and Analog
  PM->APBCMASK.reg &= ~PM_APBCMASK_TCC0;
  PM->APBCMASK.reg &= ~PM_APBCMASK_TCC1;
  PM->APBCMASK.reg &= ~PM_APBCMASK_TCC2;
  PM->APBCMASK.reg &= ~PM_APBCMASK_TC3;

  //PM->APBCMASK.reg &= ~PM_APBCMASK_TC4; // used for rtc & pulsecounter, keep it running
  if(!en_counter){
    PM->APBCMASK.reg &= ~PM_APBCMASK_TC5; //only used if 32bit rtc counter is used (ws80), not for 16bit davis6410 pulse counter
  }
  PM->APBCMASK.reg &= ~PM_APBCMASK_TC6;
  PM->APBCMASK.reg &= ~PM_APBCMASK_TC7;
}



void setup_pulse_counter(){
  uint32_t ulPin = 17; // PA04 PIN_DAVIS_SPEED, Uses Interrupt 4
  configGCLK6(true);
  // https://forum.arduino.cc/t/arduino-zero-sam-d21-hardware-counter-as-simple-input-counter-intialization/630306/2
  // Generic Clock /////////////////////////////////////////////////////////////////////////
 
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |        // Enable the generic clock...
                      GCLK_CLKCTRL_GEN_GCLK6 |    // On GCLK6 at 1024Hz
                      GCLK_CLKCTRL_ID_TC4_TC5;    // Route GCLK6 to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Port Configuration ///////////////////////////////////////////////////////////////////

//pinPeripheral(ulPin, PIO_EXTINT); 

  // Enable the port multiplexer on digital pin PIN_DAVIS_SPEED (port pin PA04)
  EPortType port = g_APinDescription[ulPin].ulPort;
  uint32_t pin = g_APinDescription[ulPin].ulPin;
  uint32_t pinMask = (1ul << pin);
  // Set pin to reset value
  PORT->Group[port].PINCFG[pin].reg = PORT_PINCFG_PMUXEN | PORT_PINCFG_INEN ; // | PORT_PINCFG_DRVSTR;
  PORT->Group[port].DIRCLR.reg = pinMask;

  // Set-up the pin as an EIC (interrupt) peripheral on PIN_DAVIS_SPEED
  if ( g_APinDescription[ulPin].ulPin & 1 ){  // is pin odd?
     PORT->Group[port].PMUX[pin >> 1].reg |= PORT_PMUX_PMUXO_A;
  } else { // even
       PORT->Group[port].PMUX[pin >> 1].reg |= PORT_PMUX_PMUXE_A;
  }

// External Interrupt Controller (EIC) ///////////////////////////////////////////////////

EExt_Interrupts in = g_APinDescription[ulPin].ulExtInt;

// Look for right CONFIG register to be addressed
uint32_t config;
uint32_t pos;
if (in > EXTERNAL_INT_7) {
    config = 1;
    pos = (in - 8) << 2;
  } else {
    config = 0;
    pos = in << 2;
  }

  EIC->WAKEUP.reg &= ~(1 << in); // Disable wakeup capability on pin in case being used during sleep
  EIC->CONFIG[config].reg &=~ (EIC_CONFIG_SENSE0_Msk << pos);
  EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_FALL_Val << pos;
  // EIC->CONFIG[0].reg |= EIC_CONFIG_SENSE4_FALL;                        // Set event detecting a FALL level on interrupt 

  EIC->EVCTRL.reg |= (1 << in);                                           // Enable event output on external interrupt

  //EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT4;                             // Disable interrupts on interrupt 4
  EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT(1 << in);

  EIC->CTRL.bit.ENABLE = 1;                                               // Enable the EIC peripheral
  while (EIC->STATUS.bit.SYNCBUSY);                                       // Wait for synchronization

  // Event System //////////////////////////////////////////////////////////////////////////

  PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;                                  // Switch on the event system peripheral

  EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) |                               // Attach the event user (receiver) to channel 0 (n + 1)
                    EVSYS_USER_USER(EVSYS_ID_USER_TC4_EVU);               // Set the event user (receiver) as timer TC4
  
  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |               // No event edge detection
                       EVSYS_CHANNEL_PATH_ASYNCHRONOUS |                  // Set event path as asynchronous
                       EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_4) |   // Set event generator (sender) as external interrupt 4
                       EVSYS_CHANNEL_CHANNEL(0);                          // Attach the generator (sender) to channel 0                                 
  
  // Timer Counter TC4 /////////////////////////////////////////////////////////////////////

  PM->APBCMASK.reg |= PM_APBCMASK_TC4; // PM enable TC4 & TC5
  //PM->APBCMASK.reg |= PM_APBCMASK_TC5; // saves 10uA

 
  TC4->COUNT16.EVCTRL.reg |= TC_EVCTRL_TCEI |              // Enable asynchronous events on the TC timer
                             TC_EVCTRL_EVACT_COUNT;        // Increment the TC timer each time an event is received

  TC4->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 |         // Configure TC4 (if 32: together with TC5 to operate in 32-bit mode)
                           TC_CTRLA_RUNSTDBY |             // Set timer to run in standby
                           TC_CTRLA_PRESCALER_DIV1 |       // Prescaler: GCLK_TC/1
                           TC_CTRLA_PRESCSYNC_GCLK;        // Reload or reset the counter on next generic clock 
                      
  TC4->COUNT16.CTRLA.bit.ENABLE = 1;                       // Enable TC4
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);                // Wait for synchronization

  TC4->COUNT16.READREQ.reg = TC_READREQ_RCONT |            // Enable a continuous read request
                             TC_READREQ_ADDR(0x10);        // Offset of the 32-bit COUNT register
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);                // Wait for synchronization

}

uint32_t read_pulse_counter(){
  TC4->COUNT16.READREQ.reg = TC_READREQ_RREQ;         // Request a read synchronization
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);           // Wait for read synchronization
  return TC4->COUNT16.COUNT.reg;
}

void reset_pulse_counter(){
  TC4->COUNT16.COUNT.reg = 0x0000;                        // Output the result
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);               // Wait for read synchronization
  TC4->COUNT16.CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;  // Retrigger the TC4 timer
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);               // Wait for synchronization
}

void stop_pulse_counter(){
  TC4->COUNT16.CTRLBSET.reg = TC_CTRLBSET_CMD_STOP;    // Stop the TC4 timer
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);            // Wait for synchronization
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
  //configGCLK6(true);
  // https://forum.arduino.cc/t/arduino-zero-sam-d21-hardware-counter-as-simple-input-counter-intialization/630306/2
  // Generic Clock /////////////////////////////////////////////////////////////////////////
 
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |        // Enable the generic clock...
                      GCLK_CLKCTRL_GEN_GCLK6 |    // On GCLK6 at 1024Hz
                      GCLK_CLKCTRL_ID_TC4_TC5;    // Route GCLK6 to TC4 and TC5
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
                              
  // Timer Counter TC4 /////////////////////////////////////////////////////////////////////

  PM->APBCMASK.reg |= PM_APBCMASK_TC4; // PM enable TC4 & TC5 for 32 bit counter
  PM->APBCMASK.reg |= PM_APBCMASK_TC5;

 
  //TC4->COUNT32.EVCTRL.reg |= TC_EVCTRL_TCEI |              // Enable asynchronous events on the TC timer
  //                           TC_EVCTRL_EVACT_COUNT;        // Increment the TC timer each time an event is received

  TC4->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32 |         // Configure TC4 (if 32: together with TC5 to operate in 32-bit mode)
                           TC_CTRLA_RUNSTDBY |             // Set timer to run in standby
                           TC_CTRLA_PRESCALER_DIV1 |       // Prescaler: GCLK_TC/1
                           TC_CTRLA_PRESCSYNC_GCLK;        // Reload or reset the counter on next generic clock 
                      
  TC4->COUNT32.CTRLA.bit.ENABLE = 1;                       // Enable TC4
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);                // Wait for synchronization

  TC4->COUNT32.READREQ.reg = TC_READREQ_RCONT |            // Enable a continuous read request
                             TC_READREQ_ADDR(0x10);        // Offset of the 32-bit COUNT register
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);                // Wait for synchronization

}

uint32_t read_time_counter(){
  TC4->COUNT32.READREQ.reg = TC_READREQ_RREQ;         // Request a read synchronization
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);           // Wait for read synchronization
  return TC4->COUNT32.COUNT.reg;
}

void reset_time_counter(){
  TC4->COUNT32.COUNT.reg = 0x0000;                        // Output the result
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);               // Wait for read synchronization
  TC4->COUNT32.CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;  // Retrigger the TC4 timer
  while (TC4->COUNT32.STATUS.bit.SYNCBUSY);               // Wait for synchronization
}