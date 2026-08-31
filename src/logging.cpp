#include "logging.h"
#include "tools.h"

// Note on flushing: the per-call DEBUGSER.flush() that used to sit in every
// helper below has been removed. Uart::flush() busy-waits until the TX ring
// buffer has been shifted out (~87 us per character at 115200), so a single
// log line blocked the CPU for over a millisecond and log_v_hex_dump() did it
// once per byte. The UART only has to be drained before the MCU sleeps (SERCOM
// clocks stop in standby) and before a reset - log_flush() still does that, and
// the sleep and reset paths call it.
//
// usb_flush() stays where it is: Serial.flush() + yield() is what runs
// tud_task() and actually pushes CDC data out, and without it the 256 byte CDC
// FIFO can overflow inside a single loop() iteration.

bool debug_en = false;
bool errors_en = false;

// Serial.flush() alone only calls tud_cdc_write_flush(), which marks data as
// ready but does not run the TinyUSB task.  yield() drives tud_task() so the
// USB IN transfer is actually scheduled and completed without waiting for loop().
static void usb_flush(){
  Serial.flush();
  yield();
}

static void log_info_msg(const char * msg){
  if(debug_en){
    DEBUGSER.print(msg);
    if(display_present()) {display_add_line(String(msg));}
  }
  if(usb_connected){
    Serial.print(msg);
    usb_flush();
  }
}

template <typename T>
static void log_info_msg_value(const char * msg, T num){
  if(debug_en){
    DEBUGSER.print(msg);
    DEBUGSER.println(num);
    if(display_present()) {display_add_line(String(msg) + String(num));}
  }
  if(usb_connected){
    Serial.print(msg);
    Serial.println(num);
    usb_flush();
  }
}

template <typename T>
static void log_info_write_value(T num){
  if(debug_en){
    DEBUGSER.print(num);
  }
  if(usb_connected){
    Serial.print(num);
    usb_flush();
  }
}

static void write_hex_to_stream(Stream &stream, uint32_t num, uint8_t width){
  char buf[9];
  uint8_t digits = 0;
  do {
    uint8_t nibble = num & 0x0F;
    buf[digits++] = (nibble < 10) ? ('0' + nibble) : ('A' + nibble - 10);
    num >>= 4;
  } while (num && digits < sizeof(buf));

  while (digits < width && digits < sizeof(buf)) {
    buf[digits++] = '0';
  }

  while (digits > 0) {
    stream.print(buf[--digits]);
  }
}

static void log_info_write_hex(uint32_t num, uint8_t width){
  if(debug_en){
    write_hex_to_stream(DEBUGSER, num, width);
  }
  if(usb_connected){
    write_hex_to_stream(Serial, num, width);
    usb_flush();
  }
}

static void log_verbose_msg(const char * msg){
  if(debug_en){
    DEBUGSER.print(msg);
    if(display_present()) {display_add_line(String(msg));}
  }
  if(settings.verbose_usb && usb_connected){
    Serial.print(msg);
    usb_flush();
  }
}

template <typename T>
static void log_verbose_msg_value(const char * msg, T num){
  if(debug_en){
    DEBUGSER.print(msg);
    DEBUGSER.println(num);
    if(display_present()) {display_add_line(String(msg));}
  }
  if(settings.verbose_usb && usb_connected){
    Serial.print(msg);
    Serial.println(num);
    usb_flush();
  }
}

void log_i(const char * msg){
  log_info_msg(msg);
}

void log_i(const char * msg, uint32_t num){
  log_info_msg_value(msg, num);
}

void log_i(const char * msg, int32_t num){
  log_info_msg_value(msg, num);
}

void log_i(const char * msg, int num){
  log_info_msg_value(msg, num);
}

void log_i(const char * msg, float num){
  log_info_msg_value(msg, num);
}

void log_v(const char * msg){
  log_verbose_msg(msg);
}

void log_v(const char * msg, uint32_t num){
  log_verbose_msg_value(msg, num);
}

void log_v(const char * msg, int32_t num){
  log_verbose_msg_value(msg, num);
}

void log_v(const char * msg, int num){
  log_verbose_msg_value(msg, num);
}

void log_v(const char * msg, float num){
  log_verbose_msg_value(msg, num);
}

void log_write(const char * msg){
  log_info_msg(msg);
}

void log_write(uint32_t num){
  log_info_write_value(num);
}

void log_write(int32_t num){
  log_info_write_value(num);
}

void log_write(int num){
  log_info_write_value(num);
}

void log_write(float num){
  log_info_write_value(num);
}

void log_write_hex(uint32_t num, uint8_t width){
  log_info_write_hex(num, width);
}

static void log_write_verbose_msg(const char * msg){
  if(debug_en){
    DEBUGSER.print(msg);
  }
  if(settings.verbose_usb && usb_connected){
    Serial.print(msg);
    usb_flush();
  }
}

template <typename T>
static void log_write_verbose_value(T num){
  if(debug_en){
    DEBUGSER.print(num);
  }
  if(settings.verbose_usb && usb_connected){
    Serial.print(num);
    usb_flush();
  }
}

static void log_write_verbose_hex(uint32_t num, uint8_t width){
  if(debug_en){
    write_hex_to_stream(DEBUGSER, num, width);
  }
  if(settings.verbose_usb && usb_connected){
    write_hex_to_stream(Serial, num, width);
    usb_flush();
  }
}

void log_write_v(const char * msg){
  log_write_verbose_msg(msg);
}

void log_write_v(uint32_t num){
  log_write_verbose_value(num);
}

void log_write_v(int32_t num){
  log_write_verbose_value(num);
}

void log_write_v(int num){
  log_write_verbose_value(num);
}

void log_write_v(float num){
  log_write_verbose_value(num);
}

void log_write_hex_v(uint32_t num, uint8_t width){
  log_write_verbose_hex(num, width);
}

void log_s(const char * msg){
  if(!settings.verbose_usb && usb_connected){
    Serial.print(msg);
    usb_flush();
  }
}

void log_s(const char * msg, uint32_t num, const char * suffix){
  if(!settings.verbose_usb && usb_connected){
    Serial.print(msg);
    Serial.print(num);
    Serial.println(suffix);
    usb_flush();
  }
}

void log_e(const char * msg){
  if(errors_en){
    DEBUGSER.print(msg);
    if(display_present()) {display_add_line(String(msg));}
  }
  if(usb_connected){
    Serial.print(msg);
  }
}

void log_flush(){
  if(debug_en){
    DEBUGSER.flush();
  }
  if(usb_connected){
    usb_flush();
  }
}

void log_ser_begin(){
  if(debug_en){
    DEBUGSER.begin(115200*div_cpu);
  }
}

void log_enable_debug(){
  debug_en = true;
}
void log_disable_debug(){
  debug_en = false;
}

void log_set_debug(bool en){
    debug_en = en;
}

bool debug_enabled(){
    return debug_en;
}

void log_set_error(bool en){
    errors_en = en;
}