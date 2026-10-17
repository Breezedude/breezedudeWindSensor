#include "logging.h"
#include "tools.h"
#include "sleep.h"
#include <string.h>

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

// Line markers ---------------------------------------------------------------
//
// Every line starts with one character and a bar, so a reader (or the test
// rig's replay view) can tell at a glance which lines are worth looking at:
//
//   E|  endangers regular operation - the station cannot do its job
//   W|  a fault occurred but does not weigh heavily - rejected packet,
//       missing optional sensor, a single discarded command
//   I|  normal operation
//   D|  extra detail, e.g. OTA chunk counters
//
// The marker belongs to the LINE, not to the call. Most messages are assembled
// from several log_i()/log_write() calls, and marking each of them would give
// "I| SAVE OK I| 5I|  changes". So the state below remembers where each stream
// stands and writes the marker only at the start of a line.
//
// The same state fixes an older truncation: log_e() writes when errors_en is
// set, but its log_write() continuations used to be gated on debug_en. With
// DEBUG=0 - the normal field setting - every error line broke off after its
// first fragment. A continuation now inherits the level of the line it belongs
// to, so it passes the same gate as the fragment that opened the line.

#define LOG_ERROR  'E'
#define LOG_WARN   'W'
#define LOG_INFO   'I'
// Extra detail. Two levels share the D marker because they say the same thing
// to a reader; they differ in when they are printed. log_d() goes wherever
// log_i() goes. log_v() is the detailed running log - every wakeup, window,
// reading and sleep as its own line - and only comes out while USB_VERBOSE is
// set, on both ports. That is what to switch on when a station has to be
// watched step by step. Marker and gate are deliberately separate: sorting a
// message into E/W/I/D is a statement about its content, and must not quietly
// change whether it comes out.
#define LOG_DETAIL  'D'
#define LOG_VERBOSE 'V'
// The compact running log: one line per wakeup and per transmission, printed
// only while USB_VERBOSE is off - the normal state of a station in the field,
// where every byte on the debug UART is awake time. It reads as ordinary
// information, so it carries the I marker.
#define LOG_STATUS 'S'
// Machine readable payload - the body of a DUMP is the settings file itself
// and has to come out byte for byte, so it gets no marker. Gated like I.
#define LOG_RAW    'R'
// A fragment that continues the current line and inherits its level.
#define LOG_CONT   '\0'

static bool ds_at_start = true;    // DEBUGSER stands at the start of a line
static bool usb_at_start = true;   // Serial stands at the start of a line
static bool line_open = false;     // a line is half written, gate independent
static char open_level = LOG_INFO; // the level of that half written line

static char marker_of(char level){
  if(level == LOG_STATUS) return LOG_INFO;
  if(level == LOG_VERBOSE) return LOG_DETAIL;
  if(level == LOG_RAW) return 0;
  return level;
}

// A continuation follows the line it belongs to. Outside any line it counts as
// information - that is what a bare log_write() at the start of a line is.
static char resolve(char level){
  if(level != LOG_CONT) return level;
  return line_open ? open_level : LOG_INFO;
}

static bool ds_allows(char level){
  if(level == LOG_ERROR || level == LOG_WARN) return errors_en;
  if(level == LOG_VERBOSE) return debug_en && settings.verbose_usb;
  if(level == LOG_STATUS)  return debug_en && !settings.verbose_usb;
  return debug_en;                                    // I, D and R
}

static bool usb_allows(char level){
  if(!usb_connected) return false;
  if(level == LOG_VERBOSE) return settings.verbose_usb;
  if(level == LOG_STATUS)  return !settings.verbose_usb;
  return true;                                        // E, W, I, D and R
}

// Serial.flush() alone only calls tud_cdc_write_flush(), which marks data as
// ready but does not run the TinyUSB task.  yield() drives tud_task() so the
// USB IN transfer is actually scheduled and completed without waiting for loop().
static void usb_flush(){
  Serial.flush();
  yield();
}

static void begin_line(Stream &stream, bool &at_start, char level){
  if(at_start){
    const char marker = marker_of(level);
    if(marker){
      stream.print(marker);
      stream.print('|');
      stream.print(' ');
    }
    at_start = false;
  }
}

// Remembers, independent of any gate, whether the line is still open. Both
// streams get the same call sequence, so one shared flag is enough - only the
// gates differ, and those are handled per stream.
static void note_text(char level, const char *msg, size_t len){
  if(len == 0u) return;
  line_open = (msg[len - 1u] != '\n');
  open_level = level;
}

// Writes text that may carry newlines of its own, marking every line it
// starts. Several messages have an embedded break - the reset banner is one
// call containing "\r\n----- RESET -----\r\n" - and marking only the first
// character would leave exactly the lines a reader looks for unmarked.
static void put_text(Stream &stream, bool &at_start, char level, const char *msg){
  const char *p = msg;
  while(*p){
    const char *nl = strchr(p, '\n');
    const size_t seg = nl ? (size_t)(nl - p) + 1u : strlen(p);

    // A line holding nothing but its terminator separates, it does not
    // report - a marker on it would be noise.
    bool blank = true;
    for(size_t i = 0; i < seg; i++){
      if(p[i] != '\r' && p[i] != '\n'){ blank = false; break; }
    }
    if(!blank){
      begin_line(stream, at_start, level);
    }

    stream.write((const uint8_t *)p, seg);
    if(nl){
      at_start = true;
    }
    p += seg;
  }
}

static void emit(char level, const char *msg){
  const char lvl = resolve(level);
  const size_t len = strlen(msg);
  // An empty fragment must not open a line - otherwise it leaves a bare
  // marker behind and the real text of the line starts unmarked.
  if(len == 0u) return;

  if(ds_allows(lvl)){
    put_text(DEBUGSER, ds_at_start, lvl, msg);
  }
  if(usb_allows(lvl)){
    put_text(Serial, usb_at_start, lvl, msg);
    usb_flush();
  }
  note_text(lvl, msg, len);
}

template <typename T>
static void emit_num(char level, T num, bool newline){
  const char lvl = resolve(level);

  if(ds_allows(lvl)){
    begin_line(DEBUGSER, ds_at_start, lvl);
    if(newline){ DEBUGSER.println(num); ds_at_start = true; }
    else       { DEBUGSER.print(num); }
  }
  if(usb_allows(lvl)){
    begin_line(Serial, usb_at_start, lvl);
    if(newline){ Serial.println(num); usb_at_start = true; }
    else       { Serial.print(num); }
    usb_flush();
  }
  line_open = !newline;
  open_level = lvl;
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

static void emit_hex(char level, uint32_t num, uint8_t width){
  const char lvl = resolve(level);

  if(ds_allows(lvl)){
    begin_line(DEBUGSER, ds_at_start, lvl);
    write_hex_to_stream(DEBUGSER, num, width);
  }
  if(usb_allows(lvl)){
    begin_line(Serial, usb_at_start, lvl);
    write_hex_to_stream(Serial, num, width);
    usb_flush();
  }
  line_open = true;
  open_level = lvl;
}

// The display shows the message without the marker - three characters of a
// four line screen are better spent on the text.
// display_present() is a constant false in builds without a display, and it
// has to stay its OWN test for the linker to see that. Folded into a combined
// condition it stopped folding, the String work stayed alive, and String(float)
// pulled in newlib's float formatting - 7 kB out of a 122 kB flash, for a
// display that is not fitted.
static void to_display(char level, const char *msg){
  if(display_present()){
    if(ds_allows(level)){ display_add_line(String(msg)); }
  }
}

template <typename T>
static void to_display_value(char level, const char *msg, T num){
  if(display_present()){
    if(ds_allows(level)){ display_add_line(String(msg) + String(num)); }
  }
}

static void log_info_msg(const char * msg){
  emit(LOG_INFO, msg);
  to_display(LOG_INFO, msg);
}

template <typename T>
static void log_info_msg_value(const char * msg, T num){
  emit(LOG_INFO, msg);
  emit_num(LOG_CONT, num, true);
  to_display_value(LOG_INFO, msg, num);
}

template <typename T>
static void log_detail_value(const char * msg, T num){
  emit(LOG_DETAIL, msg);
  emit_num(LOG_CONT, num, true);
  to_display_value(LOG_DETAIL, msg, num);
}

static void log_verbose_msg(const char * msg){
  emit(LOG_VERBOSE, msg);
  to_display(LOG_VERBOSE, msg);
}

template <typename T>
static void log_verbose_msg_value(const char * msg, T num){
  emit(LOG_VERBOSE, msg);
  emit_num(LOG_CONT, num, true);
  to_display(LOG_VERBOSE, msg);
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
  emit(LOG_CONT, msg);
}

void log_write(uint32_t num){
  emit_num(LOG_CONT, num, false);
}

void log_write(int32_t num){
  emit_num(LOG_CONT, num, false);
}

void log_write(int num){
  emit_num(LOG_CONT, num, false);
}

void log_write(float num){
  emit_num(LOG_CONT, num, false);
}

void log_write_hex(uint32_t num, uint8_t width){
  emit_hex(LOG_CONT, num, width);
}

void log_write_v(const char * msg){
  emit(LOG_VERBOSE, msg);
}

void log_write_v(uint32_t num){
  emit_num(LOG_VERBOSE, num, false);
}

void log_write_v(int32_t num){
  emit_num(LOG_VERBOSE, num, false);
}

void log_write_v(int num){
  emit_num(LOG_VERBOSE, num, false);
}

void log_write_v(float num){
  emit_num(LOG_VERBOSE, num, false);
}

void log_write_hex_v(uint32_t num, uint8_t width){
  emit_hex(LOG_VERBOSE, num, width);
}

void log_s(const char * msg){
  emit(LOG_STATUS, msg);
}

void log_s(const char * msg, uint32_t num, const char * suffix){
  emit(LOG_STATUS, msg);
  emit_num(LOG_CONT, num, false);
  emit(LOG_CONT, suffix);
  emit(LOG_CONT, "\r\n");
}

void log_w(const char * msg){
  emit(LOG_WARN, msg);
  to_display(LOG_WARN, msg);
}

template <typename T>
static void log_warn_value(const char * msg, T num){
  emit(LOG_WARN, msg);
  emit_num(LOG_CONT, num, true);
  to_display_value(LOG_WARN, msg, num);
}

void log_w(const char * msg, uint32_t num){ log_warn_value(msg, num); }
void log_w(const char * msg, int32_t num) { log_warn_value(msg, num); }
void log_w(const char * msg, int num)     { log_warn_value(msg, num); }
void log_w(const char * msg, float num)   { log_warn_value(msg, num); }

void log_e(const char * msg){
  emit(LOG_ERROR, msg);
  to_display(LOG_ERROR, msg);
}

void log_d(const char * msg){
  emit(LOG_DETAIL, msg);
  to_display(LOG_DETAIL, msg);
}

void log_d(const char * msg, uint32_t num){ log_detail_value(msg, num); }
void log_d(const char * msg, int32_t num) { log_detail_value(msg, num); }
void log_d(const char * msg, int num)     { log_detail_value(msg, num); }
void log_d(const char * msg, float num)   { log_detail_value(msg, num); }

void log_raw(const char * msg){
  emit(LOG_RAW, msg);
}

void log_flush(){
  // Only under debug_en: log_ser_begin() starts DEBUGSER on that condition,
  // and Uart::flush() busy-waits on a TXC flag that an unstarted SERCOM never
  // raises.
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
    // Uart::begin() runs a SWRST that clears RUNSTDBY, SFDE and BAUD, so the
    // standby configuration has to be put back after every single begin().
    configure_debug_uart_standby(settings.uart_wake && debug_en);
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
