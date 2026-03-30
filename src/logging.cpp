#include "logging.h"
#include "tools.h"
#include <stdarg.h>
#include <stdio.h>

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
    DEBUGSER.flush();
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
    DEBUGSER.flush();
    if(display_present()) {display_add_line(String(msg) + String(num));}
  }
  if(usb_connected){
    Serial.print(msg);
    Serial.println(num);
    usb_flush();
  }
}

static void log_verbose_msg(const char * msg){
  if(debug_en){
    DEBUGSER.print(msg);
    DEBUGSER.flush();
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
    DEBUGSER.flush();
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

void log_if(const char * fmt, ...){
  char buffer[192];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  log_info_msg(buffer);
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