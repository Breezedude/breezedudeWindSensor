#include "logging.h"

bool debug_en = false;
bool errors_en = false;

void log_i(const char * msg){
  if(debug_en){
    DEBUGSER.print(msg);
    DEBUGSER.flush();
    if(display_present()) {display_add_line(String(msg));}
  }
  if(usb_connected){
    Serial.print(msg);
  }
}
void log_i(const char * msg, uint32_t num){
  if(debug_en){
    DEBUGSER.print(msg);
    DEBUGSER.println(num);
    DEBUGSER.flush();
    if(display_present()) {display_add_line(String(msg) + String(num));}
  }
  if(usb_connected){
    Serial.print(msg);
    Serial.println(num);
  }
}
void log_i(const char * msg, int32_t num){
  if(debug_en){
    DEBUGSER.print(msg);
    DEBUGSER.println(num);
    DEBUGSER.flush();
    if(display_present()) {display_add_line(String(msg) + String(num));}
  }
  if(usb_connected){
    Serial.print(msg);
    Serial.println(num);
  }
}
void log_i(const char * msg, int num){
  if(debug_en){
    DEBUGSER.print(msg);
    DEBUGSER.println(num);
    if(display_present()) {display_add_line(String(msg) + String(num));}
  }
  if(usb_connected){
    Serial.print(msg);
    Serial.println(num);
  }
}
void log_i(const char * msg, float num){
  if(debug_en){
    DEBUGSER.print(msg);
    DEBUGSER.println(num);
    if(display_present()) {display_add_line(String(msg) + String(num));}
  }
  if(usb_connected){
    Serial.print(msg);
    Serial.println(num);
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