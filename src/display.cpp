#include "display.h"

#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

String line[NUM_LINES];
int line_pos = NUM_LINES-1;
int linecount =0;
bool display_init_ok = false;
 bool printdelay = false;

// Display
#ifdef USE_DISPLAY

void display_delay(uint32_t t){
    if(display_init_ok){
        delay(t);  // wait some time to keep the message on the screen
    }
}

bool display_check_present(uint8_t address){
    Wire.beginTransmission(address);
    if(Wire.endTransmission() == 0){
        return true;
    }
    return false;
}

bool display_present(){
    return display_init_ok;
}

void display_clear(){
    display.clearDisplay();
    display.display();
}

void display_print_linebuffer(){
    static uint32_t last_line_print = 0;
    if(printdelay){
        if(millis() - last_line_print < 50){
            int del = 50 - (millis() - last_line_print);
            delay(del > 0? del:0);
        }
    }
    display.clearDisplay();
    for (int i = 0; i < linecount; i++){
        display.setCursor(0, i*LINESPACING);
        int pos = line_pos - linecount +1 + i;
        if(pos < 0){ pos += NUM_LINES;}
        display.print(line[pos]);
        //DEBUGSER.print("pos:" + String(pos));
        //DEBUGSER.println(line[pos]);
    }
    //DEBUGSER.println("print");
    display.display();
    last_line_print = millis();
}
#endif

void display_add_line(String txt){
#ifdef USE_DISPLAY
if(display_init_ok){
    line_pos++;
    if(line_pos >= NUM_LINES){line_pos -= NUM_LINES;}
    txt.replace("\n", "");
    line[line_pos] = txt.substring(0,MAX_CHARS);
    linecount ++;
    if(linecount > NUM_LINES){ linecount = NUM_LINES;}
    display_print_linebuffer();
}
#endif
}



void setup_display(){
#ifdef USE_DISPLAY
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    //DEBUGSER.println(F("SSD1306 allocation failed"));
  } else{
    display_init_ok = display_check_present(SCREEN_ADDRESS);
    //log_i("Display setup ok\n");
    display.display();
    display.clearDisplay();
    #ifdef FONT_PICO
    display.setFont(&Picopixel);
    #endif
    display.setTextSize(0);      // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);     // Start at top-left corner

    display_add_line("Version: " + (String)VERSION);
    display_add_line("FW Build: " + (String)__DATE__ + " "+ (String)__TIME__); 
  }
#endif
}