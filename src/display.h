 #pragma once
 #include <Arduino.h>

 #define USE_DISPLAY // enable support for SSD1306 128x64 0.96" I2C display

#define FONT_PICO
//#define FONT_NORMAL

#ifdef FONT_NORMAL
    #define LINESPACING 10
    #define MAX_CHARS 20
    #define NUM_LINES 6
#endif

#ifdef FONT_PICO
    #include <Fonts/Picopixel.h>
    //#include <Fonts/Org_01.h>
    #define LINESPACING 7
    #define MAX_CHARS 40
    #define NUM_LINES 9
#endif


#define SSD1306_NO_SPLASH
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

// Display


void display_delay(uint32_t t);
bool display_check_present(uint8_t address);
bool display_present();
void display_clear();
void display_print_linebuffer();
void display_add_line(String txt);
void setup_display();
