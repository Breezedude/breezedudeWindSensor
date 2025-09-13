#pragma once
#include <Arduino.h>
#include "logging.h"
#include "defines.h"

extern uint32_t time();


// Gust History, for data transmission
typedef struct g{
  uint32_t time;
  uint32_t gust;
  uint32_t wind;
  int dir_raw;
} WindSample;



WindSample wind_history[WIND_HIST_LEN]; // gust ringbuffer
uint8_t wind_hist_pos = 0; // current position in ringbuffer

// Data histroy, mainly for heater control
typedef struct h{
  bool set = false;
  int8_t wind;
  int8_t temp;
  int8_t humd;
  int8_t light;
  int8_t batt;
  bool pv_charging;
  bool pv_done;
} History;

History history[HISTORY_LEN];
int hist_count =0;
uint32_t last_history =0;


void check_wind_hist_bin(){
  if( wind_history[wind_hist_pos].time && (time() - wind_history[wind_hist_pos].time) > WIND_HIST_STEP){
    // copy old values if there is an read error from serial to avoid 0 to be included in average
    uint32_t g = wind_history[wind_hist_pos].gust;
    uint32_t w = wind_history[wind_hist_pos].wind;
    int d = wind_history[wind_hist_pos].dir_raw;

    wind_hist_pos++;
    if(wind_hist_pos == WIND_HIST_LEN){
      wind_hist_pos = 0;
    }
    // reset values if already set (overwrite ringbuffer)
    wind_history[wind_hist_pos].gust = g;
    wind_history[wind_hist_pos].wind = w;
    wind_history[wind_hist_pos].dir_raw = d;
    wind_history[wind_hist_pos].time = 0; // will be set by every add_... 
  }
}

// Adds/updates wind value in ringbuffer
void add_wind_history_wind(float val_wind){
  check_wind_hist_bin(); // Agg slot time is over, switch to next
  wind_history[wind_hist_pos].wind = val_wind*10;
  wind_history[wind_hist_pos].time = time();
}

// Adds/updates gust value in ringbuffer
void add_wind_history_gust(float val_gust){
  check_wind_hist_bin(); // Agg slot time is over, switch to next
  wind_history[wind_hist_pos].gust = abs(val_gust)*10;
  wind_history[wind_hist_pos].time = time();
}

// Adds/updates dir value in ringbuffer
void add_wind_history_dir(int val_dir){
  check_wind_hist_bin(); // Agg slot time is over, switch to next
  wind_history[wind_hist_pos].dir_raw = val_dir;
  wind_history[wind_hist_pos].time = time();
}

// gets the highest wind value in ringbuffer, not older than age
WindSample get_wind_from_hist(uint32_t age){
  WindSample ret = {0,0,0,0};

  float y_part = 0;
  float x_part = 0;
  int samplecount =0;

  uint8_t p = wind_hist_pos;
  for( int i = 0; i < WIND_HIST_LEN; i++){
    if(p == WIND_HIST_LEN){
      p -= WIND_HIST_LEN;
    }
    if( ( wind_history[p].time && (time()- wind_history[p].time) < age)){
      ret.wind += wind_history[p].wind;
      ret.gust += wind_history[p].gust;
      x_part += cos (wind_history[p].dir_raw * M_PI / 180);
      y_part += sin (wind_history[p].dir_raw * M_PI / 180);
      samplecount++;
    }
    p++;
  }
  if(samplecount > 0){
    ret.dir_raw = atan2 (y_part / samplecount, x_part / samplecount) * 180 / M_PI;
    if(ret.dir_raw <0){ ret.dir_raw +=360;}

   // log_i("Sum Wind: ", ret.wind);
   // log_i("Sum Gust: ", ret.gust);
   // log_i("Samples: ", samplecount);
    ret.wind /= samplecount;
    ret.gust /= samplecount;
  } else {
    //log_e("Samplecount is 0\r\n");
  }
  return ret;
}

void insert_sorted(uint32_t* arr, int arrlen, uint32_t value){
    if (value <= arr[arrlen - 1]) {
        return; // value too small, skip
    }
    int i = 0;
   // while (pos < arrlen && value < arr[pos]) {
   //     pos++; // Find position to insert the value.
   // }
    // Shift elements down to make room for the new value.
    for (i = arrlen - 1; (i >= 0 && arr[i] < value); i--) {
        arr[i + 1] = arr[i]; // Verschiebe Elemente nach rechts
    }
   // if (pos < arrlen) {
   //     memmove(&arr[pos + 1], &arr[pos], (arrlen - 1 - pos) * sizeof(uint32_t));
   // }
    arr[i+1] = value;
}

float get_gust_from_hist(uint32_t age){
  #define GUSTBUFFERLEN 7
  uint32_t ret[GUSTBUFFERLEN] = {0,0,0,0,0,0,0};
  uint8_t p = wind_hist_pos;
  for( int i = 0; i < WIND_HIST_LEN; i++){
    if(p == WIND_HIST_LEN){
      p -= WIND_HIST_LEN;
    }
    // check the hisftory for gust values within the set gust age.
    if(( wind_history[p].time && (time()- wind_history[p].time) < age)){
      //printf("age: %lu, value: %0.2f\n", (time()- wind_history[p].time), (float)(wind_history[p].gust)/10.0 );
      insert_sorted(ret, GUSTBUFFERLEN, wind_history[p].gust);
    }
    p++;
  }
  //printf("0: %lu 1: %lu 2: %lu 3: %lu 4: %lu \n", ret[0], ret[1], ret[2], ret[3], ret[4] );
  return ret[4]/10.0; // return 5th highest value to avoid sending a reading error or one time high value
}



// History ----------------------------------------------------------------------------------------------------------------------
// save history value if time is ready  for a new one
void save_history(float wind_speed, float temperature, int humidity, int light_lux, float batt_volt, bool pv_charging, bool pv_done){
  static uint32_t last_history = 0;
  if(time() - last_history > HISTORY_INTERVAL){
    last_history = time();
    hist_count++;
    if(hist_count == HISTORY_LEN){hist_count = 0;}
    log_i("Saving Hist no: ", hist_count);
    history[hist_count].set = true;
    history[hist_count].wind = wind_speed*10;
    history[hist_count].temp = temperature;
    history[hist_count].humd = humidity;
    history[hist_count].light = light_lux/100;
    history[hist_count].batt = (batt_volt-2)*100;
    history[hist_count].pv_charging = pv_charging;
    history[hist_count].pv_done = pv_done;
  }
}

// calc sum of light sensor reading in n last ringbuffer entries
uint32_t history_sum_light(int len){
  int sum = 0;
  for (int i=0; i<len; i++){
    int hpos = hist_count-i;
    if(hpos <0){hpos += HISTORY_LEN;} // get pos of ringbuffer

    if(history[hpos].set){
      sum += history[hpos].light;
      if(sum > 2147480000){break;} // abort if very high value
    }
  }
  return sum;
}

// calc sum of wind speed reading in n last ringbuffer entries
uint32_t history_sum_wind(int len){
  int sum = 0;
  for (int i=0; i<len; i++){
    int hpos = hist_count-i;
    if(hpos <0){hpos += HISTORY_LEN;} // get pos of ringbuffer

    if(history[hpos].set){
      sum += history[hpos].wind;
      if(sum > 2147480000){break;} // abort if very high value
    }
  }
  return sum;
}