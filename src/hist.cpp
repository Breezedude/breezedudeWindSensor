#include "hist.h"

WindSample wind_history[WIND_HIST_LEN]; // gust ringbuffer
uint8_t wind_hist_pos = 0; // current position in ringbuffer

static constexpr float RAD_TO_DEG_F = 180.0f / 3.14159265358979323846f;

// Wind directions are whole degrees, so sinf()/cosf() are replaced by a Q15
// quarter-wave table.  Besides being much faster than libm on a core without
// an FPU, this keeps __ieee754_rem_pio2f + __kernel_rem_pio2f (~2.6 kB of
// argument range reduction) out of the image - they were linked in solely for
// these two calls.  Max amplitude error is 1.5e-5, which is far below the
// 360/256 deg quantisation FANET uses on the wire.
static const int16_t SIN_Q15[91] = {
       0,    572,   1144,   1715,   2286,   2856,   3425,   3993,   4560,   5126,
    5690,   6252,   6813,   7371,   7927,   8481,   9032,   9580,  10126,  10668,
   11207,  11743,  12275,  12803,  13328,  13848,  14364,  14876,  15383,  15886,
   16383,  16876,  17364,  17846,  18323,  18794,  19260,  19720,  20173,  20621,
   21062,  21497,  21925,  22347,  22762,  23170,  23571,  23964,  24351,  24730,
   25101,  25465,  25821,  26169,  26509,  26841,  27165,  27481,  27788,  28087,
   28377,  28659,  28932,  29196,  29451,  29697,  29934,  30162,  30381,  30591,
   30791,  30982,  31163,  31335,  31498,  31650,  31794,  31927,  32051,  32165,
   32269,  32364,  32448,  32523,  32587,  32642,  32687,  32722,  32747,  32762,
   32767
};

// sin(deg) scaled by 32767. Accepts any integer degree, including the
// unclamped values a misbehaving sensor may report.
static int16_t sin_q15(int deg){
  deg %= 360;
  if(deg < 0){ deg += 360; }
  if(deg <=  90){ return  SIN_Q15[deg]; }
  if(deg <= 180){ return  SIN_Q15[180 - deg]; }
  if(deg <= 270){ return -SIN_Q15[deg - 180]; }
  return -SIN_Q15[360 - deg];
}

static inline int16_t cos_q15(int deg){ return sin_q15(deg + 90); }


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
  if (val_wind < 0.0f) {
    val_wind = 0.0f;
  }
  wind_history[wind_hist_pos].wind = (uint32_t)(val_wind * 10.0f);
  wind_history[wind_hist_pos].time = time();
}

// Adds/updates gust value in ringbuffer
void add_wind_history_gust(float val_gust){
  check_wind_hist_bin(); // Agg slot time is over, switch to next
  wind_history[wind_hist_pos].gust = (uint32_t)(fabsf(val_gust) * 10.0f);
  wind_history[wind_hist_pos].time = time();
}

// Adds/updates dir value in ringbuffer
void add_wind_history_dir(int val_dir){
  check_wind_hist_bin(); // Agg slot time is over, switch to next
  wind_history[wind_hist_pos].dir_raw = val_dir;
  wind_history[wind_hist_pos].time = time();
}

// Returns the average wind speed and direction over all ringbuffer slots not older than age.
// ret.gust is not used by the caller (get_gust_from_hist is used instead).
WindSample get_wind_from_hist(uint32_t age){
  WindSample ret = {};
  uint32_t now = time();

  // Accumulate the direction vector in Q15 integers: WIND_HIST_LEN * 32767
  // stays far inside int32_t, so no float math is needed inside the loop.
  int32_t y_part = 0;
  int32_t x_part = 0;
  int samplecount =0;

  int p = wind_hist_pos;
  for( int i = 0; i < WIND_HIST_LEN; i++){
    if(p >= WIND_HIST_LEN){
      p -= WIND_HIST_LEN;
    }
    if( ( wind_history[p].time && (now - wind_history[p].time) < age)){
      ret.wind += wind_history[p].wind;
      ret.gust += wind_history[p].gust;
      x_part += cos_q15(wind_history[p].dir_raw);
      y_part += sin_q15(wind_history[p].dir_raw);
      samplecount++;
    }
    p++;
  }
  if(samplecount > 0){
    ret.valid   = true;
    ret.dir_raw = (int)(atan2f((float)y_part, (float)x_part) * RAD_TO_DEG_F);
    if(ret.dir_raw < 0){ ret.dir_raw += 360;}
    ret.wind /= samplecount;
    ret.gust /= samplecount;
    ret.valid = true;
  } else {
    //log_e("Samplecount is 0\r\n");
    ret.valid = false;
  }
  return ret;
}

// Keeps the GUSTBUFFERLEN highest values seen so far, sorted descending.
static void insert_sorted(uint32_t* arr, const int arrlen, uint32_t value){
  if (arr == nullptr || arrlen <= 0) {
    return;
  }
  if (value <= arr[arrlen - 1]) {
        return; // value too small, skip
    }
  int i = arrlen - 2;
    // Shift elements down to make room for the new value.
    for (; (i >= 0 && arr[i] < value); i--) {
        arr[i + 1] = arr[i]; // Verschiebe Elemente nach rechts
    }
    arr[i+1] = value;
}

#define GUSTBUFFERLEN 7
// Index of the value we normally report: the 5th highest, so a single reading
// error or a one-off spike cannot become the transmitted gust.
#define GUST_SELECT_INDEX 4

float get_gust_from_hist(uint32_t age){
  uint32_t ret[GUSTBUFFERLEN] = {0,0,0,0,0,0,0};
  uint32_t now = time();
  int samplecount = 0;
  int p = wind_hist_pos;
  for( int i = 0; i < WIND_HIST_LEN; i++){
    if(p == WIND_HIST_LEN){
      p -= WIND_HIST_LEN;
    }
    // check the hisftory for gust values within the set gust age.
    if(( wind_history[p].time && (now - wind_history[p].time) < age)){
      insert_sorted(ret, GUSTBUFFERLEN, wind_history[p].gust);
      samplecount++;
    }
    p++;
  }

  if(samplecount == 0){
    return 0.0f; // nothing in the window - caller decides what to do
  }
  // With fewer than GUST_SELECT_INDEX+1 samples (right after boot, after a
  // sensor dropout or after an undervoltage pause) ret[GUST_SELECT_INDEX] is
  // still 0 and we would transmit a gust of zero. Fall back to the lowest of
  // the values we actually have, which never over-reports.
  const int idx = (samplecount > GUST_SELECT_INDEX) ? GUST_SELECT_INDEX : (samplecount - 1);
  return ret[idx] / 10.0f;
}
