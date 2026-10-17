// Host stub for Arduino.h, for compiling src/fanet.cpp natively.
// MSVC has no __attribute__((packed)); pack(1) from here on gives the packed
// structs in fanet.h the same layout the firmware has.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#pragma pack(push, 1)
#define __attribute__(x)
#define constrain(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#include <string>
using String = std::string;   // fanet.h keeps the station name as an Arduino String
#pragma warning(disable: 4103)  // pack(1) deliberately stays in effect across the headers
