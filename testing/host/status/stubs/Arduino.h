// Host stub for Arduino.h + the CMSIS bits status.cpp touches.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#define PM_RCAUSE_POR   (1u << 0)
#define PM_RCAUSE_BOD12 (1u << 1)
#define PM_RCAUSE_BOD33 (1u << 2)
#define PM_RCAUSE_EXT   (1u << 4)
#define PM_RCAUSE_WDT   (1u << 5)
#define PM_RCAUSE_SYST  (1u << 6)

typedef struct { struct { uint8_t reg; } RCAUSE; } Pm;
extern Pm *PM;
