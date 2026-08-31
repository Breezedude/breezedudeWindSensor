// Project-local TinyUSB configuration.
//
// The Adafruit SAMD port (framework-arduino-samd-adafruit/.../arduino/ports/samd/
// tusb_config_samd.h) enables MSC, HID, MIDI, VENDOR and VIDEO unconditionally.
// TinyUSB registers all device class drivers in a static table (_usbd_driver[]
// in usbd.c), so --gc-sections cannot drop them even though this firmware only
// ever uses CDC.  That costs ~7.3 kB flash and ~1.9 kB RAM.
//
// tusb_option.h honours CFG_TUSB_CONFIG_FILE, so we point the build at this file
// instead (see -DCFG_TUSB_CONFIG_FILE in platformio.ini).  Everything below
// mirrors the upstream SAMD config; only the device class counts differ.

#ifndef _BREEZEDUDE_TUSB_CONFIG_H_
#define _BREEZEDUDE_TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------
#ifdef __SAMD51__
#define CFG_TUSB_MCU OPT_MCU_SAMD51
#else
#define CFG_TUSB_MCU OPT_MCU_SAMD21
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_NONE
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG 0
#endif

#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN TU_ATTR_ALIGNED(4)

// Enable device stack
#define CFG_TUD_ENABLED 1

// Host stack: kept enabled to match upstream (tuh_init() is never called, so the
// whole host stack is already removed by --gc-sections).
#define CFG_TUH_ENABLED 1
#define CFG_TUH_MAX3421 1

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#define CFG_TUD_ENDPOINT0_SIZE 64

//------------- CLASS -------------//
// CDC is the only class this firmware uses (settings/debug console over USB).
// Each of the others can be re-enabled here if a build ever needs it.
#define CFG_TUD_CDC 1
#define CFG_TUD_MSC 0
#define CFG_TUD_HID 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0
#define CFG_TUD_VIDEO 0
#define CFG_TUD_VIDEO_STREAMING 0

// CDC FIFO size of TX and RX
#define CFG_TUD_CDC_RX_BUFSIZE 256
#define CFG_TUD_CDC_TX_BUFSIZE 256

//--------------------------------------------------------------------
// Host Configuration
//--------------------------------------------------------------------

// Size of buffer to hold descriptors and other data used for enumeration
#define CFG_TUH_ENUMERATION_BUFSIZE 256

// Number of hub devices
#define CFG_TUH_HUB 1

// max device support (excluding hub device): 1 hub typically has 4 ports
#define CFG_TUH_DEVICE_MAX (3 * CFG_TUH_HUB + 1)

// Number of mass storage
#define CFG_TUH_MSC 1

// Number of HIDs
#define CFG_TUH_HID (3 * CFG_TUH_DEVICE_MAX)

// Number of CDC interfaces
// FTDI and CP210x are not part of CDC class, only to re-use CDC driver API
#define CFG_TUH_CDC 1
#define CFG_TUH_CDC_FTDI 1
#define CFG_TUH_CDC_CP210X 1
#define CFG_TUH_CDC_CH34X 1

// RX & TX fifo size
#define CFG_TUH_CDC_RX_BUFSIZE 64
#define CFG_TUH_CDC_TX_BUFSIZE 64

// Set Line Control state on enumeration/mounted: DTR (bit 0), RTS (bit 1)
#define CFG_TUH_CDC_LINE_CONTROL_ON_ENUM 0x03

// Set Line Coding on enumeration/mounted, value for cdc_line_coding_t
// bit rate = 115200, 1 stop bit, no parity, 8 bit data width
#define CFG_TUH_CDC_LINE_CODING_ON_ENUM                                        \
  { 115200, CDC_LINE_CONDING_STOP_BITS_1, CDC_LINE_CODING_PARITY_NONE, 8 }

#ifdef __cplusplus
}
#endif

#endif /* _BREEZEDUDE_TUSB_CONFIG_H_ */
