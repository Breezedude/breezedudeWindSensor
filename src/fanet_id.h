#pragma once
#include <stddef.h>
#include <stdint.h>

// The FANET address of a station comes from the UUID of its MCU, and has to stay
// the same for the life of the station: backend, map and device management all
// know a station by it.
//
// Up to hardware 2.4 it is the first two bytes of the UUID. Those are not spread
// evenly across the chips, and two boards can end up with the same address. So the
// boards with an ID resistor, 2.5 and later, use a checksum over the whole UUID,
// CRC-16/MODBUS over the 16 bytes in the order the UUID is printed in. The older
// boards keep the old way, since an update must not change their address.
uint16_t fanet_id_legacy(const uint8_t *uuid);
// Never 0x0000 or 0xFFFF: an address of all zeros means broadcast in FANET, and
// all ones is what an erased or missing value looks like.
uint16_t fanet_id_from_uuid(const uint8_t *uuid, size_t len);
