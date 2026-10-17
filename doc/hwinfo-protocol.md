# HWInfo packet (FANET type 0x0A, vendor 0xBD)

The station reports about itself in a FANET *hardware info* packet. Four
decoders read it - the groundstation frontend, the backend, gsmap and device
management - so this file is the contract: the firmware packs exactly what is
written here (`src/fanet.h`, `pack_hwinfo()` in `src/fanet.cpp`, filled in
`src/ota_lora.cpp`), and `testing/host/hwinfo` pins the bytes.

All multi-byte fields are little-endian. Bit 0 is the least significant bit.

## Packet layout

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 1 | FANET header byte 0 | bit 0-5 type = 0x0A, bit 6 forward, bit 7 ext header (0) |
| 1 | 1 | vendor | 0xBD |
| 2 | 2 | address | station address, LE |
| 4 | 1 | HWInfo byte 0 | bit 6 subtype + build date present, bit 4 uptime present. Always 0x50 |
| 5 | 1 | device subtype | 0x01 = Breezedude transmitter |
| 6 | 2 | build date | bit 0-4 day, 5-8 month, 9-14 year - 2019, 15 develop build |
| 8 | 2 | uptime | minutes since boot, LE, wraps at 65536 |
| 10 | 1 | debug type | selects the payload that follows |
| 11 | n | debug payload | see below |

The forward bit is set when the station has `FORWARD_DATA` or `REPEATER`
switched on; a repeater clears it when it passes the packet on.

## Debug type 1 - dynamic (10 bytes)

Sent every 10 minutes, every 30 minutes while the battery is low.

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 2 | vbatt_mv | battery voltage, mV |
| 2 | 1 | batt_perc | state of charge, 0-100 |
| 3 | 1 | flags | bit 0 charging, bit 1 charge done, bit 2-3 awake duty class |
| 4 | 2 | rx_count | FANET packets received since boot |
| 6 | 2 | forward_count | FANET packets forwarded since boot |
| 8 | 1 | lbt_counter | listen-before-talk retries since boot |
| 9 | 1 | resets | bit 0-4 resets since power-on mod 32, bit 5-7 cause of the last reset |

**Awake duty class** (flags bit 2-3): share of the time the CPU was awake
since the previous dynamic report. 0 = below 1 %, 1 = below 5 %, 2 = below
20 %, 3 = 20 % and above. A healthy station reports 0.

**Reset cause** (resets bit 5-7):

| Code | Cause |
|---|---|
| 0 | power-on |
| 1 | watchdog |
| 2 | software reset (REBOOT, OTA, config update) |
| 3 | brown-out |
| 4 | external reset pin |
| 5 | several causes at once |
| 6 | unknown |

**Counting resets in the backend.** The count wraps at 32 instead of
saturating, so the backend keeps its own running total per station: when the
new count is lower than the previous one, add 32 unless the cause is 0
(power-on), which starts the count over at 0. A station that reports count
0 with cause 0 has been power cycled.

## Debug type 2 - OTA (10 bytes)

Sent once an hour while OTA is enabled. Unchanged by this revision.

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 2 | version_bcd | firmware major.minor.patch as nibbles, e.g. 0x120 = 1.2.0 |
| 2 | 4 | nonce | challenge for the OTA offer reply |
| 6 | 2 | slot_capacity_kb | accepted image size, KiB |
| 8 | 1 | ota_proto | OTA protocol version |
| 9 | 1 | ota_state | bit 0 active slot (0 = A, 1 = B) |

## Debug type 3 - static (7 bytes)

Sent once an hour.

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 1 | config bits | bit 0-4 sensor type, bit 5 barometer, bit 6 undervoltage seen, bit 7 LBT on |
| 1 | 1 | lora_rssi_threshold | LBT threshold, negated dBm (104 = -104 dBm) |
| 2 | 1 | sensor_integ_s | sensor integration time, s |
| 3 | 1 | reduce_interval_voltage | (volt * 100) - 200, e.g. 141 = 3.41 V |
| 4 | 1 | hw_rev | see below |
| 5 | 2 | bl_version | bootloader: bit 12-15 major, 6-11 minor, 0-5 patch; 0 = unknown |

Sensor type: 0 none, 1 WS80, 2 WS85, 3 WS85_UART, 4 DAVIS6410, 5 WINDNERD.

**hw_rev** says which pin map the firmware runs with, not which board was
soldered - two ID pins cannot name every revision, but a wrong pin map is
what breaks a station:

| Bits | Meaning |
|---|---|
| 0-3 | pin map: 0 = hardware up to 2.4, 1 = hardware 2.5 |
| 4 | pinned by the `HW_REVISION` setting |
| 5 | the ID pins said one thing, the battery divider another, and the divider won |
| 6-7 | raw ID pins, bit 7 = PA28, bit 6 = PA27 (legacy boards read 11) |

Example: `0xA1` = 2.5 pin map, overruled, pins read 10.
`bl_version` 0x4081 = v4.2.1.

## Debug type 4 - repeater (3 + 5 per station)

Sent every 15 minutes, only by stations with `REPEATER` on. The payload
length tells the decoder how many station entries follow:
`(length - 3) / 5`, at most 4 per packet. A repeater that follows more than
four stations continues with the remaining ones in its next report.

Header:

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 1 | flags | see below |
| 1 | 1 | rx_duty | per cent of the time the receiver was on since the previous report |
| 2 | 1 | forwarded | packets forwarded since the previous report, capped at 255 |

flags:

| Bits | Meaning |
|---|---|
| 0-1 | mode: 1 classic (receiver always on), 2 scheduled but listening continuously (battery full and charging), 3 scheduled |
| 2 | a scan window is open right now |
| 3-5 | stations in the table (0-5) |
| 6 | a station was dropped since the previous report to make room for another |
| 7 | receiver off because of undervoltage |

Station entry, one per followed station:

| Offset | Size | Field | Notes |
|---|---|---|---|
| 0 | 1 | vendor | FANET address of the followed station |
| 1 | 2 | address | LE |
| 3 | 1 | period_s | learned weather period, s; 0 while unknown; capped at 255 |
| 4 | 1 | status | bit 0-2 weather state, bit 3-4 empty windows in a row (capped at 3), bit 5-6 HWInfo state, bit 7 reserved |

States (`SchedState` in `src/fanet_sched.h`): 0 free (never sent), 1 heard
once and learning, 2 one period measured and being confirmed, 3 synced,
4 resync after two missed windows (weather only).

A classic repeater keeps the same table without acting on it, so its report
shows whether the rhythms of its stations are stable enough for `REPEATER_SCHED`.

## Cadence

| Type | Interval | First after boot |
|---|---|---|
| 1 dynamic | 10 min (30 min on low battery) | 1 min |
| 2 OTA | 60 min | 10 min (20 s with `OTA_FAST`) |
| 3 static | 60 min | 10 min |
| 4 repeater | 15 min | 5 min |

Only one debug payload goes out per HWInfo transmission; when several are
due, type 1 wins, then 2, 3, 4, and the rest follow with the next
transmissions.

## History

- 2026-10: type 1 `pv_state` became `flags` (awake duty class in bit 2-3),
  the reserved byte became `resets`; type 3 grew `hw_rev` and `bl_version`;
  type 4 added. No station in the field sent the old layout with a repeater,
  and the old type 1/3 bytes decode unchanged except for the two new
  meanings.
