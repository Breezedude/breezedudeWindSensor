#pragma once

// Function test after manufacturing, driven by tools/breezedude-selftest.html on
// install.breezedude.de. It needs no settings file: everything it touches is set
// up here, from the state setup() leaves a blank board in.
//
//   SELFTEST=1       everything, with a WS85 on the sensor UART
//   SELFTEST=NOWS85  the same without the sensor
//   SELFTEST=FAIL    what only the page can tell: this board did not pass after
//                    all, an outdated firmware for example. Red LED.
//
// While it runs the LEDs blink red and blue in turn. Afterwards the blue one stays
// on if all went well, otherwise the red one. The normal operation of the station
// does not resume: it restarts when the USB host goes away, or on REBOOT.
//
// Each result is one line, "ST " followed by a JSON object, all numbers integers:
//
//   ST {"begin":1,"fw":"1.2.1","bl":"v4.2.0","hw":"2.5","id":"BD1234","uuid":"...",...}
//   ST {"t":"baro","ok":1,"chip":"HP203B","p_pa":96512,"t_dc":231}
//   ST {"end":1,"ok":1,"failed":0}
void selftest_request(const char *arg);

// Call first thing in loop(). True while the test owns the station, then the
// rest of loop() has to stay away from it.
bool selftest_poll();
