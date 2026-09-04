// Host test for src/logging.cpp - the E|/W|/I|/D| line markers.
//
// The marker belongs to the line, not to the call, and almost every message in
// this firmware is assembled from several calls. That rule is easy to state and
// easy to break, and breaking it is nearly invisible on hardware: the output
// still scrolls past, just with the markers in the wrong places. So it is
// pinned down here, where the streams can be read back.

#include <stdio.h>
#include "logging.cpp"

// --- the globals logging.cpp expects ---------------------------------------
Uart Serial2;
Serial_ Serial;
int div_cpu = 1;
bool usb_connected = false;
HostSettings settings = { false };

static bool display_da = false;
static char display_text[512] = { 0 };
bool display_present() { return display_da; }
void display_add_line(String txt) {
  const size_t used = strlen(display_text);
  snprintf(display_text + used, sizeof(display_text) - used, "%s\n", txt.c_str());
}

// --- test scaffolding ------------------------------------------------------
static int fehler = 0;
static int geprueft = 0;

/** Macht CR und LF sichtbar, damit ein Unterschied im Zeilenende auffaellt. */
static const char *sichtbar(const char *s, char *ziel, size_t n) {
  size_t k = 0;
  for (; *s && k + 3 < n; s++) {
    if (*s == '\r')      { ziel[k++] = '\\'; ziel[k++] = 'r'; }
    else if (*s == '\n') { ziel[k++] = '\\'; ziel[k++] = 'n'; }
    else                 { ziel[k++] = *s; }
  }
  ziel[k] = '\0';
  return ziel;
}

static void pruefe(const char *was, const char *ist, const char *soll) {
  geprueft++;
  if (strcmp(ist, soll) != 0) {
    fehler++;
    char a[2048], b[2048];
    printf("FEHLER %s\n  ist : \"%s\"\n  soll: \"%s\"\n",
           was, sichtbar(ist, a, sizeof(a)), sichtbar(soll, b, sizeof(b)));
  }
}

/** Setzt beide Stroeme und den Zeilenzustand zurueck. */
static void neu(bool debug, bool errors, bool usb, bool verbose) {
  log_set_debug(true);
  log_set_error(true);
  usb_connected = true;
  settings.verbose_usb = false;
  // Der Zeilenzustand ist statisch in logging.cpp und von aussen nicht
  // erreichbar. Eine abgeschlossene Zeile setzt ihn zurueck - dafuer dieser
  // Aufruf, danach werden die Puffer geleert.
  log_i("\r\n");

  Serial2.clear();
  Serial.clear();
  display_text[0] = '\0';
  log_set_debug(debug);
  log_set_error(errors);
  usb_connected = usb;
  settings.verbose_usb = verbose;
}

#define DS Serial2.out
#define USB Serial.out

int main() {
  // Eine Marke je Zeile, auch wenn die Zeile aus mehreren Aufrufen entsteht.
  neu(true, true, false, false);
  log_i("SAVE OK ");
  log_write((uint32_t)5);
  log_i(" changes, REBOOT to apply\r\n");
  pruefe("mehrteilige Zeile", DS, "I| SAVE OK 5 changes, REBOOT to apply\r\n");

  // Eingebettete Zeilenumbrueche: jede Zeile bekommt ihre Marke, die leere
  // Trennzeile keine.
  neu(true, true, false, false);
  log_i("\r\n--------------- RESET -------------------\r\n");
  pruefe("Reset-Banner", DS, "\r\nI| --------------- RESET -------------------\r\n");

  // Die drei anderen Ebenen.
  neu(true, true, false, false);
  log_e("SF:write failed\r\n");
  log_w("FANET RX: CRC error\r\n");
  log_d("wind speed: ", (uint32_t)7);
  pruefe("E, W und D", DS,
         "E| SF:write failed\r\n"
         "W| FANET RX: CRC error\r\n"
         "D| wind speed: 7\r\n");

  // Der Kern der alten Verstuemmelung: log_e/log_w schreiben bei errors_en,
  // ihre Fortsetzungen hingen frueher an debug_en. Mit DEBUG=0 - der
  // Feldeinstellung - brach jede Fehlerzeile nach dem ersten Stueck ab.
  neu(false, true, false, false);
  log_w("Config update: invalid key_len=");
  log_write((uint32_t)200);
  log_write("\r\n");
  pruefe("Fortsetzung erbt die Ebene", DS, "W| Config update: invalid key_len=200\r\n");

  // Gegenprobe: gewoehnliche Ausgabe bleibt bei DEBUG=0 stumm.
  neu(false, true, false, false);
  log_i("Sending Weather\r\n");
  log_d("V_Bat: ", (uint32_t)3650);
  pruefe("DEBUG=0 bleibt stumm", DS, "");

  // Maschinenlesbare Nutzlast: keine Marke, sonst waere ein DUMP keine
  // wortgetreue Kopie der settings.txt mehr.
  neu(true, true, false, false);
  log_i("DUMP BEGIN\r\n");
  log_raw("NAME=Teststation\nALT=466\n");
  log_i("DUMP END\r\n");
  pruefe("DUMP-Rumpf ohne Marke", DS,
         "I| DUMP BEGIN\r\n"
         "NAME=Teststation\nALT=466\n"
         "I| DUMP END\r\n");

  // log_v traegt die D-Marke, geht ueber USB aber nur an den, der verbose will.
  neu(true, true, true, false);
  log_v("LBT: RSSI scan not supported\r\n");
  pruefe("log_v auf dem UART", DS, "D| LBT: RSSI scan not supported\r\n");
  pruefe("log_v nicht auf USB ohne verbose", USB, "");

  neu(true, true, true, true);
  log_v("LBT: RSSI scan not supported\r\n");
  pruefe("log_v auf USB mit verbose", USB, "D| LBT: RSSI scan not supported\r\n");

  // log_d geht dorthin, wo auch log_i hingeht - nur die Marke unterscheidet
  // sich. Das ist der Punkt: die Einstufung aendert nicht, welcher Port redet.
  neu(true, true, true, false);
  log_d("Wakeup detail\r\n");
  pruefe("log_d erreicht USB wie log_i", USB, "D| Wakeup detail\r\n");

  // log_s ist der schlichte Statuskanal fuer den nicht-verbosen USB-Leser und
  // liest sich als gewoehnliche Information.
  neu(true, true, true, false);
  log_s("OTA: ", (uint32_t)42, "%");
  pruefe("log_s traegt die I-Marke", USB, "I| OTA: 42%\r\n");
  pruefe("log_s geht nicht auf den UART", DS, "");

  // Jeder Strom fuehrt seinen eigenen Zeilenstand. Eine Zeile, die nur der
  // UART sieht, darf die Marke der naechsten USB-Zeile nicht verschlucken.
  neu(true, true, true, false);
  log_v("nur UART\r\n");
  log_i("beide\r\n");
  pruefe("getrennte Zeilenstaende, UART", DS, "D| nur UART\r\nI| beide\r\n");
  pruefe("getrennte Zeilenstaende, USB", USB, "I| beide\r\n");

  // Eine Zahl, die eine Zeile eroeffnet, gilt als Information.
  neu(true, true, false, false);
  log_write((uint32_t)123);
  log_write("\r\n");
  pruefe("Zahl eroeffnet eine I-Zeile", DS, "I| 123\r\n");

  // Der Hexdump-Pfad, wie tools.cpp ihn benutzt.
  neu(true, true, false, false);
  log_write_hex(0xAB, 2);
  log_d(" ");
  log_write_hex(0xCD, 2);
  log_write("\r\n");
  pruefe("Hex-Fortsetzung", DS, "I| AB CD\r\n");

  // Die Anzeige bekommt den Text ohne Marke.
  neu(true, true, false, false);
  display_da = true;
  log_w("Baro: not found\r\n");
  pruefe("Anzeige ohne Marke", display_text, "Baro: not found\r\n\n");
  display_da = false;

  printf("%d Pruefungen, %d Fehler\n", geprueft, fehler);
  return fehler ? 1 : 0;
}
