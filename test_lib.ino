#include "AquaProbe.h"

AquaProbe probe;

const int DE_RE = 21;
const int RXD   = 18;
const int TXD   = 17;

void setup() {
  Serial.begin(115200);
  // Device on address 1, RS485 on Serial1, 9600 8N1, ESP32 pins RX=18, TX=17
  probe.begin(Serial1, /*slave*/1, DE_RE, 38400, SERIAL_8N1, RXD, TXD);
}

void loop() {
  AquaProbe::All all;
  if (probe.readAllFast(all)) {
    Serial.printf("pH: %.2f  Temp: %.1f °C  pH mV: %.0f  ORP: %.0f mV\n",
                  all.ph, all.tempC, all.ph_mV, all.orp_mV);
  } else {
    Serial.printf("Modbus error: %u\n", probe.lastError());
  }
  delay(1000);
}
