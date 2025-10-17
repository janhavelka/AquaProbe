#include <ModbusMaster.h>
#include "AquaProbe.h"
#include "CommandHandler.h"

// RS485 wiring (adjust for your board)
const int DE_RE = 21;   // DE+RE tied together
const int RXD   = 18;
const int TXD   = 17;
const uint32_t BAUD = 38400;

AquaProbe       probe;
CommandHandler  cmd(probe, Serial);

// DE/RE control for the raw reader
void preTx(){ digitalWrite(DE_RE, HIGH); }
void postTx(){ digitalWrite(DE_RE, LOW);  }

// Raw holding-register reader for CommandHandler (optional but enables GET RAW / READ REG)
ModbusMaster mb;
bool rawHolding(uint16_t addr, uint16_t count, uint16_t* out) {
  mb.begin(probe.slave(), Serial1);
  mb.preTransmission(preTx);
  mb.postTransmission(postTx);
  uint8_t ec = mb.readHoldingRegisters(addr, count);
  if (ec != mb.ku8MBSuccess) return false;
  for (uint16_t i=0;i<count;i++) out[i] = mb.getResponseBuffer(i);
  mb.clearResponseBuffer();
  return true;
}

void setup() {
  pinMode(DE_RE, OUTPUT);
  postTx();

  Serial.begin(115200);
  Serial1.begin(BAUD, SERIAL_8N1, RXD, TXD);

  // Device on Modbus address 1
  probe.begin(Serial1, 1, DE_RE, BAUD, SERIAL_8N1, RXD, TXD);

  cmd.setRawReadCallback(rawHolding);
  cmd.begin(true, false);   // banner only
  Serial.println(F("Type HELP"));
}

void loop() {
  cmd.poll(Serial);  // read commands
  cmd.tick();        // continuous mode
}
