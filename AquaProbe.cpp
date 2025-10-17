#include "AquaProbe.h"
#include <math.h>

int AquaProbe::s_deRePin = -1;

AquaProbe::AquaProbe() {}

bool AquaProbe::begin(HardwareSerial& port, uint8_t slave, int deRePin,
                      uint32_t baud, uint32_t config, int rxPin, int txPin) {
  _ser = &port;
  _slave = slave;
  _deRePin = deRePin;
  s_deRePin = deRePin;

  if (_deRePin >= 0) {
    pinMode(_deRePin, OUTPUT);
    digitalWrite(_deRePin, LOW); // RX by default
  }

  if (rxPin >= 0 && txPin >= 0) {
    _ser->begin(baud, config, rxPin, txPin);
  } else {
    _ser->begin(baud, config);
  }

  _mb.begin(_slave, *_ser);
  _mb.preTransmission(&AquaProbe::preTxStatic);
  _mb.postTransmission(&AquaProbe::postTxStatic);
  return true;
}

void AquaProbe::setSlave(uint8_t slave) {
  _slave = slave;
  _mb.begin(_slave, *_ser);
}

void AquaProbe::preTxStatic()  { if (s_deRePin >= 0) digitalWrite(s_deRePin, HIGH); }
void AquaProbe::postTxStatic() { if (s_deRePin >= 0) digitalWrite(s_deRePin, LOW);  }

bool AquaProbe::readU16(uint16_t addr, uint16_t& out) {
  _lastErr = _mb.readHoldingRegisters(addr, 1);
  if (_lastErr != _mb.ku8MBSuccess) return false;
  out = _mb.getResponseBuffer(0);
  _mb.clearResponseBuffer();
  return true;
}

bool AquaProbe::readI16(uint16_t addr, int16_t& out) {
  uint16_t u;
  if (!readU16(addr, u)) return false;
  out = (int16_t)u;
  return true;
}

bool AquaProbe::readMeta(uint16_t addr, uint8_t& decimals, uint8_t& unit) {
  uint16_t reg;
  if (!readU16(addr, reg)) return false;
  decimals = (uint8_t)(reg >> 8);  // HI byte
  unit     = (uint8_t)(reg & 0xFF);// LO byte
  return true;
}

bool AquaProbe::writeU16(uint16_t addr, uint16_t val) {
  _lastErr = _mb.writeSingleRegister(addr, val);
  return (_lastErr == _mb.ku8MBSuccess);
}

// ---- High-level reads ----
bool AquaProbe::readPH(float& ph) {
  uint16_t raw; if (!readU16(REG_PH, raw)) return false;
  uint8_t dec=2, unit=0; readMeta(REG_PH_META, dec, unit); // best-effort
  if (dec == 0 || dec > 4) dec = 2;
  ph = raw / powf(10.0f, dec);
  return true;
}

bool AquaProbe::readTemperatureC(float& tC) {
  int16_t raw; if (!readI16(REG_TEMP, raw)) return false;
  uint8_t dec=1, unit=0; readMeta(REG_TEMP_META, dec, unit);
  if (dec == 0 || dec > 3) dec = 1;
  tC = raw / powf(10.0f, dec);
  return true;
}

bool AquaProbe::readPHmV(float& mv) {
  int16_t raw; if (!readI16(REG_PHMV, raw)) return false;
  uint8_t dec=0, unit=0; readMeta(REG_MV_META, dec, unit);
  mv = raw / powf(10.0f, dec);
  return true;
}

bool AquaProbe::readORPmV(float& mv) {
  int16_t raw; if (!readI16(REG_ORP, raw)) return false;
  uint8_t dec=0, unit=0; readMeta(REG_ORP_META, dec, unit);
  mv = raw / powf(10.0f, dec);
  return true;
}

bool AquaProbe::readPHMeta(uint8_t& d, uint8_t& u)   { return readMeta(REG_PH_META, d, u); }
bool AquaProbe::readTempMeta(uint8_t& d, uint8_t& u) { return readMeta(REG_TEMP_META, d, u); }
bool AquaProbe::readMVMeta(uint8_t& d, uint8_t& u)   { return readMeta(REG_MV_META, d, u); }
bool AquaProbe::readORPMeta(uint8_t& d, uint8_t& u)  { return readMeta(REG_ORP_META, d, u); }

bool AquaProbe::readAllFast(All& out) {
  _lastErr = _mb.readHoldingRegisters(REG_PH, 8); // 0x2001..0x2008
  if (_lastErr != _mb.ku8MBSuccess) return false;

  auto U = [&](int i){ uint16_t v=_mb.getResponseBuffer(i); return v; };
  auto S = [&](int i){ return (int16_t)_mb.getResponseBuffer(i); };

  uint16_t phRaw   = U(0);
  uint16_t phMeta  = U(1);
  int16_t  tRaw    = S(2);
  uint16_t tMeta   = U(3);
  int16_t  phmRaw  = S(4);
  uint16_t mvMeta  = U(5);
  int16_t  orpRaw  = S(6);
  uint16_t orpMeta = U(7);
  _mb.clearResponseBuffer();

  out.ph_dec  = (phMeta  >> 8) & 0xFF; out.ph_unit  = phMeta  & 0xFF;
  out.t_dec   = (tMeta   >> 8) & 0xFF; out.t_unit   = tMeta   & 0xFF;
  out.mv_dec  = (mvMeta  >> 8) & 0xFF; out.mv_unit  = mvMeta  & 0xFF;
  out.orp_dec = (orpMeta >> 8) & 0xFF; out.orp_unit = orpMeta & 0xFF;

  if (out.ph_dec == 0 || out.ph_dec > 4) out.ph_dec = 2;
  if (out.t_dec  == 0 || out.t_dec  > 3) out.t_dec  = 1;

  out.ph     = phRaw / powf(10.0f, out.ph_dec);
  out.tempC  = tRaw   / powf(10.0f, out.t_dec);
  out.ph_mV  = phmRaw / powf(10.0f, out.mv_dec);
  out.orp_mV = orpRaw / powf(10.0f, out.orp_dec);

  return true;
}

bool AquaProbe::writeTemperatureC(float tC) {
  // Get decimals so we quantize properly
  uint8_t dec=1, unit=0;
  readMeta(REG_TEMP_META, dec, unit);
  if (dec == 0 || dec > 3) dec = 1;

  int32_t raw = lroundf(tC * powf(10.0f, dec));
  if (raw < -32768 || raw > 32767) return false; // out of range
  return writeU16(REG_TEMP, (uint16_t)(int16_t)raw);
}
