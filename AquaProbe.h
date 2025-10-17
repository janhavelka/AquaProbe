#pragma once
#include <Arduino.h>
#include <ModbusMaster.h>

/**
 * AquaProbe — Modbus RTU (RS485) helper for your pH/ORP/Temp monitor.
 *
 * Registers (all short = 16-bit):
 * 0x2001 pH raw (unsigned);        0x2002 pH [HI=decimals][LO=unit]
 * 0x2003 Temperature raw (signed); 0x2004 Temp [HI=decimals][LO=unit]
 * 0x2005 pH probe mV (signed);     0x2006 mV [HI=decimals][LO=unit]
 * 0x2007 ORP mV (signed);          0x2008 mV [HI=decimals][LO=unit]
 *
 * Notes:
 * - “Decimals” is in the **high byte** of the *Decimals & units* registers.
 * - Temperature and all mV values are **signed**.
 * - Temperature allows write via function 0x06 (if enabled on the device).
 */
class AquaProbe {
public:
  struct All {
    float ph = NAN;
    float tempC = NAN;
    float ph_mV = NAN;
    float orp_mV = NAN;
    uint8_t ph_dec = 2, t_dec = 1, mv_dec = 0, orp_dec = 0;
    uint8_t ph_unit = 0, t_unit = 0, mv_unit = 0, orp_unit = 0;
  };

  // Create but don’t start yet.
  AquaProbe();

  /**
   * Begin and configure the bus.
   * @param port   HardwareSerial to use (Serial1/Serial2…)
   * @param slave  Modbus unit ID (default 1)
   * @param deRePin GPIO controlling DE+RE (HIGH=TX, LOW=RX). Pass -1 to skip.
   * @param baud, config, rxPin, txPin Serial settings (ESP32 supports custom pins)
   */
  bool begin(HardwareSerial& port, uint8_t slave = 1, int deRePin = -1,
             uint32_t baud = 9600, uint32_t config = SERIAL_8N1,
             int rxPin = -1, int txPin = -1);

  // Change slave ID later if needed
  void setSlave(uint8_t slave);
  uint8_t  slave() const { return _slave; }   // <-- ADD THIS LINE

  // --- High-level reads (scaled) ---
  bool readPH(float& ph);                 // 0.00 … 14.00
  bool readTemperatureC(float& tC);       // -10.0 … 130.0
  bool readPHmV(float& mv);               // -500 … 500 mV
  bool readORPmV(float& mv);              // -2000 … 2000 mV

  // Read everything in one Modbus transaction (fast).
  bool readAllFast(All& out);

  // --- Meta (decimals & units) ---
  bool readPHMeta(uint8_t& decimals, uint8_t& unit);
  bool readTempMeta(uint8_t& decimals, uint8_t& unit);
  bool readMVMeta(uint8_t& decimals, uint8_t& unit);
  bool readORPMeta(uint8_t& decimals, uint8_t& unit);

  // Optional: write temperature setpoint (if device allows fn 0x06)
  bool writeTemperatureC(float tC);

  // Last Modbus error/status code (0 = success)
  uint8_t lastError() const { return _lastErr; }

private:
  // Register map
  static constexpr uint16_t REG_PH        = 0x2001;
  static constexpr uint16_t REG_PH_META   = 0x2002;
  static constexpr uint16_t REG_TEMP      = 0x2003;
  static constexpr uint16_t REG_TEMP_META = 0x2004;
  static constexpr uint16_t REG_PHMV      = 0x2005;
  static constexpr uint16_t REG_MV_META   = 0x2006;
  static constexpr uint16_t REG_ORP       = 0x2007;
  static constexpr uint16_t REG_ORP_META  = 0x2008;

  // Modbus helpers
  bool readU16(uint16_t addr, uint16_t& out);
  bool readI16(uint16_t addr, int16_t& out);
  bool readMeta(uint16_t addr, uint8_t& decimals, uint8_t& unit);
  bool writeU16(uint16_t addr, uint16_t val);

  // DE/RE control hooks for ModbusMaster (static — single instance support)
  static void preTxStatic();
  static void postTxStatic();
  static int  s_deRePin;

  // Internals
  ModbusMaster _mb;
  HardwareSerial* _ser = nullptr;
  uint8_t _slave = 1;
  int _deRePin = -1;
  uint8_t _lastErr = 0;
};
