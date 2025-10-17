#pragma once
#include <Arduino.h>
#include "AquaProbe.h"

// Optional raw Modbus read callback
typedef bool (*RawReadFn)(uint16_t addr, uint16_t count, uint16_t* out);

class CommandHandler {
public:
  enum OutFmt : uint8_t { FMT_JSON=0, FMT_CSV=1 };

  struct Config {
    bool     continuous = false;
    OutFmt   fmt        = FMT_JSON;
    uint32_t intervalMs = 1000;
  };

  CommandHandler(AquaProbe& probe, Print& io);

  void setRawReadCallback(RawReadFn fn) { _rawRead = fn; }

  void begin(bool banner=true, bool printHelpMsg=true);
  void printHelp();

  // Input handling
  void feedChar(char c);
  void poll(Stream& in);  // convenience to read from Serial
  void tick();            // continuous mode

  Config& config() { return _cfg; }

private:
  AquaProbe&  _probe;
  Print&      _io;
  Config      _cfg;
  RawReadFn   _rawRead = nullptr;

  static constexpr size_t BUF_LEN = 160;
  char   _buf[BUF_LEN];
  size_t _len = 0;
  uint32_t _nextTick = 0;
  bool _csvHeaderPrinted = false;

  void _printf(const char* fmt, ...);
  static void _upcase(char* s);
  void _handleLine(char* line);

  void _cmdGET(char* args);
  void _cmdREAD(char* args);
  void _cmdSET(char* args);
  void _cmdSTART(char* args);
  void _cmdSTOP();

  void _printAllJSON(const AquaProbe::All& a);
  void _printAllCSVHeader();
  void _printAllCSV(const AquaProbe::All& a);
  bool _readAllAndPrint();
};
