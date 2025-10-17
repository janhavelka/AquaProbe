#include "CommandHandler.h"
#include <stdarg.h>

CommandHandler::CommandHandler(AquaProbe& probe, Print& io)
: _probe(probe), _io(io) {}

void CommandHandler::begin(bool banner, bool printHelpMsg) {
  if (banner) _io.println(F("# CommandHandler ready. Type HELP"));
  if (printHelpMsg) printHelp();
}

void CommandHandler::_printf(const char* fmt, ...) {
  char tmp[256];
  va_list ap; va_start(ap, fmt);
  vsnprintf(tmp, sizeof(tmp), fmt, ap);  // <-- include fmt as 3rd arg
  va_end(ap);
  _io.print(tmp);
}

void CommandHandler::feedChar(char c) {
  if (c == '\r') return;
  if (c == '\n') {
    _buf[_len] = 0;
    if (_len) _handleLine(_buf);
    _len = 0;
  } else if (_len < BUF_LEN-1) {
    _buf[_len++] = c;
  }
}

void CommandHandler::poll(Stream& in) {
  while (in.available()) feedChar((char)in.read());
}

void CommandHandler::tick() {
  uint32_t now = millis();
  if (_cfg.continuous && (now >= _nextTick)) {
    _readAllAndPrint();
    _nextTick = now + _cfg.intervalMs;
  }
}

void CommandHandler::printHelp() {
  _io.println(
    F("Commands:\n"
      "  HELP                       - this help\n"
      "  PING                       - respond OK\n"
      "  GET ALL                    - read all values once\n"
      "  GET PH|TEMP|PHMV|ORP       - single value\n"
      "  GET META                   - decimals & unit codes\n"
      "  GET RAW                    - raw 0x2001..0x2008 (needs raw cb)\n"
      "  READ REG <hex> [count]     - read holding regs (needs raw cb)\n"
      "  SET TEMP <degC>            - write temperature (if supported)\n"
      "  SET SLAVE <id>             - change Modbus unit id (master-side)\n"
      "  SET RATE <ms>              - interval for continuous mode\n"
      "  SET FORMAT JSON|CSV        - output format\n"
      "  START [JSON|CSV]           - start continuous GET ALL\n"
      "  STOP                       - stop continuous mode\n"));
}

void CommandHandler::_upcase(char* s){ for(;*s;++s) *s = toupper(*s); }

void CommandHandler::_handleLine(char* line) {
  while (*line==' ' || *line=='\t') ++line;
  if (!*line) return;

  char* save=nullptr;
  char* head = strtok_r(line, " \t", &save);
  char* rest = strtok_r(nullptr, "", &save);
  if (!head) return;

  char cmd[16]; strncpy(cmd, head, sizeof(cmd)); cmd[sizeof(cmd)-1]=0; _upcase(cmd);

  if (!strcmp(cmd,"HELP") || !strcmp(cmd,"?"))          printHelp();
  else if (!strcmp(cmd,"PING"))                         _io.println(F("OK"));
  else if (!strcmp(cmd,"GET"))                          _cmdGET(rest);
  else if (!strcmp(cmd,"READ"))                         _cmdREAD(rest);
  else if (!strcmp(cmd,"SET"))                          _cmdSET(rest);
  else if (!strcmp(cmd,"START"))                        _cmdSTART(rest);
  else if (!strcmp(cmd,"STOP"))                         _cmdSTOP();
  else                                                  _io.println(F("ERR unknown"));
}

void CommandHandler::_cmdGET(char* args) {
  if (!args) { _io.println(F("ERR usage")); return; }
  char* save=nullptr;
  char* tok=strtok_r(args," \t",&save);
  if (!tok){ _io.println(F("ERR usage")); return; }

  char key[12]; strncpy(key,tok,sizeof(key)); key[sizeof(key)-1]=0; _upcase(key);

  if (!strcmp(key,"ALL")) {
    _readAllAndPrint();
  } else if (!strcmp(key,"PH")) {
    float v; if (_probe.readPH(v)) _printf("%.2f\n", v);
    else _printf("ERR %u\n", _probe.lastError());
  } else if (!strcmp(key,"TEMP") || !strcmp(key,"TC") || !strcmp(key,"T")) {
    float v; if (_probe.readTemperatureC(v)) _printf("%.1f\n", v);
    else _printf("ERR %u\n", _probe.lastError());
  } else if (!strcmp(key,"PHMV")) {
    float v; if (_probe.readPHmV(v)) _printf("%.0f\n", v);
    else _printf("ERR %u\n", _probe.lastError());
  } else if (!strcmp(key,"ORP")) {
    float v; if (_probe.readORPmV(v)) _printf("%.0f\n", v);
    else _printf("ERR %u\n", _probe.lastError());
  } else if (!strcmp(key,"META")) {
    uint8_t d,u;
    _probe.readPHMeta(d,u);   _printf("PH: dec=%u unit=%u\n", d,u);
    _probe.readTempMeta(d,u); _printf("TEMP: dec=%u unit=%u\n", d,u);
    _probe.readMVMeta(d,u);   _printf("PH mV: dec=%u unit=%u\n", d,u);
    _probe.readORPMeta(d,u);  _printf("ORP: dec=%u unit=%u\n", d,u);
  } else if (!strcmp(key,"RAW")) {
    if (!_rawRead) { _io.println(F("ERR no raw reader")); return; }
    const uint16_t base=0x2001, count=8;
    uint16_t tmp[count];
    if (_rawRead(base,count,tmp)) {
      _io.println(F("ADDR,HEX,DEC,SIGNED"));
      for (uint16_t i=0;i<count;i++) {
        _printf("0x%04X,0x%04X,%u,%d\n", base+i, tmp[i], tmp[i], (int16_t)tmp[i]);
      }
    } else {
      _io.println(F("ERR raw"));
    }
  } else {
    _io.println(F("ERR unknown GET"));
  }
}

void CommandHandler::_cmdREAD(char* args) {
  if (!_rawRead) { _io.println(F("ERR no raw reader")); return; }
  if (!args) { _io.println(F("ERR usage")); return; }
  char* save=nullptr;
  char* a=strtok_r(args," \t",&save);
  char* b=strtok_r(nullptr," \t",&save);
  if (!a){ _io.println(F("ERR usage")); return; }

  uint16_t addr = (uint16_t)strtoul(a, nullptr, 0);
  uint16_t count= b ? (uint16_t)strtoul(b,nullptr,0) : 1;
  if (!count || count>64){ _io.println(F("ERR count")); return; }

  _io.println(F("ADDR,HEX,DEC,SIGNED"));
  const uint16_t CH = 16;
  uint16_t buf[CH];
  uint16_t done=0;
  while (done < count) {
    uint16_t n = min<uint16_t>(CH, count-done);
    if (!_rawRead(addr+done, n, buf)) { _io.println(F("ERR read")); return; }
    for (uint16_t i=0;i<n;i++) {
      uint16_t v=buf[i];
      _printf("0x%04X,0x%04X,%u,%d\n", addr+done+i, v, v, (int16_t)v);
    }
    done += n;
  }
}

void CommandHandler::_cmdSET(char* args) {
  if (!args) { _io.println(F("ERR usage")); return; }
  char* save=nullptr;
  char* k=strtok_r(args," \t",&save);
  if (!k){ _io.println(F("ERR usage")); return; }

  char key[14]; strncpy(key,k,sizeof(key)); key[sizeof(key)-1]=0; _upcase(key);

  if (!strcmp(key,"TEMP")) {
    char* v=strtok_r(nullptr," \t",&save);
    if (!v){ _io.println(F("ERR usage")); return; }
    float t = atof(v);
    if (_probe.writeTemperatureC(t)) _io.println(F("OK"));
    else _printf("ERR %u\n", _probe.lastError());

  } else if (!strcmp(key,"SLAVE")) {
    char* v=strtok_r(nullptr," \t",&save);
    if (!v){ _io.println(F("ERR usage")); return; }
    int id = atoi(v);
    if (id<1 || id>247) { _io.println(F("ERR id")); return; }
    _probe.setSlave((uint8_t)id);
    _io.println(F("OK"));

  } else if (!strcmp(key,"RATE")) {
    char* v=strtok_r(nullptr," \t",&save);
    if (!v){ _io.println(F("ERR usage")); return; }
    long ms = atol(v);
    if (ms<100) ms=100;
    _cfg.intervalMs = (uint32_t)ms;
    _io.println(F("OK"));

  } else if (!strcmp(key,"FORMAT")) {
    char* v=strtok_r(nullptr," \t",&save);
    if (!v){ _io.println(F("ERR usage")); return; }
    _upcase(v);
    if (!strcmp(v,"JSON")) { _cfg.fmt = FMT_JSON; _io.println(F("OK")); }
    else if (!strcmp(v,"CSV")) { _cfg.fmt = FMT_CSV; _csvHeaderPrinted=false; _io.println(F("OK")); }
    else _io.println(F("ERR format"));

  } else {
    _io.println(F("ERR unknown SET"));
  }
}

void CommandHandler::_cmdSTART(char* args) {
  if (args) {
    char f[8]; strncpy(f,args,sizeof(f)); f[sizeof(f)-1]=0; _upcase(f);
    if (!strcmp(f,"CSV")) { _cfg.fmt=FMT_CSV; _csvHeaderPrinted=false; }
    else if (!strcmp(f,"JSON")) { _cfg.fmt=FMT_JSON; }
  }
  _cfg.continuous = true;
  _nextTick = 0;
  _io.println(F("OK CONT"));
}

void CommandHandler::_cmdSTOP() {
  _cfg.continuous = false;
  _io.println(F("OK STOP"));
}

void CommandHandler::_printAllJSON(const AquaProbe::All& a) {
  _printf("{\"ph\":%.2f,\"temp_c\":%.1f,\"ph_mv\":%.0f,\"orp_mv\":%.0f,"
          "\"meta\":{\"ph_dec\":%u,\"t_dec\":%u,\"mv_dec\":%u,\"orp_dec\":%u,"
          "\"ph_unit\":%u,\"t_unit\":%u,\"mv_unit\":%u,\"orp_unit\":%u},"
          "\"ms\":%lu}\n",
          a.ph, a.tempC, a.ph_mV, a.orp_mV,
          a.ph_dec, a.t_dec, a.mv_dec, a.orp_dec,
          a.ph_unit, a.t_unit, a.mv_unit, a.orp_unit,
          (unsigned long)millis());
}

void CommandHandler::_printAllCSVHeader() {
  _io.println(F("ms,ph,temp_c,ph_mv,orp_mv,ph_dec,t_dec,mv_dec,orp_dec,ph_unit,t_unit,mv_unit,orp_unit"));
}

void CommandHandler::_printAllCSV(const AquaProbe::All& a) {
  if (!_csvHeaderPrinted) { _printAllCSVHeader(); _csvHeaderPrinted=true; }
  _printf("%lu,%.2f,%.1f,%.0f,%.0f,%u,%u,%u,%u,%u,%u,%u,%u\n",
          (unsigned long)millis(), a.ph, a.tempC, a.ph_mV, a.orp_mV,
          a.ph_dec, a.t_dec, a.mv_dec, a.orp_dec,
          a.ph_unit, a.t_unit, a.mv_unit, a.orp_unit);
}

bool CommandHandler::_readAllAndPrint() {
  AquaProbe::All a;
  if (!_probe.readAllFast(a)) {
    _printf("ERR %u\n", _probe.lastError());
    return false;
  }
  if (_cfg.fmt==FMT_JSON) _printAllJSON(a); else _printAllCSV(a);
  return true;
}
