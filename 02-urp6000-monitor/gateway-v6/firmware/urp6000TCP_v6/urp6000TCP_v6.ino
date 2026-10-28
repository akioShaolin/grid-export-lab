#include "site_private.h"
/*
  URP6000 Datalogger de producao - ESP8266
  Modbus RTU master -> espelho Holding Registers -> Modbus TCP server

  Base: urp6000TCP_v4 + diagnosticos do v5 + mDNS + manutencao remota.

  Bibliotecas:
    - ESP8266 Arduino Core
    - modbus-esp8266, de Alexander Emelianov

  Acesso:
    - Modbus TCP: urp6000.local:502
    - Console:     http://urp6000.local/
    - Atualizacao web: http://urp6000.local/update
    - AP fallback: URP6000-XXXXXXXX / 192.168.4.1

  IMPORTANTE:
    - A UART principal e exclusiva do RS485. Nao use Serial.print().
    - Antes de instalar, altere as senhas padrao em loadDefaultConfig().
    - O espelho preserva os enderecos originais do URP6000.

  Enderecos espelhados:
    0x003C..0x003E  relacoes
    0x0088..0x0089  identificacao
    0x02BC..0x02C3  correntes/tensoes
    0x02C7..0x02CB  frequencias/cos phi
    0x02CD..0x02D2  potencias ativas

  Diagnostico:
    0x0100 = dados validos (0/1)
    0x0101 = sequencia do snapshot
    0x0102 = idade dos dados, segundos
    0x0103 = flags de estado
    0x0107 = RSSI + 200  (ex.: 135 -> -65 dBm)
    0x0108..0x010B = IP STA
    0x0114 = heap livre / 32
    0x0115 = menor heap / 32
    0x011D..0x0120 = contadores RTU OK/erro, 32 bits
    0x0121 = ultimo erro RTU
    0x0123 = recuperacoes completas do RTU

  Snapshot atomico continuo:
    0x0200..0x021F (32 words)
      [00] flags
      [01] sequencia
      [02] idade_s
      [03..05] 0x003C..0x003E
      [06..13] 0x02BC..0x02C3
      [14..18] 0x02C7..0x02CB
      [19..24] 0x02CD..0x02D2
      [25..26] 0x0088..0x0089
      [27] RTU OK LSW
      [28] RTU erro LSW
      [29] RSSI + 200
      [30] heap / 32
      [31] ultimo erro RTU
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <EEPROM.h>
#include <Updater.h>
#include <ModbusRTU.h>
#include <ModbusIP_ESP8266.h>
#include <new>
#include <stddef.h>

// Tipos usados em assinaturas de funcoes. Devem ficar antes da primeira funcao
// para evitar erro do gerador automatico de prototipos da Arduino IDE.
enum RtuState : uint8_t { RTU_WAIT_ID = 0, RTU_CYCLIC = 1 };
enum CyStep : uint8_t { CY_IDCHECK = 0, CY_RAT, CY_MEAS, CY_FQ, CY_P };

// -----------------------------------------------------------------------------
// Firmware
// -----------------------------------------------------------------------------
static const char FW_NAME[]    = "URP6000-TCP";
static const char FW_VERSION[] = "1.0.1";
static const uint16_t FW_VERSION_WORD = 0x0101;  // 1.0.1

// -----------------------------------------------------------------------------
// Hardware
// -----------------------------------------------------------------------------
#define LED_PIN LED_BUILTIN
static const uint8_t PIN_RS485_DE_RE = 12;   // GPIO12 / D6
static const uint8_t HALF_OR_FULL_PIN = 13;  // GPIO13 / D7
static const bool USE_HALF_DUPLEX_SELECTOR = true;

// -----------------------------------------------------------------------------
// Configuracao persistente
// -----------------------------------------------------------------------------
static const uint32_t CONFIG_MAGIC = 0x55525036UL; // "URP6"
static const uint16_t CONFIG_VERSION = 1;
static const size_t EEPROM_SIZE = 512;

struct DeviceConfig {
  uint32_t magic;
  uint16_t version;
  char ssid[33];
  char wifiPass[65];
  char hostname[33];
  char adminPass[33];
  char apPass[33];
  uint32_t rtuBaud;
  uint8_t slaveId;
  uint8_t reserved[3];
  uint32_t crc;
};

DeviceConfig cfg;

static uint32_t hashBytes(const uint8_t* data, size_t len) {
  uint32_t h = 2166136261UL;
  for (size_t i = 0; i < len; ++i) {
    h ^= data[i];
    h *= 16777619UL;
  }
  return h;
}

static void copyText(char* dst, size_t dstSize, const String& src) {
  if (dstSize == 0) return;
  size_t n = src.length();
  if (n >= dstSize) n = dstSize - 1;
  memcpy(dst, src.c_str(), n);
  dst[n] = '\0';
}

static bool validHostname(const char* s) {
  size_t n = strlen(s);
  if (n < 1 || n > 32 || s[0] == '-' || s[n - 1] == '-') return false;
  for (size_t i = 0; i < n; ++i) {
    char c = s[i];
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') || c == '-')) return false;
  }
  return true;
}

static bool validBaud(uint32_t baud) {
  return baud == 1200 || baud == 2400 || baud == 4800 || baud == 9600 ||
         baud == 19200 || baud == 38400 || baud == 57600 || baud == 115200;
}

static void loadDefaultConfig() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic = CONFIG_MAGIC;
  cfg.version = CONFIG_VERSION;

  // Padroes atuais. Podem ser alterados depois em /config sem regravar.
  copyText(cfg.ssid, sizeof(cfg.ssid), SITE_WIFI_SSID);
  copyText(cfg.wifiPass, sizeof(cfg.wifiPass), SITE_WIFI_PASSWORD);
  copyText(cfg.hostname, sizeof(cfg.hostname), SITE_MDNS_HOST);

  // TROQUE estas duas senhas antes da instalacao definitiva.
  copyText(cfg.adminPass, sizeof(cfg.adminPass), SITE_ADMIN_PASSWORD);
  copyText(cfg.apPass, sizeof(cfg.apPass), SITE_AP_PASSWORD);

  cfg.rtuBaud = 115200;
  cfg.slaveId = 1;
}

static void saveConfig() {
  cfg.magic = CONFIG_MAGIC;
  cfg.version = CONFIG_VERSION;
  cfg.crc = hashBytes(reinterpret_cast<const uint8_t*>(&cfg), offsetof(DeviceConfig, crc));
  EEPROM.put(0, cfg);
  EEPROM.commit();
}

static bool loadConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, cfg);

  // Garante limites mesmo se a EEPROM contiver lixo sem terminador.
  cfg.ssid[sizeof(cfg.ssid) - 1] = '\0';
  cfg.wifiPass[sizeof(cfg.wifiPass) - 1] = '\0';
  cfg.hostname[sizeof(cfg.hostname) - 1] = '\0';
  cfg.adminPass[sizeof(cfg.adminPass) - 1] = '\0';
  cfg.apPass[sizeof(cfg.apPass) - 1] = '\0';

  uint32_t expected = hashBytes(reinterpret_cast<const uint8_t*>(&cfg), offsetof(DeviceConfig, crc));
  bool ok = cfg.magic == CONFIG_MAGIC && cfg.version == CONFIG_VERSION &&
            cfg.crc == expected && strlen(cfg.ssid) <= 32 &&
            validHostname(cfg.hostname) && strlen(cfg.adminPass) >= 8 &&
            strlen(cfg.apPass) >= 8 && validBaud(cfg.rtuBaud) &&
            cfg.slaveId >= 1 && cfg.slaveId <= 247;

  if (!ok) {
    loadDefaultConfig();
    saveConfig();
  }
  return ok;
}

// -----------------------------------------------------------------------------
// Estatisticas em RTC RAM: sobrevivem a ESP.restart(), sem desgaste da flash.
// -----------------------------------------------------------------------------
static const uint32_t RTC_MAGIC = 0x52544336UL;
struct RtcStats {
  uint32_t magic;
  uint32_t bootCount;
  uint32_t plannedRestarts;
  uint32_t lastRestartReason;
  uint32_t crc;
};
static_assert((sizeof(RtcStats) % 4) == 0, "RtcStats deve ser multiplo de 4");

RtcStats rtcStats;
String bootResetReason;

static void saveRtcStats() {
  rtcStats.magic = RTC_MAGIC;
  rtcStats.crc = hashBytes(reinterpret_cast<const uint8_t*>(&rtcStats), offsetof(RtcStats, crc));
  ESP.rtcUserMemoryWrite(0, reinterpret_cast<uint32_t*>(&rtcStats), sizeof(rtcStats));
}

static void loadRtcStats() {
  bool readOk = ESP.rtcUserMemoryRead(0, reinterpret_cast<uint32_t*>(&rtcStats), sizeof(rtcStats));
  uint32_t expected = hashBytes(reinterpret_cast<const uint8_t*>(&rtcStats), offsetof(RtcStats, crc));
  if (!readOk || rtcStats.magic != RTC_MAGIC || rtcStats.crc != expected) {
    memset(&rtcStats, 0, sizeof(rtcStats));
    rtcStats.magic = RTC_MAGIC;
  }
  rtcStats.bootCount++;
  saveRtcStats();
  bootResetReason = ESP.getResetReason();
}

// -----------------------------------------------------------------------------
// Modbus / mapa do URP6000
// -----------------------------------------------------------------------------
static const uint16_t REG_ID_START   = 0x0088;
static const uint16_t REG_ID_LEN     = 2;
static const uint16_t REG_RAT_START  = 0x003C;
static const uint16_t REG_RAT_LEN    = 3;
static const uint16_t REG_MEAS_START = 0x02BC;
static const uint16_t REG_MEAS_LEN   = 8;
static const uint16_t REG_FQ_START   = 0x02C7;
static const uint16_t REG_FQ_LEN     = 5;
static const uint16_t REG_P_START    = 0x02CD;
static const uint16_t REG_P_LEN      = 6;

static const uint16_t REG_DIAG_START = 0x0100;
static const uint16_t REG_DIAG_LEN   = 0x40;
static const uint16_t REG_SNAP_START = 0x0200;
static const uint16_t REG_SNAP_LEN   = 32;

static const uint32_t WAIT_ID_PERIOD_MS   = 2000;
static const uint32_t CYCLIC_PERIOD_MS    = 1000;
static const uint32_t RTU_RETRY_DELAY_MS  = 120;
static const uint32_t RTU_TXN_TIMEOUT_MS  = 900;
static const uint32_t RTU_RECOVERY_MS     = 60000;
static const uint8_t CYCLIC_MAX_RETRIES   = 2;
static const uint32_t DATA_STALE_MS       = 10000;

ModbusIP mbTCP;

alignas(ModbusRTU) static uint8_t mbRtuStorage[sizeof(ModbusRTU)];
ModbusRTU* mbRTU = nullptr;
bool rtuInitialized = false;

uint16_t stageId[REG_ID_LEN] = {0};
uint16_t stageRat[REG_RAT_LEN] = {0};
uint16_t stageMeas[REG_MEAS_LEN] = {0};
uint16_t stageFq[REG_FQ_LEN] = {0};
uint16_t stageP[REG_P_LEN] = {0};

uint32_t rtuOkCount = 0;
uint32_t rtuErrCount = 0;
uint32_t rtuRecoveryCount = 0;
uint32_t fullCycleCount = 0;
uint16_t lastRtuErr = 0;
uint16_t consecutiveRtuErrors = 0;
uint16_t snapshotSeq = 0;
uint32_t lastCycleDurationMs = 0;
uint32_t cycleStartedMs = 0;
uint32_t lastRtuSuccessMs = 0;
uint32_t lastRtuRecoveryAttemptMs = 0;

// Relogio monotônico de 64 bits para idade/uptime alem do overflow de millis().
uint32_t lastMillisRaw = 0;
uint32_t millisWrapCount = 0;
uint64_t lastFullCycleMonoMs = 0;

static void updateMonotonicClock(uint32_t now) {
  if (now < lastMillisRaw) millisWrapCount++;
  lastMillisRaw = now;
}

static uint64_t monotonicMs() {
  return (static_cast<uint64_t>(millisWrapCount) << 32) | lastMillisRaw;
}

static uint32_t uptimeSeconds32() {
  return static_cast<uint32_t>(monotonicMs() / 1000ULL);
}

static bool deadlineReached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

static bool elapsed(uint32_t now, uint32_t since, uint32_t period) {
  return static_cast<uint32_t>(now - since) >= period;
}

static uint16_t dataAgeSeconds() {
  if (lastFullCycleMonoMs == 0) return 65535;
  uint64_t age = (monotonicMs() - lastFullCycleMonoMs) / 1000ULL;
  return age > 65535ULL ? 65535 : static_cast<uint16_t>(age);
}

static bool dataIsValid() {
  if (lastFullCycleMonoMs == 0) return false;
  return (monotonicMs() - lastFullCycleMonoMs) <= DATA_STALE_MS;
}

// -----------------------------------------------------------------------------
// Rede, console e OTA
// -----------------------------------------------------------------------------
ESP8266WebServer web(80);
ESP8266HTTPUpdateServer httpUpdater;

bool tcpStarted = false;
bool webStarted = false;
bool mdnsRunning = false;
bool otaInProgress = false;
bool apActive = false;
bool apForced = false;
bool wifiWasConnected = false;

char fallbackApSsid[33] = {0};
uint32_t wifiDownSinceMs = 0;
uint32_t wifiConnectedSinceMs = 0;
uint32_t lastWifiKickMs = 0;
uint32_t apStartedMs = 0;
uint32_t apForcedDurationMs = 0;

uint32_t tcpStartCount = 0;
uint32_t wifiReconnectCount = 0;
uint32_t mdnsStartCount = 0;
uint32_t apStartCount = 0;

static const uint32_t WIFI_RECONNECT_PERIOD_MS = 15000;
static const uint32_t AP_FALLBACK_DELAY_MS = 60000;
static const uint32_t AP_AUTO_OFF_STABLE_MS = 300000;
static const uint32_t AP_FORCED_DEFAULT_MS = 900000;

uint32_t minFreeHeap = 0xFFFFFFFFUL;
uint32_t maxLoopGapMs = 0;
uint32_t lowHeapSinceMs = 0;
static const uint32_t LOW_HEAP_LIMIT = 7000;
static const uint32_t LOW_HEAP_RESTART_MS = 30000;

bool restartScheduled = false;
uint32_t restartScheduledAt = 0;
uint32_t restartReasonCode = 0;

// -----------------------------------------------------------------------------
// Estado RTU
// -----------------------------------------------------------------------------
RtuState rtuState = RTU_WAIT_ID;
CyStep cyStep = CY_IDCHECK;
uint8_t cyRetries = 0;
uint32_t rtuNextActionAt = 0;

struct RtuRequest {
  volatile bool busy;
  volatile bool completed;
  volatile bool ok;
  volatile uint16_t result;
  uint32_t startedMs;
  CyStep step;
};

RtuRequest rtuReq = {false, false, false, 0, 0, CY_IDCHECK};

static bool rtuCallback(Modbus::ResultCode event, uint16_t, void*) {
  rtuReq.result = static_cast<uint16_t>(event);
  rtuReq.ok = (event == Modbus::EX_SUCCESS);
  rtuReq.busy = false;
  rtuReq.completed = true;
  return true;
}

static void reserveTcpRegisters() {
  mbTCP.addHreg(REG_ID_START, 0, REG_ID_LEN);
  mbTCP.addHreg(REG_RAT_START, 0, REG_RAT_LEN);
  mbTCP.addHreg(REG_MEAS_START, 0, REG_MEAS_LEN);
  mbTCP.addHreg(REG_FQ_START, 0, REG_FQ_LEN);
  mbTCP.addHreg(REG_P_START, 0, REG_P_LEN);
  mbTCP.addHreg(REG_DIAG_START, 0, REG_DIAG_LEN);
  mbTCP.addHreg(REG_SNAP_START, 0, REG_SNAP_LEN);
}

static void mirrorBlock(uint16_t start, const uint16_t* values, uint16_t len) {
  for (uint16_t i = 0; i < len; ++i) mbTCP.Hreg(start + i, values[i]);
}

static uint16_t statusFlags() {
  uint16_t f = 0;
  if (WiFi.status() == WL_CONNECTED) f |= (1U << 0);
  if (tcpStarted)                    f |= (1U << 1);
  if (rtuInitialized)                f |= (1U << 2);
  if (dataIsValid())                 f |= (1U << 3);
  if (dataAgeSeconds() <= 5)         f |= (1U << 4);
  if (mdnsRunning)                   f |= (1U << 5);
  if (apActive)                      f |= (1U << 6);
  if (otaInProgress)                 f |= (1U << 7);
  if (webStarted)                    f |= (1U << 8);
  if (ESP.getFreeHeap() < LOW_HEAP_LIMIT) f |= (1U << 9);
  return f;
}

static uint16_t encodedRssi() {
  int rssi = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : -200;
  if (rssi < -200) rssi = -200;
  if (rssi > 0) rssi = 0;
  return static_cast<uint16_t>(rssi + 200);
}

static void setIpRegs(uint16_t start, const IPAddress& ip) {
  for (uint8_t i = 0; i < 4; ++i) mbTCP.Hreg(start + i, ip[i]);
}

static void publishSnapshot() {
  mbTCP.Hreg(REG_SNAP_START + 0, statusFlags());
  mbTCP.Hreg(REG_SNAP_START + 1, snapshotSeq);
  mbTCP.Hreg(REG_SNAP_START + 2, dataAgeSeconds());

  for (uint16_t i = 0; i < REG_RAT_LEN; ++i)  mbTCP.Hreg(REG_SNAP_START + 3  + i, mbTCP.Hreg(REG_RAT_START + i));
  for (uint16_t i = 0; i < REG_MEAS_LEN; ++i) mbTCP.Hreg(REG_SNAP_START + 6  + i, mbTCP.Hreg(REG_MEAS_START + i));
  for (uint16_t i = 0; i < REG_FQ_LEN; ++i)   mbTCP.Hreg(REG_SNAP_START + 14 + i, mbTCP.Hreg(REG_FQ_START + i));
  for (uint16_t i = 0; i < REG_P_LEN; ++i)    mbTCP.Hreg(REG_SNAP_START + 19 + i, mbTCP.Hreg(REG_P_START + i));
  for (uint16_t i = 0; i < REG_ID_LEN; ++i)   mbTCP.Hreg(REG_SNAP_START + 25 + i, mbTCP.Hreg(REG_ID_START + i));

  mbTCP.Hreg(REG_SNAP_START + 27, static_cast<uint16_t>(rtuOkCount));
  mbTCP.Hreg(REG_SNAP_START + 28, static_cast<uint16_t>(rtuErrCount));
  mbTCP.Hreg(REG_SNAP_START + 29, encodedRssi());
  mbTCP.Hreg(REG_SNAP_START + 30, static_cast<uint16_t>(ESP.getFreeHeap() / 32U));
  mbTCP.Hreg(REG_SNAP_START + 31, lastRtuErr);
}

static void updateDiagnostics() {
  uint32_t uptime = uptimeSeconds32();
  uint32_t heap = ESP.getFreeHeap();
  if (heap < minFreeHeap) minFreeHeap = heap;

  mbTCP.Hreg(REG_DIAG_START + 0, dataIsValid() ? 1 : 0);
  mbTCP.Hreg(REG_DIAG_START + 1, snapshotSeq);
  mbTCP.Hreg(REG_DIAG_START + 2, dataAgeSeconds());
  mbTCP.Hreg(REG_DIAG_START + 3, statusFlags());
  mbTCP.Hreg(REG_DIAG_START + 4, static_cast<uint16_t>(uptime));
  mbTCP.Hreg(REG_DIAG_START + 5, static_cast<uint16_t>(uptime >> 16));
  mbTCP.Hreg(REG_DIAG_START + 6, static_cast<uint16_t>(WiFi.status()));
  mbTCP.Hreg(REG_DIAG_START + 7, encodedRssi());

  setIpRegs(REG_DIAG_START + 8,  WiFi.localIP());
  setIpRegs(REG_DIAG_START + 12, WiFi.gatewayIP());
  setIpRegs(REG_DIAG_START + 16, WiFi.subnetMask());

  mbTCP.Hreg(REG_DIAG_START + 20, static_cast<uint16_t>(heap / 32U));
  mbTCP.Hreg(REG_DIAG_START + 21, static_cast<uint16_t>(minFreeHeap / 32U));
  mbTCP.Hreg(REG_DIAG_START + 22, static_cast<uint16_t>(maxLoopGapMs > 65535 ? 65535 : maxLoopGapMs));
  mbTCP.Hreg(REG_DIAG_START + 23, static_cast<uint16_t>(rtcStats.bootCount));
  mbTCP.Hreg(REG_DIAG_START + 24, static_cast<uint16_t>(rtcStats.bootCount >> 16));
  mbTCP.Hreg(REG_DIAG_START + 25, static_cast<uint16_t>(tcpStartCount));
  mbTCP.Hreg(REG_DIAG_START + 26, static_cast<uint16_t>(wifiReconnectCount));
  mbTCP.Hreg(REG_DIAG_START + 27, static_cast<uint16_t>(mdnsStartCount));
  mbTCP.Hreg(REG_DIAG_START + 28, static_cast<uint16_t>(apStartCount));
  mbTCP.Hreg(REG_DIAG_START + 29, static_cast<uint16_t>(rtuOkCount));
  mbTCP.Hreg(REG_DIAG_START + 30, static_cast<uint16_t>(rtuOkCount >> 16));
  mbTCP.Hreg(REG_DIAG_START + 31, static_cast<uint16_t>(rtuErrCount));
  mbTCP.Hreg(REG_DIAG_START + 32, static_cast<uint16_t>(rtuErrCount >> 16));
  mbTCP.Hreg(REG_DIAG_START + 33, lastRtuErr);
  mbTCP.Hreg(REG_DIAG_START + 34, consecutiveRtuErrors);
  mbTCP.Hreg(REG_DIAG_START + 35, static_cast<uint16_t>(rtuRecoveryCount));
  mbTCP.Hreg(REG_DIAG_START + 36, static_cast<uint16_t>(fullCycleCount));
  mbTCP.Hreg(REG_DIAG_START + 37, static_cast<uint16_t>(fullCycleCount >> 16));
  mbTCP.Hreg(REG_DIAG_START + 38, static_cast<uint16_t>(lastCycleDurationMs > 65535 ? 65535 : lastCycleDurationMs));
  mbTCP.Hreg(REG_DIAG_START + 39, CONFIG_VERSION);
  mbTCP.Hreg(REG_DIAG_START + 40, FW_VERSION_WORD);
  mbTCP.Hreg(REG_DIAG_START + 41, static_cast<uint16_t>(rtuState));
  mbTCP.Hreg(REG_DIAG_START + 42, static_cast<uint16_t>(cyStep));
  mbTCP.Hreg(REG_DIAG_START + 43, cfg.slaveId);
  mbTCP.Hreg(REG_DIAG_START + 44, static_cast<uint16_t>(cfg.rtuBaud / 100U));
  mbTCP.Hreg(REG_DIAG_START + 45, apActive ? 1 : 0);
  mbTCP.Hreg(REG_DIAG_START + 46, otaInProgress ? 1 : 0);
  mbTCP.Hreg(REG_DIAG_START + 47, static_cast<uint16_t>(rtcStats.plannedRestarts));
  mbTCP.Hreg(REG_DIAG_START + 48, static_cast<uint16_t>(ESP.getChipId()));
  mbTCP.Hreg(REG_DIAG_START + 49, static_cast<uint16_t>(ESP.getChipId() >> 16));
  mbTCP.Hreg(REG_DIAG_START + 50, static_cast<uint16_t>(rtcStats.lastRestartReason));

  publishSnapshot();
}

static void enterWaitId(uint32_t delayMs = 0) {
  rtuState = RTU_WAIT_ID;
  cyStep = CY_IDCHECK;
  cyRetries = 0;
  rtuReq.busy = false;
  rtuReq.completed = false;
  rtuNextActionAt = millis() + delayMs;
}

static void constructRtu() {
  if (mbRTU != nullptr) {
    mbRTU->~ModbusRTU();
    mbRTU = nullptr;
  }

  while (Serial.available()) Serial.read();
  Serial.flush();
  Serial.end();
  delay(20);
  Serial.begin(cfg.rtuBaud, SERIAL_8N1);
  Serial.setTimeout(50);

  mbRTU = new (mbRtuStorage) ModbusRTU();
  mbRTU->begin(&Serial, PIN_RS485_DE_RE);
  mbRTU->master();
  rtuInitialized = true;
  lastRtuRecoveryAttemptMs = millis();
  enterWaitId(200);
}

static void recoverRtu(uint16_t reason) {
  lastRtuErr = reason;
  rtuRecoveryCount++;
  constructRtu();
}

static uint16_t* bufferForStep(CyStep step, uint16_t& address, uint16_t& length) {
  switch (step) {
    case CY_IDCHECK: address = REG_ID_START;   length = REG_ID_LEN;   return stageId;
    case CY_RAT:     address = REG_RAT_START;  length = REG_RAT_LEN;  return stageRat;
    case CY_MEAS:    address = REG_MEAS_START; length = REG_MEAS_LEN; return stageMeas;
    case CY_FQ:      address = REG_FQ_START;   length = REG_FQ_LEN;   return stageFq;
    case CY_P:       address = REG_P_START;    length = REG_P_LEN;    return stageP;
    default:         address = 0; length = 0; return nullptr;
  }
}

static bool startRtuStep(CyStep step) {
  if (!rtuInitialized || mbRTU == nullptr || rtuReq.busy || rtuReq.completed || otaInProgress) return false;

  uint16_t address = 0;
  uint16_t length = 0;
  uint16_t* destination = bufferForStep(step, address, length);
  if (destination == nullptr || length == 0) return false;

  rtuReq.busy = true;
  rtuReq.completed = false;
  rtuReq.ok = false;
  rtuReq.result = 0;
  rtuReq.startedMs = millis();
  rtuReq.step = step;

  bool started = mbRTU->readHreg(cfg.slaveId, address, destination, length, rtuCallback);
  if (!started) {
    rtuReq.busy = false;
    rtuReq.completed = false;
    rtuErrCount++;
    lastRtuErr = 0x00E5; // requisicao nao iniciada
    consecutiveRtuErrors++;
    rtuNextActionAt = millis() + RTU_RETRY_DELAY_MS;
    return false;
  }
  return true;
}

static void commitFullCycle(uint32_t now) {
  // Commit atomico: o cliente nunca recebe um ciclo parcialmente atualizado.
  mirrorBlock(REG_ID_START, stageId, REG_ID_LEN);
  mirrorBlock(REG_RAT_START, stageRat, REG_RAT_LEN);
  mirrorBlock(REG_MEAS_START, stageMeas, REG_MEAS_LEN);
  mirrorBlock(REG_FQ_START, stageFq, REG_FQ_LEN);
  mirrorBlock(REG_P_START, stageP, REG_P_LEN);

  snapshotSeq++;
  fullCycleCount++;
  lastFullCycleMonoMs = monotonicMs();
  lastCycleDurationMs = now - cycleStartedMs;
  lastRtuErr = 0;
  consecutiveRtuErrors = 0;
  publishSnapshot();
}

static void processRtuCompletion(uint32_t now) {
  if (!rtuReq.completed) return;

  bool ok = rtuReq.ok;
  uint16_t result = rtuReq.result;
  CyStep completedStep = rtuReq.step;
  rtuReq.completed = false;

  if (!ok) {
    rtuErrCount++;
    lastRtuErr = result;
    consecutiveRtuErrors++;

    if (cyRetries < CYCLIC_MAX_RETRIES) {
      cyRetries++;
      rtuNextActionAt = now + RTU_RETRY_DELAY_MS;
      return;
    }

    if (consecutiveRtuErrors >= 3) recoverRtu(result);
    else enterWaitId(WAIT_ID_PERIOD_MS);
    return;
  }

  rtuOkCount++;
  lastRtuSuccessMs = now;
  consecutiveRtuErrors = 0;

  if (completedStep == CY_IDCHECK && stageId[0] == 0 && stageId[1] == 0) {
    rtuErrCount++;
    lastRtuErr = 0x00E6; // identificacao nula
    consecutiveRtuErrors++;
    enterWaitId(WAIT_ID_PERIOD_MS);
    return;
  }

  cyRetries = 0;

  if (rtuState == RTU_WAIT_ID) {
    mirrorBlock(REG_ID_START, stageId, REG_ID_LEN);
    rtuState = RTU_CYCLIC;
    cyStep = CY_IDCHECK;
    cycleStartedMs = now;
    rtuNextActionAt = now;
    return;
  }

  if (completedStep == CY_P) {
    commitFullCycle(now);
    cyStep = CY_IDCHECK;
    cycleStartedMs = now + CYCLIC_PERIOD_MS;
    rtuNextActionAt = now + CYCLIC_PERIOD_MS;
  } else {
    cyStep = static_cast<CyStep>(static_cast<uint8_t>(completedStep) + 1U);
    rtuNextActionAt = now;
  }
}

static void serviceRtu(uint32_t now) {
  if (otaInProgress || !rtuInitialized || mbRTU == nullptr) return;

  mbRTU->task();

  if (rtuReq.busy && elapsed(now, rtuReq.startedMs, RTU_TXN_TIMEOUT_MS)) {
    rtuReq.busy = false;
    rtuReq.completed = false;
    rtuErrCount++;
    consecutiveRtuErrors++;
    recoverRtu(0x00E4); // timeout
    return;
  }

  processRtuCompletion(now);

  if (!rtuReq.busy && !rtuReq.completed && deadlineReached(now, rtuNextActionAt)) {
    if (rtuState == RTU_WAIT_ID) startRtuStep(CY_IDCHECK);
    else startRtuStep(cyStep);
  }

  // Supervisor independente: reconstrói a interface, mas nao derruba TCP/Wi-Fi.
  if (!rtuReq.busy && elapsed(now, lastRtuSuccessMs, RTU_RECOVERY_MS) &&
      elapsed(now, lastRtuRecoveryAttemptMs, RTU_RECOVERY_MS)) {
    rtuErrCount++;
    consecutiveRtuErrors++;
    recoverRtu(0x00E7); // sem resposta por tempo prolongado
  }
}

// -----------------------------------------------------------------------------
// Rede e mDNS
// -----------------------------------------------------------------------------
static void startFallbackAp(bool forced, uint32_t durationMs) {
  if (!apActive) {
    snprintf(fallbackApSsid, sizeof(fallbackApSsid), "URP6000-%06X", ESP.getChipId());
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
    if (WiFi.softAP(fallbackApSsid, cfg.apPass)) {
      apActive = true;
      apStartedMs = millis();
      apStartCount++;
    }
  }
  if (forced) {
    apForced = true;
    apForcedDurationMs = durationMs;
    apStartedMs = millis();
  }
}

static void stopFallbackAp() {
  if (!apActive) return;
  WiFi.softAPdisconnect(true);
  apActive = false;
  apForced = false;
  apForcedDurationMs = 0;
  if (WiFi.status() == WL_CONNECTED) WiFi.mode(WIFI_STA);
}

static void startMdns() {
  if (mdnsRunning) MDNS.close();
  mdnsRunning = MDNS.begin(cfg.hostname);
  if (mdnsRunning) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("modbus", "tcp", 502);
    mdnsStartCount++;
  }
}

static void startTcpServer() {
  mbTCP.server(502);
  tcpStarted = true;
  tcpStartCount++;
}

static void beginWifi() {
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.hostname(cfg.hostname);
  WiFi.mode(WIFI_STA);
  WiFi.begin(cfg.ssid, cfg.wifiPass);
  wifiDownSinceMs = millis();
  lastWifiKickMs = millis();
}

static void serviceWifi(uint32_t now) {
  bool connected = WiFi.status() == WL_CONNECTED;

  if (connected && !wifiWasConnected) {
    wifiWasConnected = true;
    wifiDownSinceMs = 0;
    wifiConnectedSinceMs = now;
    startTcpServer();
    startMdns();
  } else if (!connected && wifiWasConnected) {
    wifiWasConnected = false;
    wifiDownSinceMs = now;
    mdnsRunning = false;
  }

  if (!connected) {
    if (wifiDownSinceMs == 0) wifiDownSinceMs = now;

    if (elapsed(now, lastWifiKickMs, WIFI_RECONNECT_PERIOD_MS)) {
      lastWifiKickMs = now;
      wifiReconnectCount++;
      WiFi.reconnect();
    }

    if (!apActive && elapsed(now, wifiDownSinceMs, AP_FALLBACK_DELAY_MS)) {
      startFallbackAp(false, 0);
    }
  } else {
    if (mdnsRunning) MDNS.update();

    if (apActive) {
      if (apForced) {
        if (elapsed(now, apStartedMs, apForcedDurationMs)) stopFallbackAp();
      } else if (elapsed(now, wifiConnectedSinceMs, AP_AUTO_OFF_STABLE_MS)) {
        stopFallbackAp();
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Console web
// -----------------------------------------------------------------------------
static bool requireAdmin() {
  if (web.authenticate("admin", cfg.adminPass)) return true;
  web.requestAuthentication();
  return false;
}

static String ipText(const IPAddress& ip) {
  return ip.toString();
}

static void handleRoot() {
  String html;
  html.reserve(3500);
  html += F("<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>URP6000</title><style>body{font-family:Arial;max-width:760px;margin:24px auto;padding:0 12px}table{border-collapse:collapse;width:100%}td,th{border:1px solid #bbb;padding:6px;text-align:left}button{padding:8px;margin:3px}</style></head><body>");
  html += F("<h2>URP6000 Datalogger</h2><table>");
  html += F("<tr><th>Firmware</th><td>"); html += FW_VERSION; html += F("</td></tr>");
  html += F("<tr><th>Hostname</th><td>"); html += cfg.hostname; html += F(".local</td></tr>");
  html += F("<tr><th>Wi-Fi</th><td>"); html += (WiFi.status() == WL_CONNECTED ? "conectado" : "desconectado"); html += F("</td></tr>");
  html += F("<tr><th>IP STA</th><td>"); html += ipText(WiFi.localIP()); html += F("</td></tr>");
  html += F("<tr><th>RSSI</th><td>"); html += String(static_cast<int>(encodedRssi()) - 200); html += F(" dBm</td></tr>");
  html += F("<tr><th>RTU</th><td>"); html += (dataIsValid() ? "dados validos" : "sem dados atuais"); html += F("</td></tr>");
  html += F("<tr><th>Idade</th><td>"); html += String(dataAgeSeconds()); html += F(" s</td></tr>");
  html += F("<tr><th>RTU OK / erro</th><td>"); html += String(rtuOkCount); html += F(" / "); html += String(rtuErrCount); html += F("</td></tr>");
  html += F("<tr><th>Ultimo erro RTU</th><td>0x"); html += String(lastRtuErr, HEX); html += F("</td></tr>");
  html += F("<tr><th>Heap livre / minimo</th><td>"); html += String(ESP.getFreeHeap()); html += F(" / "); html += String(minFreeHeap); html += F(" bytes</td></tr>");
  html += F("<tr><th>Uptime</th><td>"); html += String(uptimeSeconds32()); html += F(" s</td></tr>");
  html += F("<tr><th>Reset anterior</th><td>"); html += bootResetReason; html += F("</td></tr>");
  if (apActive) {
    html += F("<tr><th>AP fallback</th><td>"); html += fallbackApSsid; html += F(" - 192.168.4.1</td></tr>");
  }
  html += F("</table><p><a href='/json'>JSON</a> | <a href='/config'>Configuracao</a> | <a href='/update'>Atualizar firmware</a></p>");
  html += F("<form method='post' action='/action/rtu'><button>Reiniciar RTU</button></form>");
  html += F("<form method='post' action='/action/wifi'><button>Reconectar Wi-Fi</button></form>");
  html += F("<form method='post' action='/action/ap'><button>Ativar AP por 15 min</button></form>");
  html += F("<form method='post' action='/action/reboot'><button>Reiniciar ESP</button></form>");
  html += F("</body></html>");
  web.send(200, "text/html; charset=utf-8", html);
}

static void handleJson() {
  String json;
  json.reserve(800);
  json += F("{\"firmware\":\""); json += FW_VERSION;
  json += F("\",\"hostname\":\""); json += cfg.hostname;
  json += F("\",\"wifi_connected\":"); json += (WiFi.status() == WL_CONNECTED ? "true" : "false");
  json += F(",\"ip\":\""); json += ipText(WiFi.localIP());
  json += F("\",\"rssi_dbm\":"); json += String(static_cast<int>(encodedRssi()) - 200);
  json += F(",\"data_valid\":"); json += (dataIsValid() ? "true" : "false");
  json += F(",\"age_s\":"); json += String(dataAgeSeconds());
  json += F(",\"seq\":"); json += String(snapshotSeq);
  json += F(",\"rtu_ok\":"); json += String(rtuOkCount);
  json += F(",\"rtu_errors\":"); json += String(rtuErrCount);
  json += F(",\"last_rtu_error\":"); json += String(lastRtuErr);
  json += F(",\"heap\":"); json += String(ESP.getFreeHeap());
  json += F(",\"uptime_s\":"); json += String(uptimeSeconds32());
  json += F(",\"ap_active\":"); json += (apActive ? "true" : "false");
  json += F("}");
  web.send(200, "application/json", json);
}

static void handleConfigGet() {
  if (!requireAdmin()) return;
  String html;
  html.reserve(2600);
  html += F("<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>Config</title></head><body><h2>Configuracao</h2>");
  html += F("<form method='post' action='/config'>");
  html += F("SSID:<br><input name='ssid' maxlength='32' value='"); html += cfg.ssid; html += F("'><br><br>");
  html += F("Senha Wi-Fi (vazio mantem):<br><input type='password' name='wifiPass' maxlength='64'><br><br>");
  html += F("Hostname:<br><input name='hostname' maxlength='32' value='"); html += cfg.hostname; html += F("'><br><br>");
  html += F("Baud RTU:<br><input name='baud' value='"); html += String(cfg.rtuBaud); html += F("'><br><br>");
  html += F("Slave RTU:<br><input name='slave' min='1' max='247' value='"); html += String(cfg.slaveId); html += F("'><br><br>");
  html += F("Nova senha admin/OTA (vazio mantem):<br><input type='password' name='adminPass' maxlength='32'><br><br>");
  html += F("Nova senha AP (min. 8; vazio mantem):<br><input type='password' name='apPass' maxlength='32'><br><br>");
  html += F("<button>Salvar e reiniciar</button></form><p><a href='/'>Voltar</a></p></body></html>");
  web.send(200, "text/html; charset=utf-8", html);
}

static void scheduleRestart(uint32_t delayMs, uint32_t reason) {
  restartScheduled = true;
  restartScheduledAt = millis() + delayMs;
  restartReasonCode = reason;
}

static void handleConfigPost() {
  if (!requireAdmin()) return;

  String ssid = web.arg("ssid");
  String hostname = web.arg("hostname");
  uint32_t baud = static_cast<uint32_t>(web.arg("baud").toInt());
  int slave = web.arg("slave").toInt();

  if (ssid.length() < 1 || ssid.length() > 32 || !validHostname(hostname.c_str()) ||
      !validBaud(baud) || slave < 1 || slave > 247) {
    web.send(400, "text/plain; charset=utf-8", "Configuracao invalida.");
    return;
  }

  copyText(cfg.ssid, sizeof(cfg.ssid), ssid);
  copyText(cfg.hostname, sizeof(cfg.hostname), hostname);
  cfg.rtuBaud = baud;
  cfg.slaveId = static_cast<uint8_t>(slave);

  String wifiPass = web.arg("wifiPass");
  String adminPass = web.arg("adminPass");
  String apPass = web.arg("apPass");

  if (wifiPass.length() > 0) copyText(cfg.wifiPass, sizeof(cfg.wifiPass), wifiPass);
  if (adminPass.length() > 0) {
    if (adminPass.length() < 8) { web.send(400, "text/plain", "Senha admin deve ter ao menos 8 caracteres."); return; }
    copyText(cfg.adminPass, sizeof(cfg.adminPass), adminPass);
  }
  if (apPass.length() > 0) {
    if (apPass.length() < 8) { web.send(400, "text/plain", "Senha AP deve ter ao menos 8 caracteres."); return; }
    copyText(cfg.apPass, sizeof(cfg.apPass), apPass);
  }

  saveConfig();
  web.send(200, "text/plain; charset=utf-8", "Salvo. Reiniciando...");
  scheduleRestart(1200, 10);
}

static void setupWebServer() {
  web.on("/", HTTP_GET, handleRoot);
  web.on("/json", HTTP_GET, handleJson);
  web.on("/config", HTTP_GET, handleConfigGet);
  web.on("/config", HTTP_POST, handleConfigPost);

  web.on("/action/rtu", HTTP_POST, []() {
    if (!requireAdmin()) return;
    recoverRtu(0x00F1);
    web.send(200, "text/plain; charset=utf-8", "RTU reiniciado.");
  });

  web.on("/action/wifi", HTTP_POST, []() {
    if (!requireAdmin()) return;
    WiFi.disconnect(false);
    delay(20);
    WiFi.begin(cfg.ssid, cfg.wifiPass);
    wifiReconnectCount++;
    web.send(200, "text/plain; charset=utf-8", "Reconexao solicitada.");
  });

  web.on("/action/ap", HTTP_POST, []() {
    if (!requireAdmin()) return;
    startFallbackAp(true, AP_FORCED_DEFAULT_MS);
    web.send(200, "text/plain; charset=utf-8", "AP ativo por 15 minutos em 192.168.4.1.");
  });

  web.on("/action/reboot", HTTP_POST, []() {
    if (!requireAdmin()) return;
    web.send(200, "text/plain; charset=utf-8", "Reiniciando...");
    scheduleRestart(1000, 11);
  });

  web.onNotFound([]() { web.send(404, "text/plain", "Nao encontrado"); });

  // OTA pelo navegador. Evita depender de ArduinoOTA.h e de uma segunda
  // biblioteca/porta OTA. A atualizacao continua em /update.
  Update.onStart([]() {
    otaInProgress = true;
  });
  Update.onEnd([]() {
    otaInProgress = false;
  });
  Update.onError([](uint8_t) {
    otaInProgress = false;
    enterWaitId(500);
  });

  httpUpdater.setup(&web, "/update", "admin", cfg.adminPass);
  web.begin();
  webStarted = true;
}

// -----------------------------------------------------------------------------
// LED e supervisores
// -----------------------------------------------------------------------------
static void serviceLed(uint32_t now) {
  static uint32_t lastToggle = 0;
  static bool on = false;

  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN, HIGH); // apagado
    return;
  }

  if (otaInProgress) {
    if (elapsed(now, lastToggle, 125)) {
      lastToggle = now;
      on = !on;
      digitalWrite(LED_PIN, on ? LOW : HIGH);
    }
    return;
  }

  if (dataIsValid()) {
    digitalWrite(LED_PIN, LOW); // aceso fixo
  } else if (elapsed(now, lastToggle, 500)) {
    lastToggle = now;
    on = !on;
    digitalWrite(LED_PIN, on ? LOW : HIGH); // 1 Hz
  }
}

static void serviceHeapWatchdog(uint32_t now) {
  uint32_t heap = ESP.getFreeHeap();
  if (heap < minFreeHeap) minFreeHeap = heap;

  if (heap < LOW_HEAP_LIMIT && !otaInProgress) {
    if (lowHeapSinceMs == 0) lowHeapSinceMs = now;
    if (elapsed(now, lowHeapSinceMs, LOW_HEAP_RESTART_MS)) scheduleRestart(500, 20);
  } else {
    lowHeapSinceMs = 0;
  }
}

static void serviceScheduledRestart(uint32_t now) {
  if (!restartScheduled || !deadlineReached(now, restartScheduledAt)) return;
  rtcStats.plannedRestarts++;
  rtcStats.lastRestartReason = restartReasonCode;
  saveRtcStats();
  delay(20);
  ESP.restart();
}

// -----------------------------------------------------------------------------
// Setup / loop
// -----------------------------------------------------------------------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  pinMode(PIN_RS485_DE_RE, OUTPUT);
  digitalWrite(PIN_RS485_DE_RE, LOW);

  if (USE_HALF_DUPLEX_SELECTOR) {
    pinMode(HALF_OR_FULL_PIN, OUTPUT);
    digitalWrite(HALF_OR_FULL_PIN, LOW); // half duplex
  }

  lastMillisRaw = millis();
  loadConfig();
  loadRtcStats();

  reserveTcpRegisters();
  setupWebServer();
  constructRtu();
  beginWifi();

  minFreeHeap = ESP.getFreeHeap();
  updateDiagnostics();
}

void loop() {
  static uint32_t lastLoopMs = 0;
  static uint32_t lastDiagMs = 0;

  uint32_t now = millis();
  updateMonotonicClock(now);

  uint32_t loopGap = now - lastLoopMs;
  lastLoopMs = now;
  if (loopGap > maxLoopGapMs) maxLoopGapMs = loopGap;

  mbTCP.task();
  web.handleClient();

  serviceWifi(now);
  serviceRtu(now);

  if (elapsed(now, lastDiagMs, 1000)) {
    lastDiagMs = now;
    updateDiagnostics();
  }

  serviceHeapWatchdog(now);
  serviceLed(now);
  serviceScheduledRestart(now);

  yield();
}