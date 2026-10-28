#include "site_private.h"
/*
  grid_controller_v1 - ESP8266 ESP-07
  ------------------------------------------------------------
  Controle de geração por PI:

    URP6000 -> espelho Modbus TCP -> este ESP
    este ESP -> Modbus RTU/RS485 -> 3 inversores WEG

  Inversores:
    - SIW500H_ST030_M3, slave 1
    - SIW400G_T100_W0, slave 2
    - SIW400G_T100_W0, slave 3

  Bibliotecas:
    - ESP8266 Arduino Core
    - WiFiManager (tzapu)
    - modbus-esp8266 (Alexander Emelianov)
    - InverterModbusLib

  Arquivo adicional obrigatório na mesma pasta:
    - grid_controller_web.h

  Acesso:
    - Status:        http://grid-controller.local/
    - Configuração: http://grid-controller.local/config
    - OTA:           http://grid-controller.local/update
    - WiFiManager:   http://192.168.4.1:8080 durante o AP

  Observações importantes:
    - A UART principal é exclusiva do RS485. Não use Serial.print().
    - As funções públicas atuais da InverterModbusLib são síncronas. Este sketch
      executa no máximo uma operação de inversor por passo do escalonador. Para RTU
      totalmente assíncrono, a biblioteca precisará expor begin/poll/callback.
    - Este sketch usa apenas a API pública solicitada: begin(), getRatedPower(),
      getActivePower() e setPowerLimit(). A habilitação do modo de limite deve ser
      validada na biblioteca/mapa do inversor antes do comissionamento.
    - O controle inicia DESABILITADO na primeira gravação. Faça o comissionamento,
      confira sinais/unidades e só então habilite pela página.
*/

// Timeout curto da biblioteca modbus-esp8266. Deve ser definido antes dos includes.
#ifndef MODBUSRTU_TIMEOUT
#define MODBUSRTU_TIMEOUT 350
#endif

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <EEPROM.h>
#include <Updater.h>
#include <WiFiManager.h>
#include <ModbusRTU.h>
#include <ModbusIP_ESP8266.h>
#include <InverterModbusLib.h>
#include <stddef.h>
#include <math.h>

#include "grid_controller_web.h"
#include "grid_controller_types.h"

// -----------------------------------------------------------------------------
// Firmware e valores padrão editáveis antes da primeira gravação
// -----------------------------------------------------------------------------
static const char FW_NAME[] = "grid_controller_v1";
static const char FW_VERSION[] = "1.0.4";

static const float DEFAULT_KP = 0.25f;
static const float DEFAULT_KI = 0.005f;              // 1/s
static const float DEFAULT_SETPOINT_W = 180000.0f;
static const float DEFAULT_DEADBAND_W = 1000.0f;
static const float DEFAULT_I_TERM_LIMIT_W = 100000.0f;
static const float DEFAULT_RAMP_W_PER_S = 20000.0f;
static const float DEFAULT_SMOOTH_ALPHA = 0.35f;      // 1 = sem EMA
static const float DEFAULT_MAX_LIMIT_PERCENT = 100.0f;
static const float DEFAULT_FAILSAFE_PERCENT = 0.0f;
static const float DEFAULT_MIN_COMMAND_DELTA_W = 500.0f;

static const uint32_t DEFAULT_METER_POLL_MS = 1000;
static const uint32_t DEFAULT_CONTROL_PERIOD_MS = 2000;
static const uint32_t DEFAULT_INVERTER_TASK_MS = 350;
static const uint32_t ACTIVE_POWER_POLL_MS = 2000;  // leitura de cada inversor com PI ativo
static const uint32_t DEFAULT_METER_TXN_TIMEOUT_MS = 1500;
static const uint32_t DEFAULT_METER_FAILSAFE_DELAY_MS = 10000;
static const uint32_t DEFAULT_INVERTER_OFFLINE_MS = 15000;
static const uint32_t DEFAULT_SEQ_STALL_MS = 6000;
static const uint16_t DEFAULT_METER_MAX_AGE_S = 5;
static const uint32_t DEFAULT_STATS_WINDOW_MS = 300000; // 5 minutos
static const uint32_t DEFAULT_RTU_BAUD = 9600;
static const uint32_t RTU_SERIAL_CONFIG = SERIAL_8N1;
// Modo rápido de comissionamento: somente os dois SIW400G (IDs 2 e 3).
// O ID 1 fica fora de todas as leituras, alarmes e comandos.
static const uint8_t ACTIVE_INVERTER_MASK = 0x06;
static const uint8_t EXPECTED_INVERTER_MASK = ACTIVE_INVERTER_MASK;
bool controlArmed = false;

// Potências apenas como fallback até getRatedPower() responder.
static const uint32_t FALLBACK_RATED_W[3] = {30000UL, 100000UL, 100000UL};

// -----------------------------------------------------------------------------
// Hardware
// -----------------------------------------------------------------------------
#define LED_PIN LED_BUILTIN
static const uint8_t PIN_RS485_DE_RE = 12;   // GPIO12 / D6
static const uint8_t HALF_OR_FULL_PIN = 13;  // GPIO13 / D7
static const bool USE_HALF_DUPLEX_SELECTOR = true;

// -----------------------------------------------------------------------------
// Endereços do espelho URP6000 v6
// -----------------------------------------------------------------------------
static const uint16_t URP_REG_RAT_START = 0x003C;
static const uint16_t URP_REG_RAT_LEN = 3;
static const uint16_t URP_REG_STATUS_START = 0x0100;
static const uint16_t URP_REG_STATUS_LEN = 3; // valid, seq, age
static const uint16_t URP_REG_POWER_START = 0x02CD;
static const uint16_t URP_REG_POWER_LEN = 6; // Pa/Pb/Pc int32
static const uint16_t URP_TCP_PORT = 502;

// -----------------------------------------------------------------------------
// Configuração persistente
// -----------------------------------------------------------------------------
static const uint32_t CONFIG_MAGIC = 0x53494331UL; // SIC1
static const uint16_t CONFIG_VERSION = 1;
static const size_t EEPROM_SIZE = 1024;

struct DeviceConfig {
  uint32_t magic;
  uint16_t version;

  char wifiSsid[33];
  char wifiPass[65];
  char hostname[33];
  char urpHost[64];
  char adminPass[33];
  char apPass[33];

  float kp;
  float ki;
  float setpointW;
  float deadbandW;
  float iTermLimitW;
  float rampWPerS;
  float smoothAlpha;
  float maxLimitPercent;
  float failsafePercent;
  float minCommandDeltaW;

  uint32_t meterPollMs;
  uint32_t controlPeriodMs;
  uint32_t inverterTaskMs;
  uint32_t meterTxnTimeoutMs;
  uint32_t meterFailsafeDelayMs;
  uint32_t inverterOfflineMs;
  uint32_t seqStallMs;
  uint32_t statsWindowMs;
  uint32_t rtuBaud;

  uint16_t meterMaxAgeS;
  uint8_t urpUnitId;
  int8_t meterSign;
  uint8_t currentNominalA;
  uint8_t controlEnabled;
  uint8_t reserved[5];

  uint32_t crc;
};
static_assert(sizeof(DeviceConfig) <= EEPROM_SIZE, "DeviceConfig excede a EEPROM reservada");

DeviceConfig cfg;
uint32_t runtimeSessionId = 0;

static uint32_t hashBytes(const uint8_t* data, size_t len) {
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < len; ++i) {
    hash ^= data[i];
    hash *= 16777619UL;
  }
  return hash;
}

static void copyText(char* dst, size_t dstSize, const String& src) {
  if (dstSize == 0) return;
  size_t len = src.length();
  if (len >= dstSize) len = dstSize - 1;
  memcpy(dst, src.c_str(), len);
  dst[len] = '\0';
}

static bool finiteInRange(float value, float minimum, float maximum) {
  return isfinite(value) && value >= minimum && value <= maximum;
}

static bool validHostname(const char* text) {
  size_t len = strlen(text);
  if (len < 1 || len > 32 || text[0] == '-' || text[len - 1] == '-') return false;
  for (size_t i = 0; i < len; ++i) {
    char c = text[i];
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

  copyText(cfg.hostname, sizeof(cfg.hostname), SITE_MDNS_HOST);
  copyText(cfg.urpHost, sizeof(cfg.urpHost), SITE_URP_HOST);
  copyText(cfg.adminPass, sizeof(cfg.adminPass), SITE_ADMIN_PASSWORD);
  copyText(cfg.apPass, sizeof(cfg.apPass), SITE_AP_PASSWORD);

  cfg.kp = DEFAULT_KP;
  cfg.ki = DEFAULT_KI;
  cfg.setpointW = DEFAULT_SETPOINT_W;
  cfg.deadbandW = DEFAULT_DEADBAND_W;
  cfg.iTermLimitW = DEFAULT_I_TERM_LIMIT_W;
  cfg.rampWPerS = DEFAULT_RAMP_W_PER_S;
  cfg.smoothAlpha = DEFAULT_SMOOTH_ALPHA;
  cfg.maxLimitPercent = DEFAULT_MAX_LIMIT_PERCENT;
  cfg.failsafePercent = DEFAULT_FAILSAFE_PERCENT;
  cfg.minCommandDeltaW = DEFAULT_MIN_COMMAND_DELTA_W;

  cfg.meterPollMs = DEFAULT_METER_POLL_MS;
  cfg.controlPeriodMs = DEFAULT_CONTROL_PERIOD_MS;
  cfg.inverterTaskMs = DEFAULT_INVERTER_TASK_MS;
  cfg.meterTxnTimeoutMs = DEFAULT_METER_TXN_TIMEOUT_MS;
  cfg.meterFailsafeDelayMs = DEFAULT_METER_FAILSAFE_DELAY_MS;
  cfg.inverterOfflineMs = DEFAULT_INVERTER_OFFLINE_MS;
  cfg.seqStallMs = DEFAULT_SEQ_STALL_MS;
  cfg.statsWindowMs = DEFAULT_STATS_WINDOW_MS;
  cfg.rtuBaud = DEFAULT_RTU_BAUD;

  cfg.meterMaxAgeS = DEFAULT_METER_MAX_AGE_S;
  cfg.urpUnitId = 1;
  cfg.meterSign = 1;
  cfg.currentNominalA = 5;
  cfg.controlEnabled = 0; // seguro para a primeira energização
}

static void saveConfig() {
  cfg.magic = CONFIG_MAGIC;
  cfg.version = CONFIG_VERSION;
  cfg.crc = hashBytes(reinterpret_cast<const uint8_t*>(&cfg), offsetof(DeviceConfig, crc));
  EEPROM.put(0, cfg);
  EEPROM.commit();
}

static bool validateConfig() {
  if (cfg.magic != CONFIG_MAGIC || cfg.version != CONFIG_VERSION) return false;
  uint32_t expected = hashBytes(reinterpret_cast<const uint8_t*>(&cfg), offsetof(DeviceConfig, crc));
  if (cfg.crc != expected) return false;

  cfg.wifiSsid[sizeof(cfg.wifiSsid) - 1] = '\0';
  cfg.wifiPass[sizeof(cfg.wifiPass) - 1] = '\0';
  cfg.hostname[sizeof(cfg.hostname) - 1] = '\0';
  cfg.urpHost[sizeof(cfg.urpHost) - 1] = '\0';
  cfg.adminPass[sizeof(cfg.adminPass) - 1] = '\0';
  cfg.apPass[sizeof(cfg.apPass) - 1] = '\0';

  return validHostname(cfg.hostname) && strlen(cfg.urpHost) >= 1 &&
         strlen(cfg.adminPass) >= 8 && strlen(cfg.apPass) >= 8 &&
         finiteInRange(cfg.kp, 0.0f, 10.0f) &&
         finiteInRange(cfg.ki, 0.0f, 2.0f) &&
         finiteInRange(cfg.setpointW, -1000000.0f, 1000000.0f) &&
         finiteInRange(cfg.deadbandW, 0.0f, 100000.0f) &&
         finiteInRange(cfg.iTermLimitW, 0.0f, 1000000.0f) &&
         finiteInRange(cfg.rampWPerS, 100.0f, 1000000.0f) &&
         finiteInRange(cfg.smoothAlpha, 0.01f, 1.0f) &&
         finiteInRange(cfg.maxLimitPercent, 0.0f, 100.0f) &&
         finiteInRange(cfg.failsafePercent, 0.0f, 100.0f) &&
         finiteInRange(cfg.minCommandDeltaW, 0.0f, 100000.0f) &&
         cfg.meterPollMs >= 250 && cfg.meterPollMs <= 60000 &&
         cfg.controlPeriodMs >= 500 && cfg.controlPeriodMs <= 60000 &&
         cfg.inverterTaskMs >= 100 && cfg.inverterTaskMs <= 10000 &&
         cfg.meterTxnTimeoutMs >= 200 && cfg.meterTxnTimeoutMs <= 10000 &&
         cfg.meterFailsafeDelayMs >= 1000 && cfg.meterFailsafeDelayMs <= 300000 &&
         cfg.inverterOfflineMs >= 2000 && cfg.inverterOfflineMs <= 300000 &&
         cfg.seqStallMs >= 2000 && cfg.seqStallMs <= 300000 &&
         cfg.statsWindowMs >= 60000 && cfg.statsWindowMs <= 3600000 &&
         validBaud(cfg.rtuBaud) && cfg.meterMaxAgeS >= 1 &&
         cfg.meterMaxAgeS <= 120 && cfg.urpUnitId >= 1 && cfg.urpUnitId <= 247 &&
         (cfg.meterSign == 1 || cfg.meterSign == -1) &&
         (cfg.currentNominalA == 1 || cfg.currentNominalA == 5);
}

static void loadConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, cfg);
  if (!validateConfig()) {
    loadDefaultConfig();
    saveConfig();
  }
}

// -----------------------------------------------------------------------------
// RTC RAM: identificação do boot para o logger Python
// -----------------------------------------------------------------------------
static const uint32_t RTC_MAGIC = 0x53494352UL;
struct RtcStats {
  uint32_t magic;
  uint32_t bootCount;
  uint32_t plannedRestarts;
  uint32_t lastRestartReason;
  uint32_t crc;
};
static_assert((sizeof(RtcStats) % 4) == 0, "RtcStats deve ser múltiplo de 4");
RtcStats rtcStats;

static void saveRtcStats() {
  rtcStats.magic = RTC_MAGIC;
  rtcStats.crc = hashBytes(reinterpret_cast<const uint8_t*>(&rtcStats), offsetof(RtcStats, crc));
  ESP.rtcUserMemoryWrite(0, reinterpret_cast<uint32_t*>(&rtcStats), sizeof(rtcStats));
}

static void loadRtcStats() {
  bool ok = ESP.rtcUserMemoryRead(0, reinterpret_cast<uint32_t*>(&rtcStats), sizeof(rtcStats));
  uint32_t expected = hashBytes(reinterpret_cast<const uint8_t*>(&rtcStats), offsetof(RtcStats, crc));
  if (!ok || rtcStats.magic != RTC_MAGIC || rtcStats.crc != expected) {
    memset(&rtcStats, 0, sizeof(rtcStats));
    rtcStats.magic = RTC_MAGIC;
  }
  rtcStats.bootCount++;
  saveRtcStats();
}

// -----------------------------------------------------------------------------
// Utilitários de tempo e números
// -----------------------------------------------------------------------------
static bool elapsedMs(uint32_t now, uint32_t since, uint32_t period) {
  return static_cast<uint32_t>(now - since) >= period;
}

static bool deadlineReached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

static float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

static uint32_t uptimeSeconds() {
  return millis() / 1000UL;
}

static int32_t int32FromRegs(uint16_t highWord, uint16_t lowWord) {
  uint32_t raw = (static_cast<uint32_t>(highWord) << 16) | lowWord;
  return static_cast<int32_t>(raw);
}

static float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// -----------------------------------------------------------------------------
// Modbus RTU e InverterModbusLib - objetos diretos, sem ponteiros de inversor
// -----------------------------------------------------------------------------
ModbusRTU inverterBus;

Inverter siw500h_1(SIW500H_ST030_M3);
Inverter siw400g_2(SIW400G_T100_W0);
Inverter siw400g_3(SIW400G_T100_W0);

ModbusConfigData invCfg1 = {1, DEFAULT_RTU_BAUD, SERIAL_8N1, PIN_RS485_DE_RE};
ModbusConfigData invCfg2 = {2, DEFAULT_RTU_BAUD, SERIAL_8N1, PIN_RS485_DE_RE};
ModbusConfigData invCfg3 = {3, DEFAULT_RTU_BAUD, SERIAL_8N1, PIN_RS485_DE_RE};

struct InverterRuntime {
  const char* name;
  uint8_t slaveId;
  uint32_t fallbackRatedW;
  uint32_t ratedW;
  float activeW;
  float desiredLimitW;
  float lastCommandW;
  uint32_t lastAnyOkMs;
  uint32_t lastActiveOkMs;
  uint32_t lastRatedOkMs;
  uint32_t lastCommandOkMs;
  uint32_t nextInitAttemptMs;
  uint32_t errors;
  bool initialized;
  bool ratedFromDevice;
  bool commandPending;
  uint8_t consecutiveFailures;
  uint32_t lastFailureMs;
};

InverterRuntime inv[3] = {
  {"SIW500H T030 M3", 1, FALLBACK_RATED_W[0], FALLBACK_RATED_W[0], 0, 0, NAN, 0, 0, 0, 0, 0, 0, false, false, false},
  {"SIW400G T100 W0", 2, FALLBACK_RATED_W[1], FALLBACK_RATED_W[1], 0, 0, NAN, 0, 0, 0, 0, 0, 0, false, false, false},
  {"SIW400G T100 W0", 3, FALLBACK_RATED_W[2], FALLBACK_RATED_W[2], 0, 0, NAN, 0, 0, 0, 0, 0, 0, false, false, false}
};

static bool inverterEnabled(uint8_t index) {
  return index < 3 && (ACTIVE_INVERTER_MASK & (1U << index));
}

static bool beginInverterByIndex(uint8_t index) {
  if (!inverterEnabled(index)) {
    inv[index].initialized = false;
    inv[index].ratedW = inv[index].fallbackRatedW;
    inv[index].ratedFromDevice = false;
    inv[index].activeW = NAN;
    inv[index].commandPending = false;
    return true;
  }

  bool ok = false;
  switch (index) {
    case 0: ok = siw500h_1.begin(); break;
    case 1: ok = siw400g_2.begin(); break;
    case 2: ok = siw400g_3.begin(); break;
    default: return false;
  }
  inv[index].initialized = ok;
  inv[index].commandPending = false;

  if (ok) {
    // begin() apenas configura o objeto. Ainda não prova comunicação.
    inv[index].lastAnyOkMs = 0;
    inv[index].lastActiveOkMs = 0;
    inv[index].activeW = NAN;
    inv[index].ratedW = inv[index].fallbackRatedW;
    inv[index].ratedFromDevice = false;
    inv[index].nextInitAttemptMs = 0;
    inv[index].consecutiveFailures = 0;
  } else {
    inv[index].errors++;
    inv[index].nextInitAttemptMs = millis() + 5000UL;
  }
  return ok;
}

static bool getRatedPowerByIndex(uint8_t index, uint32_t& value) {
  switch (index) {
    case 0: return siw500h_1.getRatedPower(value);
    case 1: return siw400g_2.getRatedPower(value);
    case 2: return siw400g_3.getRatedPower(value);
    default: return false;
  }
}

static bool getActivePowerByIndex(uint8_t index, float& value) {
  switch (index) {
    case 0: return siw500h_1.getActivePower(value);
    case 1: return siw400g_2.getActivePower(value);
    case 2: return siw400g_3.getActivePower(value);
    default: return false;
  }
}

static bool setPowerLimitByIndex(uint8_t index, float valueW) {
  if (!inverterEnabled(index)) return false;

  const float ratedW = static_cast<float>(
    inv[index].fallbackRatedW ? inv[index].fallbackRatedW : inv[index].ratedW
  );
  if (ratedW <= 0.0f) return false;

  const float percent = clampFloat(valueW * 100.0f / ratedW, 0.0f, 100.0f);

  // Os SIW400G possuem escrita percentual direta. Isso evita que
  // setPowerLimit(watts) faça uma leitura adicional de potência nominal.
  switch (index) {
    case 1: return siw400g_2.setPowerLimitPercent(percent);
    case 2: return siw400g_3.setPowerLimitPercent(percent);
    default: return false;
  }
}

static bool inverterOnline(uint8_t index, uint32_t now) {
  if (!inverterEnabled(index)) return false;
  return inv[index].initialized && inv[index].lastAnyOkMs != 0 &&
         !elapsedMs(now, inv[index].lastAnyOkMs, cfg.inverterOfflineMs);
}

static bool inverterPowerFresh(uint8_t index, uint32_t now) {
  return inverterOnline(index, now) && inv[index].lastActiveOkMs != 0 &&
         !elapsedMs(now, inv[index].lastActiveOkMs, cfg.inverterOfflineMs);
}

static void attachInverters() {
  invCfg1.id = 1; invCfg1.baud = cfg.rtuBaud; invCfg1.serialConfig = (SerialConfig)RTU_SERIAL_CONFIG; invCfg1.deRePin = PIN_RS485_DE_RE;
  invCfg2.id = 2; invCfg2.baud = cfg.rtuBaud; invCfg2.serialConfig = (SerialConfig)RTU_SERIAL_CONFIG; invCfg2.deRePin = PIN_RS485_DE_RE;
  invCfg3.id = 3; invCfg3.baud = cfg.rtuBaud; invCfg3.serialConfig = (SerialConfig)RTU_SERIAL_CONFIG; invCfg3.deRePin = PIN_RS485_DE_RE;

  siw500h_1.attachModbus(inverterBus);
  siw500h_1.attachSerial(Serial);
  siw500h_1.attachConfig(invCfg1);

  siw400g_2.attachModbus(inverterBus);
  siw400g_2.attachSerial(Serial);
  siw400g_2.attachConfig(invCfg2);

  siw400g_3.attachModbus(inverterBus);
  siw400g_3.attachSerial(Serial);
  siw400g_3.attachConfig(invCfg3);

  for (uint8_t i = 0; i < 3; ++i) {
    if (inverterEnabled(i)) beginInverterByIndex(i);
  }
}

// -----------------------------------------------------------------------------
// Cliente Modbus TCP para o URP6000
// -----------------------------------------------------------------------------
ModbusIP urpTcp;
IPAddress urpIp;
bool urpIpValid = false;
uint32_t nextUrpResolveMs = 0;
uint32_t nextUrpConnectMs = 0;

struct MeterRuntime {
  float rawPowerW;
  float filteredPowerW;
  float sampleHistory[3];
  uint8_t sampleCount;
  uint8_t samplePos;
  uint16_t seq;
  uint16_t ageS;
  uint32_t lastOkMs;
  uint32_t lastSeqChangeMs;
  uint32_t acceptedCount;
  uint32_t errorCount;
  uint16_t rtcFn;
  uint16_t rtp;
  bool ratiosValid;
  uint32_t ratiosReadMs;
};

MeterRuntime meter = {NAN, NAN, {0,0,0}, 0, 0, 0, 65535, 0, 0, 0, 0, 0, 0, false, 0};

uint16_t urpStatusA[URP_REG_STATUS_LEN] = {0};
uint16_t urpStatusB[URP_REG_STATUS_LEN] = {0};
uint16_t urpRatios[URP_REG_RAT_LEN] = {0};
uint16_t urpPowerRegs[URP_REG_POWER_LEN] = {0};

MeterCycleStage meterStage = METER_IDLE;
uint32_t nextMeterCycleMs = 0;

// Estado global do mDNS — precisa existir antes de resolveUrpHost()
bool mdnsRunning = false;

struct TcpTransaction {
  volatile bool active;
  volatile bool completed;
  volatile uint16_t result;
  uint16_t id;
  uint32_t startedMs;
};

TcpTransaction meterTxn = {false, false, 0, 0, 0};

static bool meterTcpCallback(Modbus::ResultCode event, uint16_t transactionId, void*) {
  if (meterTxn.active && transactionId == meterTxn.id) {
    meterTxn.result = static_cast<uint16_t>(event);
    meterTxn.completed = true;
  }
  return true;
}

static void resetMeterCycle(uint32_t now, uint32_t retryMs) {
  meterTxn.active = false;
  meterTxn.completed = false;
  meterStage = METER_IDLE;
  nextMeterCycleMs = now + retryMs;
}

static void failMeterCycle(uint32_t now, uint32_t retryMs = 500) {
  meter.errorCount++;
  resetMeterCycle(now, retryMs);
}

static bool startMeterRead(uint16_t address, uint16_t* buffer, uint16_t count,
                           MeterCycleStage waitStage, uint32_t now) {
  if (!urpIpValid || !urpTcp.isConnected(urpIp) || meterTxn.active) return false;
  uint16_t tid = urpTcp.readHreg(urpIp, address, buffer, count,
                                meterTcpCallback, cfg.urpUnitId);
  if (tid == 0) return false;
  meterTxn.active = true;
  meterTxn.completed = false;
  meterTxn.result = 0xFFFF;
  meterTxn.id = tid;
  meterTxn.startedMs = now;
  meterStage = waitStage;
  return true;
}

static bool resolveUrpHost() {
  String host(cfg.urpHost);
  host.trim();
  IPAddress parsed;
  if (parsed.fromString(host)) {
    urpIp = parsed;
    urpIpValid = true;
    return true;
  }

  IPAddress resolved;
  if (host.endsWith(".local") &&
    host.length() > 6 &&
    mdnsRunning) {

    String shortName = host.substring(0, host.length() - 6);

    // O URP6000 anuncia o serviço _modbus._tcp.
    int services = MDNS.queryService("modbus", "tcp");

    for (int i = 0; i < services; ++i) {
      String foundName = MDNS.hostname(i);
      String foundShort = foundName;

      if (foundShort.endsWith(".local")) {
        foundShort.remove(foundShort.length() - 6);
      }

      if (foundName.equalsIgnoreCase(host) ||
          foundShort.equalsIgnoreCase(shortName)) {

        resolved = MDNS.IP(i);

        if (resolved != IPAddress(0, 0, 0, 0)) {
          urpIp = resolved;
          urpIpValid = true;
          return true;
        }
      }
    }
  }

  if (WiFi.hostByName(host.c_str(), resolved) == 1 &&
      resolved != IPAddress(0, 0, 0, 0)) {
    urpIp = resolved;
    urpIpValid = true;
    return true;
  }

  urpIpValid = false;
  return false;
}

static bool meterFresh(uint32_t now) {
  if (meter.lastOkMs == 0 || meter.lastSeqChangeMs == 0) return false;
  if (elapsedMs(now, meter.lastOkMs,
                cfg.meterFailsafeDelayMs + cfg.meterPollMs * 2UL)) return false;
  if (elapsedMs(now, meter.lastSeqChangeMs, cfg.seqStallMs)) return false;
  return meter.ageS <= cfg.meterMaxAgeS;
}

static void acceptMeterSample(uint32_t now) {
  int32_t pa = int32FromRegs(urpPowerRegs[0], urpPowerRegs[1]);
  int32_t pb = int32FromRegs(urpPowerRegs[2], urpPowerRegs[3]);
  int32_t pc = int32FromRegs(urpPowerRegs[4], urpPowerRegs[5]);
  float scale = cfg.currentNominalA == 5 ? 256.0f : 1280.0f;
  int64_t totalRaw = static_cast<int64_t>(pa) +
                     static_cast<int64_t>(pb) +
                     static_cast<int64_t>(pc);
  float watts = static_cast<float>(totalRaw) *
                static_cast<float>(meter.rtcFn) *
                static_cast<float>(meter.rtp) / scale;
  watts *= static_cast<float>(cfg.meterSign);

  meter.rawPowerW = watts;
  meter.sampleHistory[meter.samplePos] = watts;
  meter.samplePos = (meter.samplePos + 1U) % 3U;
  if (meter.sampleCount < 3) meter.sampleCount++;

  float filteredInput = watts;
  if (meter.sampleCount == 3) {
    filteredInput = median3(meter.sampleHistory[0], meter.sampleHistory[1], meter.sampleHistory[2]);
  }

  if (!isfinite(meter.filteredPowerW)) meter.filteredPowerW = filteredInput;
  else meter.filteredPowerW = cfg.smoothAlpha * filteredInput +
                              (1.0f - cfg.smoothAlpha) * meter.filteredPowerW;

  uint16_t newSeq = urpStatusB[1];
  if (meter.lastOkMs == 0 || newSeq != meter.seq) meter.lastSeqChangeMs = now;
  meter.seq = newSeq;
  meter.ageS = urpStatusB[2];
  meter.lastOkMs = now;
  meter.acceptedCount++;
}

static void processMeterTransaction(uint32_t now) {
  if (!meterTxn.active) return;

  if (!meterTxn.completed) {
    if (elapsedMs(now, meterTxn.startedMs, cfg.meterTxnTimeoutMs)) {
      urpTcp.dropTransactions();
      meterTxn.active = false;
      failMeterCycle(now);
    }
    return;
  }

  bool success = meterTxn.result == static_cast<uint16_t>(Modbus::EX_SUCCESS);
  meterTxn.active = false;
  meterTxn.completed = false;
  if (!success) {
    failMeterCycle(now);
    return;
  }

  switch (meterStage) {
    case METER_WAIT_STATUS_A:
      if (urpStatusA[0] != 1 || urpStatusA[2] > cfg.meterMaxAgeS) {
        failMeterCycle(now, cfg.meterPollMs);
      } else if (!meter.ratiosValid || elapsedMs(now, meter.ratiosReadMs, 3600000UL)) {
        if (!startMeterRead(URP_REG_RAT_START, urpRatios, URP_REG_RAT_LEN,
                            METER_WAIT_RATIOS, now)) failMeterCycle(now);
      } else if (!startMeterRead(URP_REG_POWER_START, urpPowerRegs, URP_REG_POWER_LEN,
                                 METER_WAIT_POWER, now)) {
        failMeterCycle(now);
      }
      break;

    case METER_WAIT_RATIOS:
      if (urpRatios[0] == 0 || urpRatios[2] == 0) {
        meter.ratiosValid = false;
        failMeterCycle(now, cfg.meterPollMs);
      } else {
        meter.rtcFn = urpRatios[0];
        meter.rtp = urpRatios[2];
        meter.ratiosValid = true;
        meter.ratiosReadMs = now;
        if (!startMeterRead(URP_REG_POWER_START, urpPowerRegs, URP_REG_POWER_LEN,
                            METER_WAIT_POWER, now)) failMeterCycle(now);
      }
      break;

    case METER_WAIT_POWER:
      if (!startMeterRead(URP_REG_STATUS_START, urpStatusB, URP_REG_STATUS_LEN,
                          METER_WAIT_STATUS_B, now)) failMeterCycle(now);
      break;

    case METER_WAIT_STATUS_B:
      if (urpStatusB[0] == 1 && urpStatusB[2] <= cfg.meterMaxAgeS &&
          urpStatusB[1] == urpStatusA[1]) {
        acceptMeterSample(now);
        resetMeterCycle(now, cfg.meterPollMs);
      } else {
        // O snapshot mudou durante a leitura ou ficou antigo: descarta sem controlar.
        failMeterCycle(now, 100);
      }
      break;

    default:
      resetMeterCycle(now, cfg.meterPollMs);
      break;
  }
}

static void serviceMeterTcp(uint32_t now) {
  urpTcp.task();
  processMeterTransaction(now);

  if (WiFi.status() != WL_CONNECTED) return;

  if (!urpIpValid && deadlineReached(now, nextUrpResolveMs)) {
    nextUrpResolveMs = now + 30000;
    resolveUrpHost();
  }

  if (!urpIpValid) return;

  if (!urpTcp.isConnected(urpIp)) {
    if (deadlineReached(now, nextUrpConnectMs)) {
      nextUrpConnectMs = now + 5000;
      urpTcp.connect(urpIp, URP_TCP_PORT);
    }
    return;
  }

  if (meterStage == METER_IDLE && !meterTxn.active && deadlineReached(now, nextMeterCycleMs)) {
    if (!startMeterRead(URP_REG_STATUS_START, urpStatusA, URP_REG_STATUS_LEN,
                        METER_WAIT_STATUS_A, now)) {
      failMeterCycle(now);
    }
  }
}

// -----------------------------------------------------------------------------
// Controle PI, distribuição proporcional e failsafe
// -----------------------------------------------------------------------------
float piIntegralTermW = 0.0f;
float lastErrorW = 0.0f;
float controlBiasW = 0.0f;  // geração lida no instante do armamento
float totalControlledPowerW = 0.0f;
float totalCommandW = 0.0f;
float lastTotalCommandW = NAN;
uint32_t lastControlMs = 0;
bool failsafeActive = false;
uint32_t lastFailsafeApplyMs = 0;
uint32_t meterUnhealthySinceMs = 0;

static bool inverterControlEligible(uint8_t index, uint32_t now) {
  if (!inverterEnabled(index)) return false;

  // Leitura de potência permanece ativa mesmo durante o PI. Assim, o estado
  // online e a geração exibida dependem de feedback real, não apenas da escrita.
  return inverterPowerFresh(index, now);
}

static bool inverterFailsafeEligible(uint8_t index, uint32_t now) {
  if (!inverterEnabled(index)) return false;
  // Precisa ter respondido pelo menos uma leitura de potência desde o boot.
  if (!inv[index].initialized) return false;
  if (inv[index].lastActiveOkMs == 0) return false;
  if (inv[index].lastAnyOkMs == 0) return false;

  return !elapsedMs(
    now,
    inv[index].lastAnyOkMs,
    cfg.inverterOfflineMs
  );
}

static uint8_t onlineMask(uint32_t now) {
  uint8_t mask = 0;
  for (uint8_t i = 0; i < 3; ++i) if (inverterControlEligible(i, now)) mask |= (1U << i);
  return mask;
}

static uint8_t onlineCount(uint32_t now) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < 3; ++i) if (inverterControlEligible(i, now)) count++;
  return count;
}

static float sumControlledPower(uint32_t now) {
  float sum = 0.0f;
  for (uint8_t i = 0; i < 3; ++i) {
    if (inverterPowerFresh(i, now) && isfinite(inv[i].activeW)) sum += inv[i].activeW;
  }
  return sum;
}

static float inverterCapacityW(uint8_t index) {
  float rated = static_cast<float>(inv[index].ratedW ? inv[index].ratedW : inv[index].fallbackRatedW);
  return rated * cfg.maxLimitPercent / 100.0f;
}

static float sumConfiguredCapacity() {
  float sum = 0.0f;
  for (uint8_t i = 0; i < 3; ++i) {
    if (inverterEnabled(i) && inv[i].initialized) sum += inverterCapacityW(i);
  }
  return sum;
}

static float sumControlCapacity(uint32_t now) {
  float sum = 0.0f;
  for (uint8_t i = 0; i < 3; ++i) {
    if (inverterControlEligible(i, now)) sum += inverterCapacityW(i);
  }
  return sum;
}

static float sumCommandableCapacity(uint32_t now) {
  float sum = 0.0f;
  for (uint8_t i = 0; i < 3; ++i) {
    if (inverterFailsafeEligible(i, now)) {
      sum += inverterCapacityW(i);
    }
  }
  return sum;
}

static void queueDistributedCommand(float targetTotalW, uint32_t now, bool force, bool allowStaleFeedback = false) {
  (void)allowStaleFeedback;

  // No modo rápido, a distribuição usa somente os dois inversores configurados.
  // A potência nominal fixa evita depender de leituras durante a malha fechada.
  float capacity = sumConfiguredCapacity();
  if (capacity <= 0.0f) return;

  targetTotalW = clampFloat(targetTotalW, 0.0f, capacity);
  totalCommandW = targetTotalW;

  for (uint8_t i = 0; i < 3; ++i) {
    if (!inverterEnabled(i) || !inv[i].initialized) continue;

    float share = targetTotalW * inverterCapacityW(i) / capacity;
    share = clampFloat(share, 0.0f, inverterCapacityW(i));
    inv[i].desiredLimitW = share;

    bool firstCommand = !isfinite(inv[i].lastCommandW);
    if (force || firstCommand ||
        fabsf(share - inv[i].lastCommandW) >= cfg.minCommandDeltaW ||
        elapsedMs(now, inv[i].lastCommandOkMs, 30000UL)) {
      inv[i].commandPending = true;
    }
  }
}

static void applyFailsafe(uint32_t now, bool force) {
  float target = sumConfiguredCapacity() * cfg.failsafePercent / 100.0f;
  piIntegralTermW = 0.0f;
  lastTotalCommandW = target;
  // No failsafe, tenta comandar todo inversor que ainda aceita operações, mesmo
  // que seu último feedback de potência já tenha envelhecido.
  queueDistributedCommand(target, now, force, true);
  failsafeActive = true;
  lastFailsafeApplyMs = now;
}

static void resetControllerState() {
  // Qualquer pausa ou alteração de parâmetros exige novo armamento e nova
  // captura da potência atual dos inversores como bias do controlador.
  controlArmed = false;
  piIntegralTermW = 0.0f;
  lastErrorW = 0.0f;
  controlBiasW = 0.0f;
  lastControlMs = 0;
  lastTotalCommandW = NAN;
  meterUnhealthySinceMs = 0;

  // Evita enviar, ao reativar o PI, um comando que havia ficado pendente
  // antes de uma pausa ou alteração de parâmetros.
  for (uint8_t i = 0; i < 3; ++i) inv[i].commandPending = false;
}

static void serviceController(uint32_t now) {
  if (!cfg.controlEnabled) {
    failsafeActive = false;
    controlArmed = false;
    return;
  }

  if (!meterFresh(now)) {
    if (!controlArmed) return;

    if (meterUnhealthySinceMs == 0) meterUnhealthySinceMs = now;

    if (elapsedMs(now, meterUnhealthySinceMs, cfg.meterFailsafeDelayMs)) {
      if (!failsafeActive || elapsedMs(now, lastFailsafeApplyMs, 10000UL)) {
        applyFailsafe(now, true);
      }
    }
    return;
  }

  meterUnhealthySinceMs = 0;

  if (!controlArmed) {
    // Armamento: precisa de uma leitura válida dos IDs 2 e 3.
    if (onlineMask(now) != EXPECTED_INVERTER_MASK) return;

    controlBiasW = sumControlledPower(now);
    totalControlledPowerW = controlBiasW;
    totalCommandW = controlBiasW;
    lastTotalCommandW = controlBiasW;
    piIntegralTermW = 0.0f;
    lastErrorW = meter.filteredPowerW - cfg.setpointW;
    lastControlMs = now;
    controlArmed = true;

    // Envia uma primeira referência igual à geração observada, evitando degrau.
    queueDistributedCommand(controlBiasW, now, true);
    return;
  }

  if (failsafeActive) {
    failsafeActive = false;
    // O retorno do medidor usa o último comando como novo ponto de operação.
    controlBiasW = isfinite(lastTotalCommandW) ? lastTotalCommandW : totalCommandW;
    piIntegralTermW = 0.0f;
    lastControlMs = now;
  }

  if (lastControlMs != 0 &&
      !elapsedMs(now, lastControlMs, cfg.controlPeriodMs)) return;

  float dtS = static_cast<float>(now - lastControlMs) / 1000.0f;
  lastControlMs = now;
  if (dtS <= 0.0f || dtS > 60.0f) {
    dtS = cfg.controlPeriodMs / 1000.0f;
  }

  float errorW = meter.filteredPowerW - cfg.setpointW;
  lastErrorW = errorW;
  float controlErrorW = fabsf(errorW) <= cfg.deadbandW ? 0.0f : errorW;

  float capacityW = sumConfiguredCapacity();
  if (capacityW <= 0.0f) return;

  float candidateI = clampFloat(
    piIntegralTermW + cfg.ki * controlErrorW * dtS,
    -cfg.iTermLimitW,
    cfg.iTermLimitW
  );

  float unsaturatedTarget =
    controlBiasW - cfg.kp * controlErrorW - candidateI;

  bool pushingLow =
    unsaturatedTarget < 0.0f && controlErrorW > 0.0f;
  bool pushingHigh =
    unsaturatedTarget > capacityW && controlErrorW < 0.0f;

  if (!pushingLow && !pushingHigh) {
    piIntegralTermW = candidateI;
  }

  float targetW =
    controlBiasW - cfg.kp * controlErrorW - piIntegralTermW;
  targetW = clampFloat(targetW, 0.0f, capacityW);

  if (!isfinite(lastTotalCommandW)) {
    lastTotalCommandW = controlBiasW;
  }

  float maxDelta = cfg.rampWPerS * dtS;
  targetW = clampFloat(
    targetW,
    lastTotalCommandW - maxDelta,
    lastTotalCommandW + maxDelta
  );
  targetW = clampFloat(targetW, 0.0f, capacityW);

  lastTotalCommandW = targetW;

  // totalControlledPowerW é atualizado pelas leituras reais dos inversores.
  queueDistributedCommand(targetW, now, false);
}

// -----------------------------------------------------------------------------
// Escalonador cooperativo das operações InverterModbusLib
// -----------------------------------------------------------------------------
uint32_t nextInverterTaskMs = 0;
uint8_t inverterRoundRobin = 0;
bool preferReadOperation = true;

static void markInverterFailure(uint8_t index, uint32_t now) {
  inv[index].errors++;
  if (inv[index].consecutiveFailures < 255) {
    inv[index].consecutiveFailures++;
  }
  inv[index].lastFailureMs = now;

  // Mantém o objeto inicializado e, quando existir comando pendente,
  // permite nova tentativa. Evita o ciclo begin()/offline que não recuperava.
  inv[index].nextInitAttemptMs = now + 1000UL;
}

static uint32_t nextActivePowerReadMs[3] = {0, 0, 0};

static bool servicePendingInverterWrite(uint32_t now) {
  for (uint8_t offset = 0; offset < 3; ++offset) {
    uint8_t i = (inverterRoundRobin + offset) % 3U;

    if (!inverterEnabled(i) || !inv[i].initialized) continue;
    if (!inv[i].commandPending) continue;
    if (!deadlineReached(now, inv[i].nextInitAttemptMs)) continue;

    bool ok = setPowerLimitByIndex(i, inv[i].desiredLimitW);

    if (ok) {
      inv[i].lastCommandW = inv[i].desiredLimitW;
      inv[i].lastCommandOkMs = now;
      inv[i].lastAnyOkMs = now;
      inv[i].commandPending = false;
      inv[i].consecutiveFailures = 0;
      inv[i].nextInitAttemptMs = 0;
    } else {
      markInverterFailure(i, now);
    }

    inverterRoundRobin = (i + 1U) % 3U;
    preferReadOperation = true;
    return true;
  }

  return false;
}

static bool serviceInverterPowerRead(uint32_t now) {
  for (uint8_t offset = 0; offset < 3; ++offset) {
    uint8_t i = (inverterRoundRobin + offset) % 3U;

    if (!inverterEnabled(i) || !inv[i].initialized) continue;
    if (!deadlineReached(now, inv[i].nextInitAttemptMs)) continue;
    if (!deadlineReached(now, nextActivePowerReadMs[i])) continue;

    // Agenda a próxima leitura deste inversor antes da chamada síncrona.
    nextActivePowerReadMs[i] = now + ACTIVE_POWER_POLL_MS;

    float active = 0.0f;
    bool ok = getActivePowerByIndex(i, active);

    if (ok && isfinite(active)) {
      inv[i].activeW = active;
      inv[i].lastActiveOkMs = now;
      inv[i].lastAnyOkMs = now;
      inv[i].ratedW = inv[i].fallbackRatedW;
      inv[i].consecutiveFailures = 0;
      inv[i].nextInitAttemptMs = 0;
    } else {
      markInverterFailure(i, now);
    }

    // Atualiza o valor exibido e utilizado nas estatísticas com feedback real.
    totalControlledPowerW = sumControlledPower(now);

    inverterRoundRobin = (i + 1U) % 3U;
    preferReadOperation = false;
    return true;
  }

  return false;
}

static void serviceOneInverterOperation(uint32_t now) {
  if (!deadlineReached(now, nextInverterTaskMs)) return;
  nextInverterTaskMs = now + cfg.inverterTaskMs;

  // Uma única operação RTU por passagem. Com o PI ativo, alterna leitura e
  // escrita; comandos pendentes continuam sendo atendidos sem congelar o
  // feedback de potência dos inversores.
  if (cfg.controlEnabled && controlArmed) {
    if (preferReadOperation) {
      if (serviceInverterPowerRead(now)) return;
      servicePendingInverterWrite(now);
      return;
    }

    if (servicePendingInverterWrite(now)) return;
    serviceInverterPowerRead(now);
    return;
  }

  // Com o PI pausado, mantém somente as leituras de potência ativa.
  serviceInverterPowerRead(now);
}

// -----------------------------------------------------------------------------
// Estatísticas de janelas (padrão 5 min) - ring buffer em RAM
// -----------------------------------------------------------------------------
static const uint8_t STATS_RING_SIZE = 72; // 6 h a 5 min; Python preserva em CSV

struct StatsAccumulator {
  uint32_t startedMs;
  uint32_t samples;
  double meterSum;
  double invSum;
  double setpointSum;
  double errorSum;
  double commandSum;
  float meterMin;
  float meterMax;
  float invMin;
  float invMax;
  uint8_t onlineMaskOr;
  uint16_t alarmFlagsOr;
};

StatsAccumulator statsAcc = {0,0,0,0,0,0,0,INFINITY,-INFINITY,INFINITY,-INFINITY,0,0};
StatsBucket statsRing[STATS_RING_SIZE];
uint8_t statsHead = 0;
uint8_t statsCount = 0;
uint32_t statsSequence = 0;
uint32_t lastStatsMeterAcceptedCount = 0;

// Alarmes
static const uint16_t ALARM_WIFI = 1U << 0;
static const uint16_t ALARM_METER = 1U << 1;
static const uint16_t ALARM_INV1 = 1U << 2;
static const uint16_t ALARM_INV2 = 1U << 3;
static const uint16_t ALARM_INV3 = 1U << 4;
static const uint16_t ALARM_FAILSAFE = 1U << 5;
static const uint16_t ALARM_CONTROL_DISABLED = 1U << 6;
static const uint16_t ALARM_LOW_HEAP = 1U << 7;

static uint16_t currentAlarmFlags(uint32_t now) {
  uint16_t flags = 0;
  if (WiFi.status() != WL_CONNECTED) flags |= ALARM_WIFI;
  if (!meterFresh(now)) flags |= ALARM_METER;
  if (inverterEnabled(0) && !inverterOnline(0, now)) flags |= ALARM_INV1;
  if (inverterEnabled(1) && !inverterOnline(1, now)) flags |= ALARM_INV2;
  if (inverterEnabled(2) && !inverterOnline(2, now)) flags |= ALARM_INV3;
  if (failsafeActive) flags |= ALARM_FAILSAFE;
  if (!cfg.controlEnabled) flags |= ALARM_CONTROL_DISABLED;
  if (ESP.getFreeHeap() < 7000) flags |= ALARM_LOW_HEAP;
  return flags;
}

static void resetStatsAccumulator(uint32_t now) {
  memset(&statsAcc, 0, sizeof(statsAcc));
  statsAcc.startedMs = now;
  statsAcc.meterMin = INFINITY;
  statsAcc.meterMax = -INFINITY;
  statsAcc.invMin = INFINITY;
  statsAcc.invMax = -INFINITY;
}

static void finalizeStatsWindow(uint32_t now) {
  if (statsAcc.samples == 0) {
    resetStatsAccumulator(now);
    return;
  }

  StatsBucket bucket;
  bucket.seq = ++statsSequence;
  bucket.bootCount = rtcStats.bootCount;
  bucket.endUptimeS = uptimeSeconds();
  bucket.durationS = (now - statsAcc.startedMs) / 1000UL;
  bucket.samples = statsAcc.samples;
  bucket.meterAvgW = static_cast<float>(statsAcc.meterSum / statsAcc.samples);
  bucket.meterMinW = statsAcc.meterMin;
  bucket.meterMaxW = statsAcc.meterMax;
  bucket.invAvgW = static_cast<float>(statsAcc.invSum / statsAcc.samples);
  bucket.invMinW = statsAcc.invMin;
  bucket.invMaxW = statsAcc.invMax;
  bucket.setpointAvgW = static_cast<float>(statsAcc.setpointSum / statsAcc.samples);
  bucket.errorAvgW = static_cast<float>(statsAcc.errorSum / statsAcc.samples);
  bucket.commandAvgW = static_cast<float>(statsAcc.commandSum / statsAcc.samples);
  bucket.onlineMask = statsAcc.onlineMaskOr;
  bucket.alarmFlags = statsAcc.alarmFlagsOr;

  statsRing[statsHead] = bucket;
  statsHead = (statsHead + 1U) % STATS_RING_SIZE;
  if (statsCount < STATS_RING_SIZE) statsCount++;
  resetStatsAccumulator(now);
}

static void serviceStatistics(uint32_t now) {
  if (statsAcc.startedMs == 0) resetStatsAccumulator(now);
  if (elapsedMs(now, statsAcc.startedMs, cfg.statsWindowMs)) finalizeStatsWindow(now);

  // Uma contribuição por nova amostra aceita do medidor.
  if (meter.acceptedCount == lastStatsMeterAcceptedCount || !meterFresh(now)) return;
  lastStatsMeterAcceptedCount = meter.acceptedCount;

  float invW = sumControlledPower(now);
  float meterW = meter.filteredPowerW;
  if (!isfinite(invW) || !isfinite(meterW)) return;

  statsAcc.samples++;
  statsAcc.meterSum += meterW;
  statsAcc.invSum += invW;
  statsAcc.setpointSum += cfg.setpointW;
  statsAcc.errorSum += meterW - cfg.setpointW;
  statsAcc.commandSum += totalCommandW;
  if (meterW < statsAcc.meterMin) statsAcc.meterMin = meterW;
  if (meterW > statsAcc.meterMax) statsAcc.meterMax = meterW;
  if (invW < statsAcc.invMin) statsAcc.invMin = invW;
  if (invW > statsAcc.invMax) statsAcc.invMax = invW;
  statsAcc.onlineMaskOr |= onlineMask(now);
  statsAcc.alarmFlagsOr |= currentAlarmFlags(now);
}

// -----------------------------------------------------------------------------
// Wi-Fi, WiFiManager, mDNS, web e OTA
// -----------------------------------------------------------------------------
ESP8266WebServer web(80);
ESP8266HTTPUpdateServer httpUpdater;
WiFiManager wifiManager;

bool webStarted = false;
bool otaInProgress = false;
bool wifiWasConnected = false;
bool wifiPortalForced = false;
uint32_t wifiDownSinceMs = 0;
uint32_t wifiConnectedSinceMs = 0;
uint32_t lastWifiReconnectMs = 0;
uint32_t wifiPortalStartedMs = 0;
uint32_t wifiPortalDurationMs = 0;
char wifiPortalSsid[33] = {0};

static const uint32_t WIFI_RECONNECT_MS = 15000;
static const uint32_t WIFI_PORTAL_FALLBACK_MS = 60000;
static const uint32_t WIFI_PORTAL_FORCED_MS = 900000; // 15 min
static const uint16_t WIFI_MANAGER_PORT = 8080;

bool restartScheduled = false;
uint32_t restartAtMs = 0;
uint32_t restartReason = 0;
uint32_t minFreeHeap = 0xFFFFFFFFUL;
uint32_t lowHeapSinceMs = 0;

static bool requireAdmin() {
  if (web.authenticate("admin", cfg.adminPass)) return true;
  web.requestAuthentication();
  return false;
}

static String jsonEscape(const String& input) {
  String output;
  output.reserve(input.length() + 8);
  for (uint16_t i = 0; i < input.length(); ++i) {
    char c = input[i];
    if (c == '"' || c == '\\') { output += '\\'; output += c; }
    else if (c == '\n') output += F("\\n");
    else if (c == '\r') output += F("\\r");
    else output += c;
  }
  return output;
}

static void startMdns() {
  if (mdnsRunning) MDNS.close();
  mdnsRunning = MDNS.begin(cfg.hostname);
  if (mdnsRunning) {
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("grid-controller", "tcp", 80);
  }
}

static void saveCurrentWifiCredentialsIfChanged() {
  if (WiFi.status() != WL_CONNECTED) return;
  String ssid = WiFi.SSID();
  String pass = WiFi.psk();
  bool changed = ssid.length() > 0 && ssid != String(cfg.wifiSsid);
  if (pass.length() > 0 && pass != String(cfg.wifiPass)) changed = true;
  if (!changed) return;
  copyText(cfg.wifiSsid, sizeof(cfg.wifiSsid), ssid);
  if (pass.length() > 0) copyText(cfg.wifiPass, sizeof(cfg.wifiPass), pass);
  saveConfig();
}

static void startWifiPortal(bool forced, uint32_t durationMs) {
  if (wifiManager.getConfigPortalActive()) {
    wifiPortalForced = wifiPortalForced || forced;
    wifiPortalDurationMs = durationMs;
    wifiPortalStartedMs = millis();
    return;
  }

  snprintf(wifiPortalSsid, sizeof(wifiPortalSsid), "grid_controller-%06X", ESP.getChipId());
  wifiManager.setDebugOutput(false);
  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setConfigPortalTimeout(durationMs / 1000UL);
  wifiManager.setConnectTimeout(20);
  wifiManager.setHttpPort(WIFI_MANAGER_PORT);
  wifiManager.setAPStaticIPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  wifiManager.setHostname(cfg.hostname);
  wifiManager.startConfigPortal(wifiPortalSsid, cfg.apPass);
  wifiPortalStartedMs = millis();
  wifiPortalDurationMs = durationMs;
  wifiPortalForced = forced;
}

static void stopWifiPortal() {
  if (wifiManager.getConfigPortalActive()) wifiManager.stopConfigPortal();
  wifiPortalForced = false;
  wifiPortalDurationMs = 0;
  if (WiFi.status() == WL_CONNECTED) WiFi.mode(WIFI_STA);
}

static void beginWifi() {
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.hostname(cfg.hostname);
  WiFi.mode(WIFI_STA);
  if (strlen(cfg.wifiSsid) > 0) WiFi.begin(cfg.wifiSsid, cfg.wifiPass);
  wifiDownSinceMs = millis();
  lastWifiReconnectMs = millis();
}

static void serviceWifi(uint32_t now) {
  if (wifiManager.getConfigPortalActive()) wifiManager.process();

  bool connected = WiFi.status() == WL_CONNECTED;
  if (connected && !wifiWasConnected) {
    wifiWasConnected = true;
    wifiConnectedSinceMs = now;
    wifiDownSinceMs = 0;
    startMdns();
    saveCurrentWifiCredentialsIfChanged();
    urpIpValid = false;
    nextUrpResolveMs = now;
  } else if (!connected && wifiWasConnected) {
    wifiWasConnected = false;
    wifiDownSinceMs = now;
    if (mdnsRunning) MDNS.close();
    mdnsRunning = false;
    urpIpValid = false;
  }

  if (connected) {
    if (mdnsRunning) MDNS.update();
    if (wifiManager.getConfigPortalActive()) {
      if (wifiPortalForced) {
        if (elapsedMs(now, wifiPortalStartedMs, wifiPortalDurationMs)) stopWifiPortal();
      } else if (elapsedMs(now, wifiConnectedSinceMs, 120000UL)) {
        stopWifiPortal();
      }
    }
  } else {
    if (wifiDownSinceMs == 0) wifiDownSinceMs = now;
    if (elapsedMs(now, lastWifiReconnectMs, WIFI_RECONNECT_MS)) {
      lastWifiReconnectMs = now;
      if (strlen(cfg.wifiSsid) > 0) WiFi.begin(cfg.wifiSsid, cfg.wifiPass);
    }
    if (!wifiManager.getConfigPortalActive() &&
        elapsedMs(now, wifiDownSinceMs, WIFI_PORTAL_FALLBACK_MS)) {
      startWifiPortal(false, WIFI_PORTAL_FORCED_MS);
    }
  }
}

static void scheduleRestart(uint32_t delayMs, uint32_t reason) {
  restartScheduled = true;
  restartAtMs = millis() + delayMs;
  restartReason = reason;
}

static void handleStatusPage() {
  web.send_P(200, "text/html; charset=utf-8", grid_controller_STATUS_PAGE);
}

static void handleConfigPage() {
  if (!requireAdmin()) return;
  web.send_P(200, "text/html; charset=utf-8", grid_controller_CONFIG_PAGE);
}

static void appendAlarmNames(String& json, uint16_t flags) {
  bool first = true;
  json += '[';
  #define APPEND_ALARM(mask, text) do { if (flags & (mask)) { if (!first) json += ','; json += '"'; json += F(text); json += '"'; first = false; } } while (0)
  APPEND_ALARM(ALARM_WIFI, "Wi-Fi desconectado");
  APPEND_ALARM(ALARM_METER, "URP6000 inválido ou desatualizado");
  APPEND_ALARM(ALARM_INV1, "Inversor ID 1 offline");
  APPEND_ALARM(ALARM_INV2, "Inversor ID 2 offline");
  APPEND_ALARM(ALARM_INV3, "Inversor ID 3 offline");
  APPEND_ALARM(ALARM_FAILSAFE, "Failsafe ativo");
  APPEND_ALARM(ALARM_CONTROL_DISABLED, "Controle PI desabilitado");
  APPEND_ALARM(ALARM_LOW_HEAP, "Heap baixo");
  #undef APPEND_ALARM
  json += ']';
}

static void handleApiStatus() {
  uint32_t now = millis();
  uint16_t alarms = currentAlarmFlags(now);
  String json;
  json.reserve(2600);
  json += F("{\"firmware\":\""); json += FW_VERSION;
  json += F("\",\"hostname\":\""); json += jsonEscape(cfg.hostname);
  json += F("\",\"chip_id\":"); json += String(ESP.getChipId());
  json += F(",\"session_id\":"); json += String(runtimeSessionId);
  json += F(",\"wifi_connected\":"); json += WiFi.status() == WL_CONNECTED ? F("true") : F("false");
  json += F(",\"wifi_ssid\":\""); json += jsonEscape(WiFi.SSID());
  json += F("\",\"ip\":\""); json += WiFi.localIP().toString();
  json += F("\",\"rssi_dbm\":"); json += WiFi.status() == WL_CONNECTED ? String(WiFi.RSSI()) : String(-200);
  json += F(",\"urp_host\":\""); json += jsonEscape(cfg.urpHost);
  json += F("\",\"urp_ip\":\""); json += urpIpValid ? urpIp.toString() : String("-");
  json += F("\",\"meter_valid\":"); json += meterFresh(now) ? F("true") : F("false");
  json += F(",\"meter_raw_w\":"); json += isfinite(meter.rawPowerW) ? String(meter.rawPowerW, 2) : F("null");
  json += F(",\"meter_filtered_w\":"); json += isfinite(meter.filteredPowerW) ? String(meter.filteredPowerW, 2) : F("null");
  json += F(",\"meter_seq\":"); json += String(meter.seq);
  json += F(",\"meter_age_s\":"); json += String(meter.ageS);
  json += F(",\"controlled_power_w\":"); json += String(totalControlledPowerW, 2);
  json += F(",\"setpoint_w\":"); json += String(cfg.setpointW, 2);
  json += F(",\"error_w\":"); json += String(lastErrorW, 2);
  json += F(",\"i_term_w\":"); json += String(piIntegralTermW, 2);
  json += F(",\"total_command_w\":"); json += String(totalCommandW, 2);
  json += F(",\"control_enabled\":"); json += cfg.controlEnabled ? F("true") : F("false");
  json += F(",\"failsafe\":"); json += failsafeActive ? F("true") : F("false");
  json += F(",\"online_count\":"); json += String(onlineCount(now));
  json += F(",\"alarm_flags\":"); json += String(alarms);
  json += F(",\"alarm_names\":"); appendAlarmNames(json, alarms);
  json += F(",\"heap\":"); json += String(ESP.getFreeHeap());
  json += F(",\"uptime_s\":"); json += String(uptimeSeconds());
  json += F(",\"boot_count\":"); json += String(rtcStats.bootCount);
  json += F(",\"portal_active\":"); json += wifiManager.getConfigPortalActive() ? F("true") : F("false");
  json += F(",\"inverters\":[");
  for (uint8_t i = 0; i < 3; ++i) {
    if (i) json += ',';
    json += F("{\"name\":\""); json += jsonEscape(inv[i].name);
    json += F("\",\"id\":"); json += String(inv[i].slaveId);
    json += F(",\"enabled\":"); json += inverterEnabled(i) ? F("true") : F("false");
    json += F(",\"online\":"); json += inverterControlEligible(i, now) ? F("true") : F("false");
    json += F(",\"comm_online\":"); json += inverterOnline(i, now) ? F("true") : F("false");
    json += F(",\"feedback_fresh\":"); json += inverterPowerFresh(i, now) ? F("true") : F("false");
    json += F(",\"active_w\":"); json += isfinite(inv[i].activeW) ? String(inv[i].activeW, 2) : F("null");
    json += F(",\"rated_w\":"); json += String(inv[i].ratedW);
    json += F(",\"rated_from_device\":"); json += inv[i].ratedFromDevice ? F("true") : F("false");
    json += F(",\"command_w\":"); json += isfinite(inv[i].lastCommandW) ? String(inv[i].lastCommandW, 2) : F("null");
    json += F(",\"errors\":"); json += String(inv[i].errors);
    json += '}';
  }
  json += F("]}");
  web.sendHeader("Cache-Control", "no-store");
  web.send(200, "application/json; charset=utf-8", json);
}

static void handleApiConfig() {
  if (!requireAdmin()) return;
  String json;
  json.reserve(1800);
  json += F("{\"kp\":"); json += String(cfg.kp, 6);
  json += F(",\"ki\":"); json += String(cfg.ki, 6);
  json += F(",\"setpoint_kw\":"); json += String(cfg.setpointW / 1000.0f, 3);
  json += F(",\"deadband_kw\":"); json += String(cfg.deadbandW / 1000.0f, 3);
  json += F(",\"i_limit_kw\":"); json += String(cfg.iTermLimitW / 1000.0f, 3);
  json += F(",\"ramp_kw_s\":"); json += String(cfg.rampWPerS / 1000.0f, 3);
  json += F(",\"smooth_alpha\":"); json += String(cfg.smoothAlpha, 3);
  json += F(",\"max_limit_pct\":"); json += String(cfg.maxLimitPercent, 2);
  json += F(",\"failsafe_pct\":"); json += String(cfg.failsafePercent, 2);
  json += F(",\"min_cmd_kw\":"); json += String(cfg.minCommandDeltaW / 1000.0f, 3);
  json += F(",\"meter_poll_ms\":"); json += String(cfg.meterPollMs);
  json += F(",\"control_ms\":"); json += String(cfg.controlPeriodMs);
  json += F(",\"inverter_poll_ms\":"); json += String(cfg.inverterTaskMs);
  json += F(",\"meter_timeout_ms\":"); json += String(cfg.meterTxnTimeoutMs);
  json += F(",\"failsafe_delay_s\":"); json += String(cfg.meterFailsafeDelayMs / 1000UL);
  json += F(",\"inverter_offline_s\":"); json += String(cfg.inverterOfflineMs / 1000UL);
  json += F(",\"seq_stall_s\":"); json += String(cfg.seqStallMs / 1000UL);
  json += F(",\"stats_min\":"); json += String(cfg.statsWindowMs / 60000UL);
  json += F(",\"meter_max_age_s\":"); json += String(cfg.meterMaxAgeS);
  json += F(",\"urp_unit\":"); json += String(cfg.urpUnitId);
  json += F(",\"meter_sign\":"); json += String(cfg.meterSign);
  json += F(",\"current_nominal_a\":"); json += String(cfg.currentNominalA);
  json += F(",\"rtu_baud\":"); json += String(cfg.rtuBaud);
  json += F(",\"control_enabled\":"); json += cfg.controlEnabled ? F("true") : F("false");
  json += F(",\"hostname\":\""); json += jsonEscape(cfg.hostname);
  json += F("\",\"urp_host\":\""); json += jsonEscape(cfg.urpHost);
  json += F("\",\"wifi_ssid\":\""); json += jsonEscape(cfg.wifiSsid);
  json += F("\"}");
  web.sendHeader("Cache-Control", "no-store");
  web.send(200, "application/json; charset=utf-8", json);
}

static uint32_t parseUnsignedArg(const char* name, uint32_t fallback) {
  if (!web.hasArg(name)) return fallback;
  long value = web.arg(name).toInt();
  return value < 0 ? fallback : static_cast<uint32_t>(value);
}

static float parseFloatArg(const char* name, float fallback) {
  if (!web.hasArg(name)) return fallback;
  float value = web.arg(name).toFloat();
  return isfinite(value) ? value : fallback;
}

static void handleConfigPost() {
  if (!requireAdmin()) return;
  DeviceConfig next = cfg;

  next.kp = parseFloatArg("kp", cfg.kp);
  next.ki = parseFloatArg("ki", cfg.ki);
  next.setpointW = parseFloatArg("setpoint_kw", cfg.setpointW / 1000.0f) * 1000.0f;
  next.deadbandW = parseFloatArg("deadband_kw", cfg.deadbandW / 1000.0f) * 1000.0f;
  next.iTermLimitW = parseFloatArg("i_limit_kw", cfg.iTermLimitW / 1000.0f) * 1000.0f;
  next.rampWPerS = parseFloatArg("ramp_kw_s", cfg.rampWPerS / 1000.0f) * 1000.0f;
  next.smoothAlpha = parseFloatArg("smooth_alpha", cfg.smoothAlpha);
  next.maxLimitPercent = parseFloatArg("max_limit_pct", cfg.maxLimitPercent);
  next.failsafePercent = parseFloatArg("failsafe_pct", cfg.failsafePercent);
  next.minCommandDeltaW = parseFloatArg("min_cmd_kw", cfg.minCommandDeltaW / 1000.0f) * 1000.0f;

  next.meterPollMs = parseUnsignedArg("meter_poll_ms", cfg.meterPollMs);
  next.controlPeriodMs = parseUnsignedArg("control_ms", cfg.controlPeriodMs);
  next.inverterTaskMs = parseUnsignedArg("inverter_poll_ms", cfg.inverterTaskMs);
  next.meterTxnTimeoutMs = parseUnsignedArg("meter_timeout_ms", cfg.meterTxnTimeoutMs);
  next.meterFailsafeDelayMs = parseUnsignedArg("failsafe_delay_s", cfg.meterFailsafeDelayMs / 1000UL) * 1000UL;
  next.inverterOfflineMs = parseUnsignedArg("inverter_offline_s", cfg.inverterOfflineMs / 1000UL) * 1000UL;
  next.seqStallMs = parseUnsignedArg("seq_stall_s", cfg.seqStallMs / 1000UL) * 1000UL;
  next.statsWindowMs = parseUnsignedArg("stats_min", cfg.statsWindowMs / 60000UL) * 60000UL;
  next.meterMaxAgeS = static_cast<uint16_t>(parseUnsignedArg("meter_max_age_s", cfg.meterMaxAgeS));
  next.urpUnitId = static_cast<uint8_t>(parseUnsignedArg("urp_unit", cfg.urpUnitId));
  next.meterSign = web.hasArg("meter_sign")
                     ? static_cast<int8_t>(web.arg("meter_sign").toInt())
                     : cfg.meterSign;
  next.currentNominalA = static_cast<uint8_t>(parseUnsignedArg("current_nominal_a", cfg.currentNominalA));
  next.rtuBaud = parseUnsignedArg("rtu_baud", cfg.rtuBaud);
  next.controlEnabled = web.hasArg("control_enabled")
                          ? (web.arg("control_enabled").toInt() ? 1 : 0)
                          : cfg.controlEnabled;

  copyText(next.hostname, sizeof(next.hostname), web.arg("hostname"));
  copyText(next.urpHost, sizeof(next.urpHost), web.arg("urp_host"));

  String adminPass = web.arg("admin_pass");
  String apPass = web.arg("ap_pass");
  if (adminPass.length() > 0) copyText(next.adminPass, sizeof(next.adminPass), adminPass);
  if (apPass.length() > 0) copyText(next.apPass, sizeof(next.apPass), apPass);

  DeviceConfig old = cfg;
  cfg = next;
  cfg.magic = CONFIG_MAGIC;
  cfg.version = CONFIG_VERSION;
  cfg.crc = hashBytes(reinterpret_cast<const uint8_t*>(&cfg), offsetof(DeviceConfig, crc));
  if (!validateConfig()) {
    cfg = old;
    web.send(400, "text/plain; charset=utf-8", "Configuração inválida. Revise os limites dos campos.");
    return;
  }

  bool restartRequired = strcmp(old.hostname, cfg.hostname) != 0 ||
                         old.rtuBaud != cfg.rtuBaud ||
                         strcmp(old.adminPass, cfg.adminPass) != 0 ||
                         strcmp(old.apPass, cfg.apPass) != 0;
  bool statsChanged = old.statsWindowMs != cfg.statsWindowMs;
  bool urpChanged = strcmp(old.urpHost, cfg.urpHost) != 0 ||
                    old.urpUnitId != cfg.urpUnitId ||
                    old.currentNominalA != cfg.currentNominalA ||
                    old.meterSign != cfg.meterSign;

  saveConfig();
  resetControllerState();
  if (statsChanged) resetStatsAccumulator(millis());
  if (urpChanged) {
    urpTcp.dropTransactions();
    if (urpIpValid) urpTcp.disconnect(urpIp);
    urpIpValid = false;
    meter.ratiosValid = false;
    meter.lastOkMs = 0;
    resetMeterCycle(millis(), 0);
  }
  if (restartRequired) {
    web.send(200, "text/plain; charset=utf-8", "Salvo. Reiniciando para aplicar rede/RTU/senhas...");
    scheduleRestart(1500, 10);
  } else {
    web.send(200, "text/plain; charset=utf-8", "Configuração salva e aplicada.");
  }
}

static void sendBucketJson(const StatsBucket& b) {
  String s;
  s.reserve(480);
  s += F("{\"seq\":"); s += String(b.seq);
  s += F(",\"boot_count\":"); s += String(b.bootCount);
  s += F(",\"end_uptime_s\":"); s += String(b.endUptimeS);
  s += F(",\"duration_s\":"); s += String(b.durationS);
  s += F(",\"samples\":"); s += String(b.samples);
  s += F(",\"meter_avg_w\":"); s += String(b.meterAvgW, 2);
  s += F(",\"meter_min_w\":"); s += String(b.meterMinW, 2);
  s += F(",\"meter_max_w\":"); s += String(b.meterMaxW, 2);
  s += F(",\"inv_avg_w\":"); s += String(b.invAvgW, 2);
  s += F(",\"inv_min_w\":"); s += String(b.invMinW, 2);
  s += F(",\"inv_max_w\":"); s += String(b.invMaxW, 2);
  s += F(",\"setpoint_avg_w\":"); s += String(b.setpointAvgW, 2);
  s += F(",\"error_avg_w\":"); s += String(b.errorAvgW, 2);
  s += F(",\"command_avg_w\":"); s += String(b.commandAvgW, 2);
  s += F(",\"online_mask\":"); s += String(b.onlineMask);
  s += F(",\"alarm_flags\":"); s += String(b.alarmFlags);
  s += '}';
  web.sendContent(s);
}

static void handleApiStats() {
  uint32_t after = web.hasArg("after") ? strtoul(web.arg("after").c_str(), nullptr, 10) : 0;
  uint32_t limit = web.hasArg("limit") ? strtoul(web.arg("limit").c_str(), nullptr, 10) : STATS_RING_SIZE;
  if (limit < 1) limit = 1;
  if (limit > STATS_RING_SIZE) limit = STATS_RING_SIZE;

  web.sendHeader("Cache-Control", "no-store");
  web.setContentLength(CONTENT_LENGTH_UNKNOWN);
  web.send(200, "application/json; charset=utf-8", "");
  String head = F("{\"device\":\""); head += cfg.hostname;
  head += F("\",\"chip_id\":"); head += String(ESP.getChipId());
  head += F(",\"session_id\":"); head += String(runtimeSessionId);
  head += F(",\"boot_count\":"); head += String(rtcStats.bootCount);
  head += F(",\"latest_seq\":"); head += String(statsSequence);
  head += F(",\"buckets\":[");
  web.sendContent(head);

  bool first = true;
  uint32_t sent = 0;
  uint8_t oldest = (statsHead + STATS_RING_SIZE - statsCount) % STATS_RING_SIZE;

  // Sem cursor, entrega as janelas mais recentes. Com cursor, entrega em ordem
  // cronológica as primeiras ainda não coletadas, evitando buracos no CSV.
  uint8_t startN = 0;
  if (after == 0 && statsCount > limit) startN = statsCount - static_cast<uint8_t>(limit);

  for (uint8_t n = startN; n < statsCount && sent < limit; ++n) {
    uint8_t index = (oldest + n) % STATS_RING_SIZE;
    const StatsBucket& bucket = statsRing[index];
    if (bucket.seq <= after) continue;
    if (!first) web.sendContent(",");
    sendBucketJson(bucket);
    first = false;
    sent++;
  }
  web.sendContent(F("]}"));
  web.sendContent("");
}

static void setupWebServer() {
  web.on("/", HTTP_GET, handleStatusPage);
  web.on("/api/status", HTTP_GET, handleApiStatus);
  web.on("/api/stats", HTTP_GET, handleApiStats);
  web.on("/config", HTTP_GET, handleConfigPage);
  web.on("/api/config", HTTP_GET, handleApiConfig);
  web.on("/config", HTTP_POST, handleConfigPost);

  web.on("/action/control", HTTP_POST, []() {
    if (!requireAdmin()) return;

    bool enable = web.arg("enable").toInt() != 0;
    uint32_t now = millis();

    if (enable) {
      if (!meterFresh(now)) {
        web.send(
          409,
          "text/plain; charset=utf-8",
          "Controle não ativado: URP6000 sem leitura válida."
        );
        return;
      }

      if (onlineMask(now) != EXPECTED_INVERTER_MASK) {
        web.send(
          409,
          "text/plain; charset=utf-8",
          "Controle não ativado: os inversores ID 2 e ID 3 precisam responder antes do armamento."
        );
        return;
      }

      // A rota apenas autoriza o controle. O armamento real ocorre em
      // serviceController(), que captura a potência atual como bias sem degrau.
      controlArmed = false;
    } else {
      controlArmed = false;
    }

    cfg.controlEnabled = enable ? 1 : 0;
    saveConfig();
    resetControllerState();
    web.send(200, "text/plain; charset=utf-8",
             cfg.controlEnabled ? "Controle PI ativado." : "Controle PI pausado; último limite foi mantido.");
  });

  web.on("/action/reset-integral", HTTP_POST, []() {
    if (!requireAdmin()) return;
    piIntegralTermW = 0.0f;
    web.send(200, "text/plain; charset=utf-8", "Termo integral zerado.");
  });

  web.on("/action/wifi-portal", HTTP_POST, []() {
    if (!requireAdmin()) return;
    startWifiPortal(true, WIFI_PORTAL_FORCED_MS);
    web.send(200, "text/plain; charset=utf-8",
             "AP ativo por 15 min. Conecte em grid_controller-XXXXXX e abra 192.168.4.1:8080.");
  });

  web.on("/action/reboot", HTTP_POST, []() {
    if (!requireAdmin()) return;
    web.send(200, "text/plain; charset=utf-8", "Reiniciando...");
    scheduleRestart(1000, 11);
  });

  web.onNotFound([]() { web.send(404, "text/plain; charset=utf-8", "Não encontrado"); });

  Update.onStart([]() { otaInProgress = true; });
  Update.onEnd([]() { otaInProgress = false; });
  Update.onError([](uint8_t) { otaInProgress = false; });
  httpUpdater.setup(&web, "/update", "admin", cfg.adminPass);
  web.begin();
  webStarted = true;
}

// -----------------------------------------------------------------------------
// LED, heap e reinício
// -----------------------------------------------------------------------------
static void setLed(bool on) {
  digitalWrite(LED_PIN, on ? LOW : HIGH); // LED_BUILTIN ativo em LOW
}

static void serviceLed(uint32_t now) {
  uint32_t phase = now % 2000UL;
  if (otaInProgress) { setLed((now / 125UL) & 1U); return; }
  if (wifiManager.getConfigPortalActive()) { setLed(phase < 100 || (phase >= 250 && phase < 350)); return; }
  if (WiFi.status() != WL_CONNECTED) { setLed(false); return; }
  if (failsafeActive || !meterFresh(now)) { setLed((now / 125UL) & 1U); return; }

  uint8_t count = onlineCount(now);
  if (count == 0) { setLed(phase < 100 || (phase >= 250 && phase < 350) || (phase >= 500 && phase < 600)); return; }
  if (count < 3) { setLed(phase < 100 || (phase >= 300 && phase < 400)); return; }
  if (!cfg.controlEnabled) { setLed(phase < 120); return; }
  setLed(true);
}

static void serviceHeapWatchdog(uint32_t now) {
  uint32_t heap = ESP.getFreeHeap();
  if (heap < minFreeHeap) minFreeHeap = heap;
  if (heap < 6000 && !otaInProgress) {
    if (lowHeapSinceMs == 0) lowHeapSinceMs = now;
    if (elapsedMs(now, lowHeapSinceMs, 30000UL)) scheduleRestart(1000, 20);
  } else lowHeapSinceMs = 0;
}

static void serviceScheduledRestart(uint32_t now) {
  if (!restartScheduled || !deadlineReached(now, restartAtMs)) return;
  rtcStats.plannedRestarts++;
  rtcStats.lastRestartReason = restartReason;
  saveRtcStats();
  delay(20);
  ESP.restart();
}

// -----------------------------------------------------------------------------
// Setup / loop
// -----------------------------------------------------------------------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  setLed(false);

  pinMode(PIN_RS485_DE_RE, OUTPUT);
  digitalWrite(PIN_RS485_DE_RE, LOW);
  if (USE_HALF_DUPLEX_SELECTOR) {
    pinMode(HALF_OR_FULL_PIN, OUTPUT);
    digitalWrite(HALF_OR_FULL_PIN, LOW); // half duplex
  }

  loadConfig();
  cfg.controlEnabled = 0;
  controlArmed = false;

  loadRtcStats();
  runtimeSessionId = ESP.getChipId() ^ ESP.getCycleCount() ^ micros() ^ (rtcStats.bootCount * 2654435761UL);
  if (runtimeSessionId == 0) runtimeSessionId = ESP.getChipId() | 1UL;

  setupWebServer();
  beginWifi();
  urpTcp.client();
  attachInverters();
  resetStatsAccumulator(millis());
  minFreeHeap = ESP.getFreeHeap();

  if (strlen(cfg.wifiSsid) == 0) startWifiPortal(false, WIFI_PORTAL_FORCED_MS);
}

void loop() {
  uint32_t now = millis();

  web.handleClient();
  serviceWifi(now);
  serviceMeterTcp(now);

  if (!otaInProgress) {
    serviceOneInverterOperation(now);
    serviceController(now);
    serviceStatistics(now);
  }

  serviceHeapWatchdog(now);
  serviceLed(now);
  serviceScheduledRestart(now);
  yield();
}