#include "site_private.h"
/*
  URP6000 Datalogger - WiFi/TCP "Tank Mode" (ESP8266)
  Objetivo: NÃO exigir ida ao campo. Wi-Fi pode oscilar, RTU pode falhar, mas o ESP tenta se recuperar sozinho.

  IP fixo:
      SSID  : YOUR_WIFI_SSID
// Credencial definida em site_private.h.
      IP    : 192.0.2.10
      GW    : 192.0.2.10
      MASK  : 255.255.252.0

  Melhorias:
  - WiFi sem sleep (WIFI_NONE_SLEEP)
  - Autoreconnect + reconexão ativa (kick)
  - Reinício automático se WiFi não volta 
  - Rearm frequente do servidor Modbus/TCP (502)
  - Diagnóstico por registradores Modbus (sem Serial)
  - Snapshot contíguo (0x0200..0x021F)

  RTU/RS485:
  - Esqueleto pronto (ENABLE_RTU), por padrão desligado.
*/

#include <ESP8266WiFi.h>
#include <ModbusIP_ESP8266.h>

static const char* WIFI_SSID = SITE_WIFI_SSID;
static const char* WIFI_PASS = SITE_WIFI_PASSWORD;

IPAddress local_IP(SITE_IP_LOCAL_IP);
IPAddress gateway (SITE_IP_GATEWAY);
IPAddress subnet  (255, 255, 252, 0);

ModbusIP mbTCP;

#define ENABLE_RTU 0
#define DEBUG_SERIAL 0

static const uint32_t TCP_SERVER_REARM_MS     = 3000;
static const uint32_t DIAG_UPDATE_MS          = 1000;
static const uint32_t SNAP_UPDATE_MS          = 500;
static const uint32_t WIFI_DOWN_RECONNECT_MS  = 8000;
static const uint32_t WIFI_DOWN_RESTART_MS    = 30000;
static const uint32_t HEAP_MIN_RESTART        = 8000;
static const uint32_t HEAP_CHECK_MS           = 5000;

static const uint16_t REG_DIAG_START = 0x0100;
static const uint16_t REG_DIAG_LEN   = 0x40;
static const uint16_t REG_SNAP_START = 0x0200;
static const uint16_t REG_SNAP_LEN   = 32;

uint32_t tNextRearm   = 0;
uint32_t tNextDiag    = 0;
uint32_t tNextSnap    = 0;
uint32_t tNextHeapChk = 0;

uint32_t wifiDownSince = 0;
uint32_t tNextWifiKick = 0;

uint16_t resetCount = 0;
uint16_t tcpRearmCount = 0;
uint16_t wifiReconnectCount = 0;
uint16_t wifiRestartCount = 0;
uint16_t heapRestartCount = 0;

uint32_t rtuOkCount = 0;
uint32_t rtuErrCount = 0;
uint16_t lastRtuErr = 0;

uint16_t seq = 0;
uint32_t lastSnapUpdateMs = 0;
bool snapValid = false;

uint16_t snap[REG_SNAP_LEN] = {0};

static inline void set_u8(uint16_t reg, uint8_t v) { mbTCP.Hreg(reg, (uint16_t)v); }

void reserveRegs() {
  mbTCP.addHreg(REG_DIAG_START, 0, REG_DIAG_LEN);
  mbTCP.addHreg(REG_SNAP_START, 0, REG_SNAP_LEN);
}

uint16_t buildStatusFlags() {
  uint16_t f = 0;
  if (WiFi.status() == WL_CONNECTED) f |= (1u << 0);
  f |= (1u << 1);
  if (ENABLE_RTU) f |= (1u << 2);
  if (lastRtuErr == 0 && rtuOkCount > 0) f |= (1u << 3);
  if (snapValid) f |= (1u << 4);
  return f;
}

void updateDiag() {
  uint32_t up = millis() / 1000;

  mbTCP.Hreg(REG_DIAG_START + 0, (uint16_t)(up & 0xFFFF));
  mbTCP.Hreg(REG_DIAG_START + 1, (uint16_t)WiFi.status());

  int rssi = WiFi.RSSI();
  if (rssi < -200) rssi = -200;
  if (rssi > 0) rssi = 0;
  mbTCP.Hreg(REG_DIAG_START + 2, (uint16_t)(rssi + 200));

  IPAddress ip = WiFi.localIP();
  IPAddress gw = WiFi.gatewayIP();
  IPAddress mk = WiFi.subnetMask();

  set_u8(REG_DIAG_START + 3, ip[0]);
  set_u8(REG_DIAG_START + 4, ip[1]);
  set_u8(REG_DIAG_START + 5, ip[2]);
  set_u8(REG_DIAG_START + 6, ip[3]);

  set_u8(REG_DIAG_START + 7, gw[0]);
  set_u8(REG_DIAG_START + 8, gw[1]);
  set_u8(REG_DIAG_START + 9, gw[2]);
  set_u8(REG_DIAG_START + 10, gw[3]);

  set_u8(REG_DIAG_START + 11, mk[0]);
  set_u8(REG_DIAG_START + 12, mk[1]);
  set_u8(REG_DIAG_START + 13, mk[2]);
  set_u8(REG_DIAG_START + 14, mk[3]);

  mbTCP.Hreg(REG_DIAG_START + 15, (uint16_t)(ESP.getFreeHeap() / 32));
  mbTCP.Hreg(REG_DIAG_START + 16, resetCount);
  mbTCP.Hreg(REG_DIAG_START + 17, tcpRearmCount);
  mbTCP.Hreg(REG_DIAG_START + 18, wifiReconnectCount);
  mbTCP.Hreg(REG_DIAG_START + 19, wifiRestartCount);
  mbTCP.Hreg(REG_DIAG_START + 20, heapRestartCount);
  mbTCP.Hreg(REG_DIAG_START + 21, 0);
  mbTCP.Hreg(REG_DIAG_START + 22, (uint16_t)(rtuOkCount & 0xFFFF));
  mbTCP.Hreg(REG_DIAG_START + 23, (uint16_t)(rtuErrCount & 0xFFFF));
  mbTCP.Hreg(REG_DIAG_START + 24, lastRtuErr);
  mbTCP.Hreg(REG_DIAG_START + 25, buildStatusFlags());
}

void publishSnapshot() {
  snapValid = true;
  snap[0]  = buildStatusFlags();
  snap[1]  = seq;
  snap[2]  = (uint16_t)((millis() - lastSnapUpdateMs) / 1000);

  int rssi = WiFi.RSSI();
  if (rssi < -200) rssi = -200;
  if (rssi > 0) rssi = 0;
  snap[29] = (uint16_t)(rssi + 200);
  snap[30] = (uint16_t)(ESP.getFreeHeap() / 32);
  snap[27] = (uint16_t)(rtuOkCount & 0xFFFF);
  snap[28] = (uint16_t)(rtuErrCount & 0xFFFF);
  snap[31] = 0;

  for (uint16_t i = 0; i < REG_SNAP_LEN; i++) {
    mbTCP.Hreg(REG_SNAP_START + i, snap[i]);
  }
}

void connectWifiBlocking() {
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.mode(WIFI_STA);

  WiFi.disconnect(true);
  delay(200);

  //WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    yield();
    if (millis() - t0 > 20000) {
      wifiRestartCount++;
      ESP.restart();
    }
  }
  wifiDownSince = 0;
}

void rtuTaskNonBlocking() {
#if ENABLE_RTU
  // TODO: integrar Modbus RTU aqui (não-bloqueante)
#endif
}

void initSnapshotDefaults() {
  // Preenche o snapshot com valores válidos MESMO sem RTU.
  for (uint16_t i = 0; i < REG_SNAP_LEN; i++) snap[i] = 0;
  seq = 0;
  lastSnapUpdateMs = millis();
  snapValid = true;

  // Campos básicos
  snap[0] = buildStatusFlags();
  snap[1] = seq;
  snap[2] = 0; // age_s

  publishSnapshot();
  updateDiag();
}


void setup() {
#if DEBUG_SERIAL
  Serial.begin(115200);
  delay(50);
#endif

  resetCount++;

  // 1) Wi-Fi primeiro (se não conectar, reinicia)
  connectWifiBlocking();

  // 2) Registros Modbus ANTES do servidor
  reserveRegs();

  // 3) Sobe o servidor Modbus/TCP
  mbTCP.server(502);
  tcpRearmCount++;

  // 4) Snapshot/Diag já válidos desde o boot (mesmo sem RTU)
  initSnapshotDefaults();

  tNextRearm   = millis() + TCP_SERVER_REARM_MS;
  tNextDiag    = millis() + DIAG_UPDATE_MS;
  tNextSnap    = millis() + SNAP_UPDATE_MS;
  tNextHeapChk = millis() + HEAP_CHECK_MS;
}

void loop() {
  mbTCP.task();

  if (WiFi.status() == WL_CONNECTED && millis() >= tNextRearm) {
    tNextRearm = millis() + TCP_SERVER_REARM_MS;
    mbTCP.server(502);
    tcpRearmCount++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    if (wifiDownSince == 0) wifiDownSince = millis();

    if (millis() >= tNextWifiKick && (millis() - wifiDownSince) >= WIFI_DOWN_RECONNECT_MS) {
      tNextWifiKick = millis() + 5000;
      wifiReconnectCount++;
      WiFi.disconnect(true);
      delay(10);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }

    if ((millis() - wifiDownSince) >= WIFI_DOWN_RESTART_MS) {
      wifiRestartCount++;
      ESP.restart();
    }
  } else {
    wifiDownSince = 0;
  }

  if (millis() >= tNextHeapChk) {
    tNextHeapChk = millis() + HEAP_CHECK_MS;
    if (ESP.getFreeHeap() < HEAP_MIN_RESTART) {
      heapRestartCount++;
      ESP.restart();
    }
  }

  rtuTaskNonBlocking();

  if (millis() >= tNextSnap) {
    tNextSnap = millis() + SNAP_UPDATE_MS;
    seq++;
    lastSnapUpdateMs = millis();
    snapValid = true;
    publishSnapshot();
  }

  if (millis() >= tNextDiag) {
    tNextDiag = millis() + DIAG_UPDATE_MS;
    updateDiag();
  }

  yield();
}
