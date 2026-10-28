#include "site_private.h"
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <Ticker.h>
#include <ModbusRTU.h>

// ========================= HARDWARE / MODBUS =========================
#define LED_PIN           LED_BUILTIN
#define DE_RE             12
#define HALF_OR_FULL_PIN  13

#define SLAVE_1_ID        2
#define RS485_BAUD        9600

#define HR_EN_ACT_PWR_LIM 0x9D6B
#define HR_ACT_PWR_LIM_VL 0x9D6C
#define REG_COUNT         1

// ========================= WIFI =========================
const char* ssid = SITE_WIFI_SSID;
const char* password = SITE_WIFI_PASSWORD;

// ========================= TCP / mDNS =========================
static const uint16_t TCP_PORT = 8899;
static const char* MDNS_HOST = SITE_MDNS_HOST;         // foxess-power.local
static const char* MDNS_SERVICE = "_inverterctrl";       // _foxessctrl._tcp.local

// ========================= CONTROL =========================
static const uint16_t DEFAULT_POWER_TENTHS = 1000;      // 100.0%
static const uint32_t CONTROL_PERIOD_MS = 1000;         // 1 second
static const uint32_t CLIENT_TIMEOUT_MS = 5000;         // fallback if no TCP messages
static const uint32_t WIFI_RETRY_MS = 10000;

Ticker tick1s;
ModbusRTU rtu;
WiFiServer server(TCP_PORT);
WiFiClient client;

volatile bool tickFlag = false;

bool mdnsActive = false;
bool limitEnabled = false;
bool defaultApplied = false;
uint16_t targetPowerTenths = DEFAULT_POWER_TENTHS;
uint16_t lastAppliedPowerTenths = 0xFFFF;
uint32_t lastClientMsgMs = 0;
uint32_t lastWiFiAttemptMs = 0;
String rxLine;

void onTick() {
  tickFlag = true;
}

void logLine(const char* msg) {
  //Serial.println(msg);
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void ensureWiFiConnected() {
  if (WiFi.status() == WL_CONNECTED) return;

  uint32_t now = millis();
  if (now - lastWiFiAttemptMs < WIFI_RETRY_MS) return;
  lastWiFiAttemptMs = now;

  //Serial.println();
  //Serial.println("[WiFi] Conectando...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
}

void startMdns() {
  if (mdnsActive || WiFi.status() != WL_CONNECTED) return;

  if (MDNS.begin(MDNS_HOST)) {
    MDNS.addService(MDNS_SERVICE + 1, "tcp", TCP_PORT); // remove leading '_'
    MDNS.addServiceTxt(MDNS_SERVICE + 1, "tcp", "role", "inverter-power-control");
    MDNS.addServiceTxt(MDNS_SERVICE + 1, "tcp", "reg", "0x9D6C");
    MDNS.addServiceTxt(MDNS_SERVICE + 1, "tcp", "slave", "2");
    mdnsActive = true;
    //Serial.printf("[mDNS] Ativo em %s.local:%u\n", MDNS_HOST, TCP_PORT);
  } else {
    //Serial.println("[mDNS] Falha ao iniciar");
  }
}

void stopMdns() {
  if (!mdnsActive) return;
  MDNS.close();
  mdnsActive = false;
  //Serial.println("[mDNS] Desativado");
}

bool waitForBus(uint32_t timeoutMs = 400) {
  uint32_t t0 = millis();
  while (rtu.slave() && (millis() - t0 < timeoutMs)) {
    rtu.task();
    delay(2);
  }
  return !rtu.slave();
}

bool writeSingleRegister(uint16_t reg, uint16_t value) {
  if (rtu.slave()) return false;
  if (!rtu.writeHreg(SLAVE_1_ID, reg, value)) return false;
  return waitForBus();
}

bool ensureLimitEnabled() {
  if (limitEnabled) return true;
  if (!writeSingleRegister(HR_EN_ACT_PWR_LIM, 1)) {
    //Serial.println("[MODBUS] Falha ao habilitar limitacao em 0x9D6B");
    return false;
  }
  limitEnabled = true;
  //Serial.println("[MODBUS] Limitacao habilitada em 0x9D6B = 1");
  return true;
}

bool applyPower(uint16_t valueTenths) {
  if (!ensureLimitEnabled()) return false;
  if (!writeSingleRegister(HR_ACT_PWR_LIM_VL, valueTenths)) {
    //Serial.printf("[MODBUS] Falha ao escrever 0x9D6C = %u\n", valueTenths);
    return false;
  }

  lastAppliedPowerTenths = valueTenths;
  //Serial.printf("[MODBUS] 0x9D6C <- %u (%.1f%%)\n", valueTenths, valueTenths / 10.0f);
  return true;
}

void applyDefaultIfNeeded() {
  if (defaultApplied && lastAppliedPowerTenths == DEFAULT_POWER_TENTHS) return;
  if (applyPower(DEFAULT_POWER_TENTHS)) {
    defaultApplied = true;
    targetPowerTenths = DEFAULT_POWER_TENTHS;
    //Serial.println("[CTRL] Potencia default aplicada");
  }
}

void sendClient(const String& s) {
  if (client && client.connected()) client.print(s);
}

void handleCommand(String cmd) {
  cmd.trim();
  if (!cmd.length()) return;

  lastClientMsgMs = millis();
  defaultApplied = false;

  if (cmd.equalsIgnoreCase("PING")) {
    sendClient("PONG\n");
    return;
  }

  if (cmd.equalsIgnoreCase("GET")) {
    char buf[64];
    snprintf(buf, sizeof(buf), "POWER %u %.1f\n", lastAppliedPowerTenths, lastAppliedPowerTenths / 10.0f);
    sendClient(buf);
    return;
  }

  if (cmd.equalsIgnoreCase("HELLO")) {
    sendClient("HELLO ESP FOXESS\n");
    return;
  }

  if (cmd.startsWith("SET ")) {
    long v = cmd.substring(4).toInt();
    if (v < 0 || v > 1100) {
      sendClient("ERR RANGE 0..1100\n");
      return;
    }

    targetPowerTenths = (uint16_t)v;
    if (applyPower(targetPowerTenths)) sendClient("OK\n");
    else sendClient("ERR MODBUS\n");
    return;
  }

  if (cmd.startsWith("SETPCT ")) {
    float pct = cmd.substring(7).toFloat();
    long v = lroundf(pct * 10.0f);
    if (v < 0 || v > 1100) {
      sendClient("ERR RANGE 0.0..110.0\n");
      return;
    }
    targetPowerTenths = (uint16_t)v;
    if (applyPower(targetPowerTenths)) sendClient("OK\n");
    else sendClient("ERR MODBUS\n");
    return;
  }

  sendClient("ERR CMD\n");
}

void acceptClientIfAny() {
  if (client && client.connected()) return;

  WiFiClient newClient = server.available();
  if (!newClient) return;

  if (client) client.stop();
  client = newClient;
  client.setNoDelay(true);
  rxLine = "";
  lastClientMsgMs = millis();
  defaultApplied = false;

  //Serial.printf("[TCP] Cliente conectado: %s\n", client.remoteIP().toString().c_str());
  stopMdns();
  sendClient("READY\n");
}

void processTcp() {
  acceptClientIfAny();

  if (!(client && client.connected())) return;

  while (client.available()) {
    char c = (char)client.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String cmd = rxLine;
      rxLine = "";
      handleCommand(cmd);
    } else {
      if (rxLine.length() < 120) rxLine += c;
    }
  }
}

void handleConnectivityTimeout() {
  uint32_t now = millis();

  bool timedOut = false;
  if (!(client && client.connected())) {
    timedOut = true;
  } else if (now - lastClientMsgMs > CLIENT_TIMEOUT_MS) {
    timedOut = true;
  }

  if (timedOut) {
    if (client) client.stop();
    applyDefaultIfNeeded();
    startMdns();
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(HALF_OR_FULL_PIN, OUTPUT);
  digitalWrite(HALF_OR_FULL_PIN, LOW);
  digitalWrite(LED_PIN, HIGH);

  Serial.begin(RS485_BAUD);
  rtu.master();
  rtu.begin(&Serial, DE_RE);

  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  //Serial.println();
  //Serial.println("[BOOT] Inicializando");
  //Serial.println("[WiFi] Modo STA");

  server.begin();
  server.setNoDelay(true);

  tick1s.attach_ms(CONTROL_PERIOD_MS, onTick);
}

void loop() {
  rtu.task();

  ensureWiFiConnected();

  if (WiFi.status() == WL_CONNECTED) {
    static bool announcedIp = false;
    if (!announcedIp) {
      //Serial.printf("[WiFi] Conectado. IP: %s\n", WiFi.localIP().toString().c_str());
      announcedIp = true;
      startMdns();
    }
    if (mdnsActive) MDNS.update();
  } else {
    static bool announcedDown = false;
    if (!announcedDown) {
      //Serial.println("[WiFi] Aguardando conexao...");
      announcedDown = true;
    }
    stopMdns();
  }

  processTcp();

  if (tickFlag) {
    tickFlag = false;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    handleConnectivityTimeout();
  }

  yield();
}
