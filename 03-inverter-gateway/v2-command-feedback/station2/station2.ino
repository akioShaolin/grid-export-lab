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

#define HR_ACT_PWR_OUT    0x9C8F
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
static const uint16_t DEFAULT_POWER_TENTHS = 0;         // 0.0%
static const uint32_t CONTROL_PERIOD_MS = 1000;         // 1 second
static const uint32_t CLIENT_TIMEOUT_MS = 5000;         // fallback if no TCP messages
static const uint32_t HOLD_LAST_CMD_MS = 10000;   // segura ultimo comando por 5 s
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
bool commFaultActive = false;
uint32_t commFaultSinceMs = 0;
String rxLine;

enum CommandType {
  CMD_NONE = 0,
  CMD_PING,
  CMD_GET,
  CMD_HELLO,
  CMD_SET,
  CMD_SETPCT,
  CMD_READPWR
};

struct CommandItem {
  CommandType type;
  uint16_t value;   // usado em SET / SETPCT já convertido para décimos
};

static const uint8_t CMD_QUEUE_SIZE = 8;

CommandItem cmdQueue[CMD_QUEUE_SIZE];
uint8_t cmdHead = 0;
uint8_t cmdTail = 0;
uint8_t cmdCount = 0;

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

bool enqueueCommand(const CommandItem& item) {
  if (cmdCount >= CMD_QUEUE_SIZE) return false;
  cmdQueue[cmdTail] = item;
  cmdTail = (cmdTail + 1) % CMD_QUEUE_SIZE;
  cmdCount++;
  return true;
}

bool dequeueCommand(CommandItem* item) {
  if (cmdCount == 0) return false;
  *item = cmdQueue[cmdHead];
  cmdHead = (cmdHead + 1) % CMD_QUEUE_SIZE;
  cmdCount--;
  return true;
}

bool queueIsEmpty() {
  return cmdCount == 0;
}

void clearQueue() {
  cmdHead = 0;
  cmdTail = 0;
  cmdCount = 0;
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
  if (!waitForBus(200)) return false;
  if (!rtu.writeHreg(SLAVE_1_ID, reg, value)) return false;
  return waitForBus(500);
}

bool readSingleRegister(uint16_t reg, uint16_t* value) {
  if (!waitForBus(200)) return false;
  if (!rtu.readHreg(SLAVE_1_ID, reg, value)) return false;
  return waitForBus(500);
}

bool ensureLimitEnabled() {
  if (limitEnabled) return true;
  if (!writeSingleRegister(HR_EN_ACT_PWR_LIM, 1)) {
    return false;
  }
  limitEnabled = true;
  return true;
}

bool applyPower(uint16_t valueTenths) {
  if (!ensureLimitEnabled()) return false;
  if (!writeSingleRegister(HR_ACT_PWR_LIM_VL, valueTenths)) {
    limitEnabled = false;
    return false;
  }

  lastAppliedPowerTenths = valueTenths;
  return true;
}

bool readPower(uint16_t* value) {
  return readSingleRegister(HR_ACT_PWR_OUT, value);
}

void executeCommand(const CommandItem& item) {
  switch (item.type) {
    case CMD_PING:
      sendClient("PONG\n");
      break;

    case CMD_GET: {
      char buf[64];
      snprintf(buf, sizeof(buf), "POWER %u %.1f\n",
               lastAppliedPowerTenths,
               lastAppliedPowerTenths / 10.0f);
      sendClient(buf);
      break;
    }

    case CMD_HELLO:
      sendClient("HELLO ESP FOXESS\n");
      break;

    case CMD_SET:
    case CMD_SETPCT:
      targetPowerTenths = item.value;
      if (applyPower(targetPowerTenths)) sendClient("OK\n");
      else sendClient("ERR MODBUS\n");
      break;

    case CMD_READPWR: {
      uint16_t pwr = 0;
      if (readPower(&pwr)) {
        char buf[96];
        snprintf(buf, sizeof(buf),
          "ACTPWR %u LIM %u %.1f\n",
          pwr,
          lastAppliedPowerTenths,
          lastAppliedPowerTenths / 10.0f
        );
        sendClient(buf);
      } else {
        sendClient("ERR READPWR\n");
      }
      break;
    }

    default:
      sendClient("ERR CMD\n");
      break;
  }
}

void processCommandQueue() {
  if (queueIsEmpty()) return;
  if (rtu.slave()) return;  // barramento ocupado

  CommandItem item;
  if (dequeueCommand(&item)) {
    executeCommand(item);
  }
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

void enqueueFromText(String cmd) {
  cmd.trim();
  if (!cmd.length()) return;

  lastClientMsgMs = millis();
  defaultApplied = false;

  CommandItem item;
  item.type = CMD_NONE;
  item.value = 0;

  if (cmd.equalsIgnoreCase("PING")) {
    item.type = CMD_PING;
  }
  else if (cmd.equalsIgnoreCase("GET")) {
    item.type = CMD_GET;
  }
  else if (cmd.equalsIgnoreCase("HELLO")) {
    item.type = CMD_HELLO;
  }
  else if (cmd.startsWith("SET ")) {
    long v = cmd.substring(4).toInt();
    if (v < 0 || v > 1100) {
      sendClient("ERR RANGE 0..1100\n");
      return;
    }
    item.type = CMD_SET;
    item.value = (uint16_t)v;
  }
  else if (cmd.startsWith("SETPCT")) {
    float pct = cmd.substring(7).toFloat();
    long v = lroundf(pct * 10.0f);
    if (v < 0 || v > 1100) {
      sendClient("ERR RANGE 0.0..110.0\n");
      return;
    }
    item.type = CMD_SETPCT;
    item.value = (uint16_t)v;
  }
  else if (cmd.equalsIgnoreCase("READPWR")) {
    item.type = CMD_READPWR;
  }
  else {
    sendClient("ERR CMD\n");
    return;
  }

  if (!enqueueCommand(item)) {
    sendClient("ERR QUEUE FULL\n");
    return;
  }

  //sendClient("QUEUED\n");
}

void acceptClientIfAny() {
  if (client && client.connected()) return;

  WiFiClient newClient = server.available();
  if (!newClient) return;

  if (client) client.stop();

  clearQueue();  
  client = newClient;
  client.setNoDelay(true);
  rxLine = "";
  lastClientMsgMs = millis();
  defaultApplied = false;
  commFaultActive = false;
  commFaultSinceMs = 0;

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
      enqueueFromText(cmd);
    } else {
      if (rxLine.length() < 120) rxLine += c;
    }
  }
}

void handleConnectivityTimeout() {
  uint32_t now = millis();

  bool commLost = false;
  if (!(client && client.connected())) {
    commLost = true;
  } else if (now - lastClientMsgMs > CLIENT_TIMEOUT_MS) {
    commLost = true;
  }

  if (!commLost) {
    commFaultActive = false;
    commFaultSinceMs = 0;
    return;
  }

  // primeira detecção de falha
  if (!commFaultActive) {
    commFaultActive = true;
    commFaultSinceMs = now;
  }

  // durante a janela de retenção: mantém último valor
  if ((now - commFaultSinceMs) < HOLD_LAST_CMD_MS) {
    return;
  }

  // passou do tempo de retenção: entra em fallback
  if (client) client.stop();
  clearQueue();
  applyDefaultIfNeeded();
  startMdns();
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
      announcedIp = true;
      startMdns();
    }
    if (mdnsActive) MDNS.update();
  } else {
    static bool announcedDown = false;
    if (!announcedDown) announcedDown = true;
    stopMdns();
  }

  processTcp();
  processCommandQueue();

  if (tickFlag) {
    tickFlag = false;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    handleConnectivityTimeout();
  }

  yield();
}
