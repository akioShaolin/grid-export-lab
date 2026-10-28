#include "site_private.h"
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// ===================== RS485 / HW =====================
#define RS485_BAUD 9600
#define DE_RE 12
#define HALF_OR_FULL_PIN 13

// ===== Binary KeepAlive (A5 5A 00 00 00) =====
static uint32_t lastActivityMs = 0;
static const uint32_t KEEPALIVE_INTERVAL_MS = 2000;

static void tcpSendKeepAlive(WiFiClient& c) {
  uint8_t ka[5] = {0xA5, 0x5A, 0x00, 0x00, 0x00};
  c.write(ka, 5);
}

static const uint16_t MAX_RTU = 260;

// Gap RTU: 3.5 chars em 9600 ~ 3646us. Usamos margem:
static const uint32_t MODBUS_GAP_US = 5000;

// Janela para ignorar RX após TX (evita eco/turnaround)
static uint32_t ignoreRxUntilUs = 0;
// tempo aproximado por byte em 9600 8N1 (~10 bits) => ~1042us
static const uint32_t BYTE_TIME_US = 1100; // margem segura

// ===================== WiFi =====================
static const char* STA_SSID = SITE_WIFI_SSID;
static const char* STA_PASS = SITE_WIFI_PASSWORD;

static const char* AP_SSID  = SITE_AP_SSID;
static const char* AP_PASS  = SITE_AP_PASSWORD;

// ===================== TCP tunnel =====================
static const uint16_t TCP_PORT = 1502;
static IPAddress serverIP(SITE_IP_SERVERIP);
static const uint8_t MAGIC0 = 0xA5;
static const uint8_t MAGIC1 = 0x5A;

WiFiClient tcpClient;

// ===================== Web Debug =====================
ESP8266WebServer web(80);

static const uint16_t LOG_LINES = 120;
static String logBuf[LOG_LINES];
static uint16_t logHead = 0;
static uint32_t bootMs = 0;

static void logLine(const String& s) {
  logBuf[logHead] = String(millis()) + " " + s;
  logHead = (logHead + 1) % LOG_LINES;
}
static String getLogs(uint16_t lastN = 90) {
  if (lastN > LOG_LINES) lastN = LOG_LINES;
  String out;
  for (uint16_t i = 0; i < lastN; i++) {
    int idx = (int)logHead - (int)lastN + (int)i;
    while (idx < 0) idx += LOG_LINES;
    idx %= LOG_LINES;
    if (logBuf[idx].length()) out += logBuf[idx] + "\n";
  }
  return out;
}

static void handleRoot() {
  const char* html =
    "<!doctype html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>GW-MASTER Debug</title>"
    "<style>body{font-family:monospace;background:#111;color:#eee;margin:0}"
    "header{padding:10px;background:#222;position:sticky;top:0}"
    "button{margin-right:8px} pre{padding:10px;white-space:pre-wrap}</style>"
    "</head><body>"
    "<header>"
    "<button onclick='fetch(\"/api/reconnect_tcp\",{method:\"POST\"})'>Reconnect TCP</button>"
    "<button onclick='fetch(\"/api/reconnect_wifi\",{method:\"POST\"})'>Reconnect WiFi</button>"
    "<button onclick='fetch(\"/api/clear\",{method:\"POST\"})'>Clear</button>"
    "<span id='st'></span>"
    "</header>"
    "<pre id='log'>loading...</pre>"
    "<script>"
    "async function tick(){"
    " let s=await (await fetch('/api/status')).text();"
    " document.getElementById('st').textContent=s;"
    " let t=await (await fetch('/api/log')).text();"
    " document.getElementById('log').textContent=t;"
    "}"
    "setInterval(tick,1000); tick();"
    "</script></body></html>";
  web.send(200, "text/html", html);
}

static void handleClear() {
  for (uint16_t i = 0; i < LOG_LINES; i++) logBuf[i] = "";
  web.send(200, "text/plain", "ok");
}
static void handleReconnectTcp() {
  if (tcpClient) tcpClient.stop();
  web.send(200, "text/plain", "ok");
  logLine("DBG: TCP stop requested");
}
static void handleReconnectWiFi() {
  WiFi.disconnect();
  WiFi.begin(STA_SSID, STA_PASS);
  web.send(200, "text/plain", "ok");
  logLine("DBG: WiFi reconnect requested");
}
static void handleLog() { web.send(200, "text/plain", getLogs(90)); }

// ===================== CRC Modbus =====================
static uint16_t modbusCrc16(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}
static bool crcOk(const uint8_t* frame, uint16_t len) {
  if (len < 4) return false;
  uint16_t calc = modbusCrc16(frame, len - 2);
  uint16_t rx   = (uint16_t)frame[len - 2] | ((uint16_t)frame[len - 1] << 8);
  return calc == rx;
}
static String toHex(const uint8_t* p, uint16_t n) {
  static const char* h = "0123456789ABCDEF";
  String s;
  for (uint16_t i = 0; i < n; i++) {
    uint8_t b = p[i];
    s += h[b >> 4];
    s += h[b & 0x0F];
    s += ' ';
  }
  return s;
}

// ===================== RS485 driver =====================
static uint8_t rtuBuf[MAX_RTU];
static uint16_t rtuLen = 0;
static bool inFrame = false;
static uint32_t lastByteUs = 0;

static inline void rs485SetTx(bool tx) {
  digitalWrite(DE_RE, tx ? HIGH : LOW);
  if (tx) delayMicroseconds(60);
}

static void rs485Send(const uint8_t* data, uint16_t len) {
  // Driver enable + guard time antes do 1º byte
  rs485SetTx(true);

  Serial.write(data, len);
  Serial.flush();

  // Guard time após o último byte (evita cortar o último stop-bit em alguns módulos)
  delayMicroseconds(BYTE_TIME_US * 2);

  // Turnaround seguro
  delayMicroseconds(1500);

  rs485SetTx(false);

  // Ignore proporcional ao frame para evitar eco/lixo imediato
  ignoreRxUntilUs = micros() + (uint32_t)len * BYTE_TIME_US + MODBUS_GAP_US + 2000;
}

static bool rs485CollectFrame(uint8_t* out, uint16_t* outLen) {
  // Se acabamos de transmitir, ignorar RX (e limpar FIFO)
  if ((int32_t)(micros() - ignoreRxUntilUs) < 0) {
    while (Serial.available()) (void)Serial.read();
    return false;
  }

  // 1) Coleta bytes
  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();

    // Ignora lixo “idle” fora de frame
    if (!inFrame && b == 0x00) {
      continue;
    }

    if (!inFrame) {
      inFrame = true;
      rtuLen = 0;
    }

    if (rtuLen < MAX_RTU) {
      rtuBuf[rtuLen++] = b;
    }
    lastByteUs = micros();
  }

  // 2) Fecha frame por GAP
  if (inFrame && (uint32_t)(micros() - lastByteUs) > MODBUS_GAP_US) {
    inFrame = false;

    // descarta lixo pequeno (Modbus mínimo = 4 bytes)
    if (rtuLen < 4) {
      rtuLen = 0;
      return false;
    }

    memcpy(out, rtuBuf, rtuLen);
    *outLen = rtuLen;
    rtuLen = 0;
    return true;
  }

  return false;
}



// ===================== Keepalive (para Wi-Fi congestionado/NAT) =====================
// Envia um HELLO leve quando não há tráfego Modbus por um tempo.
// Isso mantém o socket "quente" e reduz reconexões em redes carregadas.
static const uint32_t KEEPALIVE_MS = 15000;

// Payload HELLO
static const uint8_t HELLO_PAYLOAD[5] = {'H','E','L','L','O'};
// ===================== TCP framing =====================
static const uint8_t TCP_RECV_RETRY = 3; // tenta 3x antes de derrubar

static const uint32_t TCP_BYTE_TIMEOUT_MS  = 8000;
static const uint32_t TCP_RESYNC_TIMEOUT_MS = 8000;

static bool tcpReadByte(WiFiClient& c, uint8_t* b, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (true) {
    if (!c.connected()) return false;
    if (c.available() > 0) { *b = (uint8_t)c.read(); return true; }
    if (millis() - t0 > timeoutMs) return false;
    yield();
  }
}

static bool tcpResyncToMagic(WiFiClient& c, uint32_t timeoutMs) {
  uint8_t b = 0;
  uint32_t t0 = millis();
  while (millis() - t0 <= timeoutMs) {
    if (!tcpReadByte(c, &b, timeoutMs)) return false;
    if (b == MAGIC0) {
      uint8_t b2 = 0;
      if (!tcpReadByte(c, &b2, timeoutMs)) return false;
      if (b2 == MAGIC1) return true;
    }
  }
  return false;
}

static bool tcpRecvFrame(WiFiClient& c, uint8_t* seq, uint8_t* payload, uint16_t* len) {
  if (!tcpResyncToMagic(c, TCP_RESYNC_TIMEOUT_MS)) return false;

  uint8_t hdr[3];
  for (int i = 0; i < 3; i++) {
    if (!tcpReadByte(c, &hdr[i], TCP_BYTE_TIMEOUT_MS)) return false;
  }

  *seq = hdr[0];
  *len = (uint16_t)hdr[1] | ((uint16_t)hdr[2] << 8);
  if (*len > MAX_RTU) return false;

  for (uint16_t i = 0; i < *len; i++) {
    if (!tcpReadByte(c, &payload[i], TCP_BYTE_TIMEOUT_MS)) return false;
  }
  return true;
}

static bool tcpSendFrame(WiFiClient& c, uint8_t seq, const uint8_t* payload, uint16_t len) {
  if (!c.connected()) return false;
  uint8_t hdr[5] = {MAGIC0, MAGIC1, seq, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
  if (c.write(hdr, 5) != 5) return false;
  if (len && c.write(payload, len) != len) return false;
  return true;
}



// ===================== Frame sanity (reduz falso-positivo de frame) =====================
static bool isValidAddr(uint8_t a) { return (a >= 1 && a <= 247); }
static bool isLikelyModbusReq(const uint8_t* f, uint16_t n) {
  if (n < 4) return false;
  if (!isValidAddr(f[0])) return false;
  uint8_t fc = f[1];
  // Funções comuns: 0x03,0x04,0x06,0x10,0x2B(43)
  if (!(fc == 0x03 || fc == 0x04 || fc == 0x06 || fc == 0x10 || fc == 0x2B)) return false;

  // Requests típicos: 0x03/0x04/0x06 => 8 bytes; 0x10 => >=9; 0x2B/0x0E => 7 bytes
  if ((fc == 0x03 || fc == 0x04 || fc == 0x06) && n != 8) return false;
  if (fc == 0x2B && n != 7) return false;

  return true;
}
// ===================== Stats + last frames =====================
static uint32_t lastGoodTcpMs = 0;
static uint32_t tcpIn = 0, tcpOut = 0;
static uint32_t rtuIn = 0, rtuOut = 0;
static uint32_t failR = 0, failS = 0;
static uint32_t crcReqBad = 0, crcRespBad = 0;
static uint8_t seqCounter = 0;

static uint16_t lastReqLen = 0, lastRespLen = 0;
static uint8_t lastReq[MAX_RTU], lastResp[MAX_RTU];

static void handleStatus() {
  String s;
  s += "Uptime(ms)=" + String(millis() - bootMs);
  s += " | STA=" + String((WiFi.status() == WL_CONNECTED) ? "" : "DOWN");
  s += " IP=" + WiFi.localIP().toString();
  s += " RSSI=" + String(WiFi.RSSI());
  s += " | TCP=" + String(tcpClient.connected() ? "CONNECTED" : "DISCONNECTED");
  s += " | tcp_in=" + String(tcpIn) + " tcp_out=" + String(tcpOut);
  s += " rtu_in=" + String(rtuIn) + " rtu_out=" + String(rtuOut);
  s += " failR=" + String(failR) + " failS=" + String(failS);
  s += " crcReqBad=" + String(crcReqBad) + " crcRespBad=" + String(crcRespBad);
  web.send(200, "text/plain", s);
}

static void handleLast() {
  String out;
  out += "LAST REQ (" + String(lastReqLen) + "): " + toHex(lastReq, lastReqLen) + "\n";
  out += "CRC_OK=" + String(crcOk(lastReq, lastReqLen) ? "YES" : "NO") + "\n\n";
  out += "LAST RESP (" + String(lastRespLen) + "): " + toHex(lastResp, lastRespLen) + "\n";
  out += "CRC_OK=" + String((lastRespLen >= 4) ? (crcOk(lastResp, lastRespLen) ? "YES" : "NO") : "NO") + "\n";
  web.send(200, "text/plain", out);
}

// ===================== Connectivity helpers =====================
static void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.disconnect();
  WiFi.begin(STA_SSID, STA_PASS);
}

static void ensureTcp() {
  if (tcpClient.connected()) return;

  tcpClient.stop();
  tcpClient.setNoDelay(true);

  uint32_t t0 = millis();
  while (!tcpClient.connect(serverIP, TCP_PORT)) {
    web.handleClient();
    yield();
    delay(200);
    if (millis() - t0 > 8000) break;
  }

  if (tcpClient.connected()) {
    tcpClient.setNoDelay(true);
    lastGoodTcpMs = millis();
    logLine("TCP: connected to server");
  }
}

// ===================== SETUP/LOOP =====================
void setup() {
  bootMs = millis();
  lastActivityMs = millis();

  // Serial = RS485 (sem debug)
  Serial.begin(RS485_BAUD);
  Serial.setRxBufferSize(1024);

  pinMode(DE_RE, OUTPUT);
  pinMode(HALF_OR_FULL_PIN, OUTPUT);

  // Mantido LOW (conforme seu hardware que sempre funcionou)
  digitalWrite(HALF_OR_FULL_PIN, LOW);
  digitalWrite(DE_RE, LOW); // RX

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.begin(STA_SSID, STA_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(100);
    yield();
  }
  logLine("BOOT: WiFi started");

  web.on("/", handleRoot);
  web.on("/api/status", handleStatus);
  web.on("/api/log", handleLog);
  web.on("/api/last", handleLast);
  web.on("/api/clear", HTTP_POST, handleClear);
  web.on("/api/reconnect_tcp", HTTP_POST, handleReconnectTcp);
  web.on("/api/reconnect_wifi", HTTP_POST, handleReconnectWiFi);
  web.begin();
  logLine("BOOT: Web debug ready (AP GW-MASTER)");

  ensureTcp();
}

void loop() {
  if (tcpClient.connected() && (millis() - lastActivityMs > KEEPALIVE_INTERVAL_MS)) {
    tcpSendKeepAlive(tcpClient);
    lastActivityMs = millis();
  }

  // Mantém serviços vivos
  web.handleClient();
  ensureWiFi();
  ensureTcp();

  // Não mate TCP por ociosidade durante testes (10 min)
  if (tcpClient.connected() && (millis() - lastGoodTcpMs > 600000UL)) {
    logLine("TCP: idle timeout -> stop");
    tcpClient.stop();
    return;
  }

  // 1) Captura frame do RS485 (Smartlogger->túnel)
  uint8_t req[MAX_RTU];
  uint16_t reqLen = 0;

  if (!rs485CollectFrame(req, &reqLen)) {
    web.handleClient();
    ensureWiFi();
    ensureTcp();

    // Keepalive: só quando conectado e sem atividade por KEEPALIVE_MS
    if (tcpClient.connected() && (millis() - lastActivityMs > KEEPALIVE_MS)) {
      uint8_t seq = ++seqCounter;
      if (tcpSendFrame(tcpClient, seq, HELLO_PAYLOAD, 5)) {
        tcpOut++;
  lastActivityMs = millis();
        lastGoodTcpMs = millis();
        lastActivityMs = millis();
        logLine("TCP: keepalive HELLO");
      }
    }

    yield();
    return;
  }

  // 2) CRC inválido: ignora (não derruba TCP)
  if (!crcOk(req, reqLen)) { crcReqBad++; }

  // Só aqui conta e salva LAST REQ
  rtuIn++;
  lastActivityMs = millis();
  lastReqLen = reqLen;
  memcpy(lastReq, req, reqLen);

  // 3) Envia via TCP
  uint8_t seq = ++seqCounter;
  if (!tcpSendFrame(tcpClient, seq, req, reqLen)) {
    failS++;
    logLine("TCP: send fail -> stop");
    tcpClient.stop();
    return;
  }

  tcpOut++;
  lastActivityMs = millis();
  lastActivityMs = millis();
  lastGoodTcpMs = millis();

  // 4) Recebe resposta via TCP
  uint8_t respSeq = 0;
  uint8_t resp[MAX_RTU];
  uint16_t respLen = 0;

  bool got = false;
  for (uint8_t attempt = 0; attempt < TCP_RECV_RETRY; attempt++) {
    if (tcpRecvFrame(tcpClient, &respSeq, resp, &respLen)) { got = true; break; }
    yield();
  }
  if (!got) {
    failR++;
    logLine("TCP: recv fail x" + String(TCP_RECV_RETRY) + " -> stop");
    tcpClient.stop();
    return;
  }

  tcpIn++;
  lastActivityMs = millis();
  lastActivityMs = millis();
  lastGoodTcpMs = millis();

  // guarda lastResp (mesmo que len=0)
  lastRespLen = respLen;
  if (respLen) memcpy(lastResp, resp, respLen);

  // 5) Se veio resposta, valida CRC (se inválido, ignora)
  if (respLen >= 4 && !crcOk(resp, respLen)) { crcRespBad++; }

  // 6) Injeta resposta no RS485 (túnel->Smartlogger)
  if (respLen > 0) {
    rs485Send(resp, respLen);
    rtuOut++;
    logLine("RS485: sent resp len=" + String(respLen));
  }
}
