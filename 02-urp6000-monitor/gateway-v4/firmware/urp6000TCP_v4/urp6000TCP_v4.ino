#include "site_private.h"
/*
  ESP8266 - URP6000 (Pextron) Modbus RTU -> espelho em Holding Registers -> Modbus TCP server

  Bibliotecas (Alexander Emelianov):
    - ModbusRTU.h
    - ModbusIP_ESP8266.h

  Estratégia:
    - RTU Master lê o relé (serial 1, RS485) e grava em espelho (Holding Registers)
    - TCP Server expõe Holding Registers com os MESMOS endereços (espelho)

  Registradores espelhados (URP6000):
    - Identificador (para "iniciar" Modbus conforme manual): 0x0088 (136) len=2
    - Relações: 0x003C..0x003E (0060..0062) len=3  (RTC_FN, RTC_D, RTP)
    - Correntes/Tensões: 0x02BC..0x02C3 (0700..0707) len=8
    - Frequências e cosφ: 0x02C7..0x02CB (0711..0715) len=5
    - Potência ativa por fase: 0x02CD..0x02D2 (0717..0722) len=6  (3x int32)

  Observação: este sketch NÃO converte unidades. Ele só "espelha" registradores.
*/

#include <ESP8266WiFi.h>
#include <ModbusRTU.h>
#include <ModbusIP_ESP8266.h>

// -------------------- WiFi --------------------
static const char* WIFI_SSID = SITE_WIFI_SSID;
static const char* WIFI_PASS = SITE_WIFI_PASSWORD;

IPAddress local_IP(SITE_IP_LOCAL_IP);
IPAddress gateway (SITE_IP_GATEWAY);
IPAddress subnet  (255, 255, 255, 0);

// -------------------- RS485 (ajuste conforme seu hardware) --------------------
#define LED_PIN           LED_BUILTIN
#define BTN_PIN           0

// Pino DE/RE ligado ao transceiver RS485 (ex: MAX485, SN75176)
static const uint8_t PIN_RS485_DE_RE = 12;   // D6 (GPIO12) - AJUSTE se necessário

// Se sua placa tiver seletor HALF/FULL (como no seu sketch do DTSU), você pode manter.
// Se não tiver, comente estas 2 linhas e remova o uso no setup.
static const uint8_t HALF_OR_FULL_PIN = 13;  // D7 (GPIO13) - AJUSTE se necessário
static const bool    USE_HALF_DUPLEX_SELECTOR = true;

// -------------------- Modbus --------------------
static const uint8_t  SLAVE_ID = 1;          // End.1 do URP6000 (Modbus RTU)

static const uint16_t REG_ID_START   = 0x0088; // 136
static const uint16_t REG_ID_LEN     = 2;

static const uint16_t REG_RAT_START  = 0x003C; // 0060
static const uint16_t REG_RAT_LEN    = 3;      // RTC_FN, RTC_D, RTP

static const uint16_t REG_MEAS_START = 0x02BC; // 0700
static const uint16_t REG_MEAS_LEN   = 8;      // 0700..0707

static const uint16_t REG_FQ_START   = 0x02C7; // 0711
static const uint16_t REG_FQ_LEN     = 5;      // 0711..0715 (FREQ, FREQ_B, cosA, cosB, cosC)

static const uint16_t REG_P_START    = 0x02CD; // 0717
static const uint16_t REG_P_LEN      = 6;      // 0717..0722

// "registradores extras" do ESP
static const uint16_t REG_STATUS = 0x0100;    // 0=sem dados, 1=dados válidos
static const uint16_t REG_SEQ    = 0x0101;    // contador de atualização
static const uint16_t REG_AGE_S  = 0x0102;    // idade do snapshot (segundos)

// Timings
static const uint32_t WAIT_ID_PERIOD_MS  = 2000;
static const uint32_t CYCLIC_PERIOD_MS   = 1000;
static const uint8_t  CYCLIC_MAX_RETRIES = 2;
static const uint8_t  ID_REFRESH_EVERY_CYCLES = 2;  // a cada N ciclos, força IDCHECK no início


static const uint32_t RTU_TXN_TIMEOUT_MS = 700; // ajuste: 300~1000ms
uint32_t rtuTxnStart = 0;

// -------------------- Objetos --------------------
ModbusRTU mbRTU;
ModbusIP  mbTCP;

// -------------------- Estado --------------------
enum SystemState : uint8_t { S_WAIT_ID = 0, S_CYCLIC };
SystemState state = S_WAIT_ID;

enum LedState { LED_OFF = 0, LED_WIFI_OK, LED_RTU_ERROR, LED_ALL_OK };
LedState ledState = LED_OFF;

struct RtuReq {
  bool busy = false;
  uint16_t addr = 0;
  uint16_t len  = 0;
  uint16_t* buf = nullptr;
  bool ok = false;
} rtu;

// buffers temporários (maior bloco = 8 regs)
uint16_t buf[16];
uint16_t idType = 0;
uint16_t idVer  = 0;

uint32_t tNextWaitId = 0;

// Ciclo
enum CyStep : uint8_t { CY_IDCHECK=0, CY_RAT, CY_MEAS, CY_FQ, CY_P };
CyStep cyStep = CY_IDCHECK;
uint8_t cyRetries = 0;
uint16_t cycleCount = 0;
uint32_t tNextCycleStart = 0;

static inline void rs485TxEnable(bool en) {
  digitalWrite(PIN_RS485_DE_RE, en ? HIGH : LOW);
}

void tcpReserveRegs() {
  // IMPORTANTE: manter endereços 'extras' baixos evita alocação grande em algumas libs e o erro 2.

  mbTCP.addHreg(REG_ID_START, 0, REG_ID_LEN);
  mbTCP.addHreg(REG_RAT_START, 0, REG_RAT_LEN);
  mbTCP.addHreg(REG_MEAS_START, 0, REG_MEAS_LEN);
  mbTCP.addHreg(REG_FQ_START,   0, REG_FQ_LEN);
  mbTCP.addHreg(REG_P_START,    0, REG_P_LEN);

  mbTCP.addHreg(REG_STATUS, 0);
  mbTCP.addHreg(REG_SEQ, 0);
  mbTCP.addHreg(REG_AGE_S, 0);
}

static inline void mirrorWriteBlock(uint16_t start, const uint16_t* data, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) mbTCP.Hreg(start + i, data[i]);
}

bool rtuReadHreg(uint16_t addr, uint16_t* dst, uint16_t len) {
  if (rtu.busy) return false;

  rtu.busy = true;
  rtu.addr = addr;
  rtu.len  = len;
  rtu.buf  = dst;
  rtu.ok   = false;
  rtuTxnStart = millis();

  auto cb = [](Modbus::ResultCode event, uint16_t /*transactionId*/, void* /*data*/) -> bool {
    rtu.ok = (event == Modbus::EX_SUCCESS);
    rtu.busy = false;
    return true;
  };

  bool started = mbRTU.readHreg(SLAVE_ID, addr, dst, len, cb);
  if (!started) {
    rtu.busy = false;
    rtu.ok = false;
    return false;
  }
  return true;
}

void gotoWaitId() {
  state = S_WAIT_ID;
  tNextWaitId = 0;
  cyStep = CY_IDCHECK;
  cyRetries = 0;
  tNextCycleStart = 0;
  mbTCP.Hreg(REG_STATUS, 0x0000);
}

bool cyclicStartStep(CyStep step) {
  switch(step) {
    case CY_IDCHECK: return rtuReadHreg(REG_ID_START,   buf, REG_ID_LEN);
    case CY_RAT:     return rtuReadHreg(REG_RAT_START,  buf, REG_RAT_LEN);
    case CY_MEAS:    return rtuReadHreg(REG_MEAS_START, buf, REG_MEAS_LEN);
    case CY_FQ:      return rtuReadHreg(REG_FQ_START,   buf, REG_FQ_LEN);
    case CY_P:       return rtuReadHreg(REG_P_START,    buf, REG_P_LEN);
    default:         return false;
  }
}

void cyclicCommitStep(CyStep step) {
  switch(step) {
    case CY_IDCHECK: mirrorWriteBlock(REG_ID_START,   buf, REG_ID_LEN);   break;
    case CY_RAT:     mirrorWriteBlock(REG_RAT_START,  buf, REG_RAT_LEN);  break;
    case CY_MEAS:    mirrorWriteBlock(REG_MEAS_START, buf, REG_MEAS_LEN); break;
    case CY_FQ:      mirrorWriteBlock(REG_FQ_START,   buf, REG_FQ_LEN);   break;
    case CY_P:       mirrorWriteBlock(REG_P_START,    buf, REG_P_LEN);    break;
  }
}

void handleLed() {
  static uint32_t tLed = 0;
  static bool ledOn = false;
  uint32_t now = millis();

  switch (ledState) {
    case LED_OFF:
      digitalWrite(LED_PIN, HIGH);
      break;
    case LED_WIFI_OK:  // 2 Hz
      if (now - tLed >= 250) { tLed = now; ledOn = !ledOn; digitalWrite(LED_PIN, ledOn ? LOW : HIGH); }
      break;
    case LED_RTU_ERROR: // 1 Hz
      if (now - tLed >= 500) { tLed = now; ledOn = !ledOn; digitalWrite(LED_PIN, ledOn ? LOW : HIGH); }
      break;
    case LED_ALL_OK:
      digitalWrite(LED_PIN, LOW);
      break;
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  pinMode(BTN_PIN, INPUT_PULLUP);

  pinMode(PIN_RS485_DE_RE, OUTPUT);
  rs485TxEnable(false);

  if (USE_HALF_DUPLEX_SELECTOR) {
    pinMode(HALF_OR_FULL_PIN, OUTPUT);
    digitalWrite(HALF_OR_FULL_PIN, LOW); // Half duplex
  }

  // Serial do RS485
  Serial.begin(115200, SERIAL_8N1);

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  // WiFi.config(local_IP, gateway, subnet);  // manter comentado (DHCP)
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(200);
  }

  // Modbus TCP server
  mbTCP.server(502);
  tcpReserveRegs();

  // Modbus RTU master
  mbRTU.begin(&Serial, PIN_RS485_DE_RE);
  mbRTU.master();

  gotoWaitId();
}

void loop() {
  // AGE counter
  static uint32_t tAge = 0;
  if (millis() - tAge >= 1000) {
    tAge += 1000;
    uint16_t age = mbTCP.Hreg(REG_AGE_S);
    if (age < 65535) mbTCP.Hreg(REG_AGE_S, age + 1);
  }

  // WiFi watchdog simples
  static uint32_t wifiRetryTimer = 0;
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - wifiRetryTimer > 5000) {
      wifiRetryTimer = millis();
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
    ledState = LED_OFF;
  }

  uint32_t now = millis();

  mbTCP.task();
  mbRTU.task();
  yield();

  // timeout de transação RTU
  if (rtu.busy && (millis() - rtuTxnStart) > RTU_TXN_TIMEOUT_MS) {
    rtu.busy = false;
    rtu.ok = false;
    ledState = LED_RTU_ERROR;
    gotoWaitId();
  }

  // detecta fim de transação (callback liberou busy)
  static bool lastBusy = false;
  if (lastBusy && !rtu.busy) {
    if (state == S_WAIT_ID) {
      if (rtu.ok) {
        idType = buf[0];
        idVer  = buf[1];
        cyclicCommitStep(CY_IDCHECK); // espelha 0x0088..0x0089

        // se vier tudo zero, provavelmente não respondeu direito
        if ((idType != 0x0000) || (idVer != 0x0000)) {
          state = S_CYCLIC;
          cyStep = CY_IDCHECK;
          cyRetries = 0;
          tNextCycleStart = now;
        }
      }
      // se falhar, tenta de novo
    } else if (state == S_CYCLIC) {
      if (!rtu.ok) {
        if (cyRetries < CYCLIC_MAX_RETRIES) {
          cyRetries++;
          // repete mesmo step
        } else {
          ledState = LED_RTU_ERROR;
          gotoWaitId();
        }
      } else {
        cyclicCommitStep(cyStep);
        cyRetries = 0;

        if (cyStep == CY_P) {
          // ciclo completo
          mbTCP.Hreg(REG_STATUS, 0x0001);
          mbTCP.Hreg(REG_SEQ, (mbTCP.Hreg(REG_SEQ) + 1) & 0xFFFF);
          mbTCP.Hreg(REG_AGE_S, 0);

          cyStep = CY_IDCHECK;
          tNextCycleStart = now + CYCLIC_PERIOD_MS;
        } else {
          cyStep = (CyStep)((uint8_t)cyStep + 1);
        }
      }
    }
  }
  lastBusy = rtu.busy;

  if (WiFi.status() == WL_CONNECTED && state != S_CYCLIC) ledState = LED_WIFI_OK;

  // executa estado atual
  if (state == S_WAIT_ID) {
    if (now >= tNextWaitId && !rtu.busy) {
      tNextWaitId = now + WAIT_ID_PERIOD_MS;
      rtuReadHreg(REG_ID_START, buf, REG_ID_LEN);
    }
  } else { // S_CYCLIC
    ledState = LED_ALL_OK;
    if ((cyStep == CY_IDCHECK || cyStep == CY_RAT) && now < tNextCycleStart) {
      // espera início do ciclo
    } else if (!rtu.busy) {
      cyclicStartStep(cyStep);
    }
  }

  handleLed();
}
