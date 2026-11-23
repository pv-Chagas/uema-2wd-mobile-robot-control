/*
  Firmware do robô móvel 2WD - controle de posição (0 a 1 m), comunicação por Bluetooth LE ou USB
  Placa: ESP32-C3 SuperMini | Driver: L298N | Encoders ópticos (1 canal por roda)

  Bibliotecas (Gerenciador de Bibliotecas da Arduino IDE):
    - "NimBLE-Arduino" de h2zero, versão 2.x (API conferida na 2.5.2)
    - "ArduinoJson" de Benoit Blanchon, versão 7.x
  Núcleo ESP32 para Arduino 3.x (também compila no 2.x). Preferences já vem com o núcleo.
  Ferramentas > "USB CDC On Boot" = Enabled (necessário para a conexão USB/Serial da IHM).

  Obs.: com a conexão USB, a porta Serial é o canal de dados. Linhas que não são JSON
  (como a mensagem de boot) aparecem no console da IHM.
*/

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// ======================= CONFIGURAÇÕES =======================

// --- Bluetooth ---
const char* BLE_NAME = "Robo-SQUIRTLE";   // TROQUE por um nome único da sua equipe
#define SERVICE_UUID "7b1e0001-6d0f-4c8a-9a52-3f4d2c8b9a10"
#define RX_UUID      "7b1e0002-6d0f-4c8a-9a52-3f4d2c8b9a10"
#define TX_UUID      "7b1e0003-6d0f-4c8a-9a52-3f4d2c8b9a10"
const uint32_t USB_TIMEOUT_MS = 3000;     // sem ping da IHM por esse tempo, o USB é considerado desligado

// --- Pinos do driver (L298N: ENA/ENB com o jumper removido | TB6612: PWMA/PWMB) ---
const int PIN_L_PWM = 4, PIN_L_IN1 = 5, PIN_L_IN2 = 6;    // motor esquerdo
const int PIN_R_PWM = 3, PIN_R_IN1 = 0, PIN_R_IN2 = 1;    // motor direito
const bool INVERT_LEFT  = false;          // true se a roda girar ao contrário
const bool INVERT_RIGHT = false;

// Encoders (módulo óptico com pull-up próprio, 3,3 V; GPIO8 é pino de strapping).
const int PIN_ENC_L = 7, PIN_ENC_R = 8;

// --- Servo SG90 e sensor HC-SR04 (3ª avaliação) ---
// GPIO20 e GPIO21 são os pinos de UART0 da C3, livres porque o Serial usa a USB nativa.
const int PIN_SERVO = 10;                 // sinal do SG90 (laranja/amarelo)
const int PIN_TRIG  = 21;                 // TRIG do HC-SR04
const int PIN_ECHO  = 20;                 // ECHO do HC-SR04, SEMPRE por divisor 1 kΩ / 2 kΩ (5 V -> 3,3 V)
const int   SERVO_MIN_US = 500;           // largura de pulso em 0 grau (calibre com o seu SG90)
const int   SERVO_MAX_US = 2400;          // largura de pulso em 180 graus
const bool  SERVO_INVERT = false;         // true se o servo estiver montado ao contrário
const float SERVO_TRIM_DEG = 0;           // ajuste fino do centro (graus)
const int   SERVO_BITS = 14;              // resolução do PWM de 50 Hz do servo
const int   MAX_SCAN_PTS = 91;            // 180 graus em passos de 2 graus

// --- Geometria e odometria ---
const float WHEEL_DIAMETER_M = 0.065f;    // MEÇA o diâmetro da sua roda
const int   ENC_HOLES        = 20;        // furos do disco encoder
const int   PULSES_PER_REV   = ENC_HOLES * 2;   // contamos subida e descida (CHANGE)
const float M_PER_PULSE      = 3.14159265f * WHEEL_DIAMETER_M / PULSES_PER_REV;
const uint32_t ENC_DEBOUNCE_US = 300;
// A distância entre as rodas (bitola) é ajustada pela IHM (SET.wb, em mm).

// --- Controle ---
const uint32_t CTRL_PERIOD_MS = 20;       // malha de controle e odometria a 50 Hz
const float PWM_MIN_PCT   = 30.0f;        // % mínimo para vencer o atrito (CALIBRE; 0 desativa)
const float D_FILTER      = 0.7f;         // filtro do termo derivativo

// --- Telemetria (o registro completo é baixado depois, então a ao vivo pode ser mais leve) ---
const uint32_t TELEM_RUN_MS    = 50;      // 20 Hz durante a execução
const uint32_t TELEM_MANUAL_MS = 100;     // 10 Hz na condução manual
const uint32_t TELEM_IDLE_MS   = 200;     // 5 Hz parado
const uint16_t TELEM_BACKLOG   = 300;     // não acumula telemetria velha na fila de envio

// --- PWM ---
const int PWM_FREQ = 1000, PWM_BITS = 10, PWM_MAX = (1 << PWM_BITS) - 1;

// --- Registro (log) na memória do ESP32 ---
const int MAX_LOG   = 1500;               // 30 s a 50 Hz (20 bytes por amostra = 30 KB)
const int LOG_CHUNK = 16;                 // amostras por mensagem ao enviar o registro

// --- Filas ---
const uint16_t TXQ_SIZE = 4096;           // fila de saída (bytes)
const uint16_t RX_MAX   = 520;            // maior linha aceita (o "save" com ganhos tem ~400 bytes)
const uint8_t  LQ_SIZE  = 4;              // linhas BLE completas aguardando processamento

#define DEBUG_SERIAL false                // imprime a odometria no Serial (atrapalha a conexão USB: deixe false)

// ======================= VARIÁVEIS =======================
Preferences prefs;
NimBLEServer*         bleServer = nullptr;
NimBLECharacteristic* txChar    = nullptr;

// Estado dos enlaces. Os callbacks BLE rodam em outra tarefa: só mexem nestas variáveis e nas
// filas; todo o resto (controle, motores, JSON) roda apenas no loop().
volatile bool     bleReady = false;       // IHM conectada por BLE e com notificações ativas
volatile bool     evDropped = false;      // um enlace caiu
volatile bool     cfgPending = false;     // enviar a configuração ao conectar
volatile uint16_t mtuPayload = 20;        // bytes úteis por notificação (MTU - 3)
bool     usbActive = false;               // IHM conectada por USB (ping recente)
uint32_t lastUsbRx = 0;
inline bool linkUp() { return usbActive || bleReady; }

char lineQ[LQ_SIZE][RX_MAX];              // linhas BLE (produtor: callback, consumidor: loop)
volatile uint8_t lqHead = 0, lqTail = 0;
char rxBuild[RX_MAX]; uint16_t rxLen = 0; bool rxOver = false;
char usbBuild[RX_MAX]; uint16_t usbLen = 0; bool usbOver = false;

uint8_t  txq[TXQ_SIZE];                   // fila de saída (produtor e consumidor: loop)
uint16_t txHead = 0, txTail = 0;
uint32_t txDrops = 0;

volatile int32_t encL = 0, encR = 0;
volatile int8_t  dirL = 1, dirR = 1;
volatile uint32_t lastEdgeL = 0, lastEdgeR = 0, perL = 0, perPrevL = 0, perR = 0, perPrevR = 0;

enum RunState { IDLE, RUNNING, STOPPED, DONE, MANUAL, SCANNING };
RunState state = IDLE;

struct Params {
  char  mode[4] = "PID";
  bool  ol = false, useP = true, useI = true, useD = true;   // calculados em setMode()
  float sp = 0.5f, kp = 0, ki = 0, kd = 0, vel = 50, time = 3;
} P;

struct Settings {
  float tolMm = 10, tmax = 15, pwm = 100, hg = 300, wb = 130, vmax = 0.4f;
  int rm = 1;                             // correção de rota: 0 desligada, 1 pela diferença entre rodas, 2 PID por roda
} SET;
float GL[3] = {150, 300, 0}, GR[3] = {150, 300, 0};      // PID de velocidade: roda esquerda e direita (kp, ki, kd)

// Ganhos do controlador de posição por modo: P, I, D, PI, PD, PID
const char* MODE_NAMES[6] = {"P", "I", "D", "PI", "PD", "PID"};
float G[6][3] = {{120,0,0},{0,20,0},{0,0,10},{120,20,0},{120,0,10},{120,20,10}};

// Amostra compacta (20 bytes): t em ms; sp, x e y em 0,2 mm; ul/ur em 0,1 %; vl/vr em mm/s; dw em 0,1 mm; th em 0,1 grau
struct Sample { uint16_t t, sp; int16_t x, ul, ur, vl, vr, dw, y, th; };
Sample logBuf[MAX_LOG];
int  logN = 0;
bool logSending = false; int logPos = 0;

// Pose por odometria
float px = 0, py = 0, th = 0;             // m, m, rad
int32_t lastCl = 0, lastCr = 0;
float vL = 0, vR = 0, speed = 0;          // m/s

struct WheelPid { float i = 0, ePrev = 0, dF = 0; } W[2];
float integ = 0, ePrev = 0, dFilt = 0;
float ulOut = 0, urOut = 0;
uint32_t tStart = 0, lastCtrl = 0, lastTelem = 0, lastOdo = 0, lastDbg = 0, manualUntil = 0;
float settleTime = 0;

// ======================= ENCODERS E ODOMETRIA =======================
void IRAM_ATTR isrL() {
  uint32_t n = micros(), dt = n - lastEdgeL; if (dt < ENC_DEBOUNCE_US) return;
  lastEdgeL = n; perPrevL = perL; perL = dt; encL += dirL;
}
void IRAM_ATTR isrR() {
  uint32_t n = micros(), dt = n - lastEdgeR; if (dt < ENC_DEBOUNCE_US) return;
  lastEdgeR = n; perPrevR = perR; perR = dt; encR += dirR;
}
float posL() { noInterrupts(); int32_t c = encL; interrupts(); return c * M_PER_PULSE; }
float posR() { noInterrupts(); int32_t c = encR; interrupts(); return c * M_PER_PULSE; }

// Velocidade pelo intervalo entre as últimas bordas: resolução muito melhor que contar pulsos numa janela.
float edgeSpeed(uint32_t last, uint32_t a, uint32_t b, int8_t dir) {
  uint32_t age = micros() - last;
  if (age > 150000UL || a == 0 || b == 0) return 0;      // sem pulsos recentes: parado
  float per = 0.5f * (a + b);
  if (age > per) per = age;                              // desacelerando: usa o tempo desde a última borda
  return dir * M_PER_PULSE / (per * 1e-6f);
}

void odoStep() {                                         // chamada a cada CTRL_PERIOD_MS
  int32_t cl, cr; uint32_t l1, a1, b1, l2, a2, b2;
  noInterrupts(); cl = encL; cr = encR; l1 = lastEdgeL; a1 = perL; b1 = perPrevL; l2 = lastEdgeR; a2 = perR; b2 = perPrevR; interrupts();
  float dL = (cl - lastCl) * M_PER_PULSE, dR = (cr - lastCr) * M_PER_PULSE;
  lastCl = cl; lastCr = cr;
  float ds = 0.5f * (dL + dR), dth = (dR - dL) / (SET.wb / 1000.0f), tm = th + 0.5f * dth;
  px += ds * cosf(tm); py += ds * sinf(tm); th += dth;
  vL += 0.5f * (edgeSpeed(l1, a1, b1, dirL) - vL);
  vR += 0.5f * (edgeSpeed(l2, a2, b2, dirR) - vR);
  speed = 0.5f * (vL + vR);
}
void resetOdometry() {
  noInterrupts(); encL = 0; encR = 0; perL = perPrevL = perR = perPrevR = 0; interrupts();
  lastCl = lastCr = 0; px = py = th = 0; vL = vR = speed = 0;
}

// ======================= PWM (compatível com núcleos 2.x e 3.x) =======================
void pwmAttach(int pin, int ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(ch, PWM_FREQ, PWM_BITS);
  ledcAttachPin(pin, ch);
#endif
}
void pwmWrite(int pin, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(pin == PIN_L_PWM ? 0 : 1, duty);
#endif
}

// ======================= MOTORES =======================
void setupMotors() {
  const int pins[] = {PIN_L_IN1, PIN_L_IN2, PIN_R_IN1, PIN_R_IN2};
  for (int p : pins) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  pwmAttach(PIN_L_PWM, 0);
  pwmAttach(PIN_R_PWM, 1);
  pwmWrite(PIN_L_PWM, 0); pwmWrite(PIN_R_PWM, 0);
}
void setMotor(int pwmPin, int in1, int in2, bool invert, float pct) {
  bool fwd = (pct >= 0) != invert;
  int duty = (int)(fabsf(pct) / 100.0f * PWM_MAX);
  if (duty == 0) { digitalWrite(in1, LOW); digitalWrite(in2, LOW); }
  else { digitalWrite(in1, fwd); digitalWrite(in2, !fwd); }
  pwmWrite(pwmPin, duty);
}
float deadband(float u) {                      // compensa a zona morta dos motores
  if (fabsf(u) < 0.5f) return 0;
  float s = u > 0 ? 1.f : -1.f;
  return s * (PWM_MIN_PCT + (100.0f - PWM_MIN_PCT) * fabsf(u) / 100.0f);
}
void driveWheels(float l, float r) {
  l = constrain(l, -100.f, 100.f); r = constrain(r, -100.f, 100.f);
  dirL = (l >= 0) ? 1 : -1; dirR = (r >= 0) ? 1 : -1;
  ulOut = l; urOut = r;
  setMotor(PIN_L_PWM, PIN_L_IN1, PIN_L_IN2, INVERT_LEFT,  l);
  setMotor(PIN_R_PWM, PIN_R_IN1, PIN_R_IN2, INVERT_RIGHT, r);
}
void stopMotors() { driveWheels(0, 0); }

// ======================= MEMÓRIA PERMANENTE =======================
void loadStored() {
  prefs.begin("robo", true);
  SET.tolMm = prefs.getFloat("tol", SET.tolMm); SET.tmax = prefs.getFloat("tmax", SET.tmax);
  SET.pwm   = prefs.getFloat("pwm", SET.pwm);   SET.hg   = prefs.getFloat("hg", SET.hg);
  SET.wb    = prefs.getFloat("wb", SET.wb);     SET.vmax = prefs.getFloat("vm", SET.vmax);
  SET.rm    = prefs.getInt("rm", SET.rm);
  if (prefs.isKey("gains")) prefs.getBytes("gains", G, sizeof(G));
  if (prefs.isKey("gl")) prefs.getBytes("gl", GL, sizeof(GL));
  if (prefs.isKey("gr")) prefs.getBytes("gr", GR, sizeof(GR));
  prefs.end();
}
void saveStored() {
  prefs.begin("robo", false);
  prefs.putFloat("tol", SET.tolMm); prefs.putFloat("tmax", SET.tmax);
  prefs.putFloat("pwm", SET.pwm);   prefs.putFloat("hg", SET.hg);
  prefs.putFloat("wb", SET.wb);     prefs.putFloat("vm", SET.vmax);
  prefs.putInt("rm", SET.rm);
  prefs.putBytes("gains", G, sizeof(G));
  prefs.putBytes("gl", GL, sizeof(GL));
  prefs.putBytes("gr", GR, sizeof(GR));
  prefs.end();
}

// ======================= ENVIO (fila + BLE ou USB) =======================
uint16_t txUsed() { return (txHead + TXQ_SIZE - txTail) % TXQ_SIZE; }
uint16_t txFree() { return TXQ_SIZE - 1 - txUsed(); }
void txClear() { txHead = txTail = 0; }

// Coloca uma mensagem (mais o '\n') na fila. Se não couber, descarta a mensagem inteira.
bool txPut(const char* s) {
  if (!linkUp()) return false;
  size_t n = strlen(s);
  if (n + 1 > txFree()) { txDrops++; return false; }
  for (size_t i = 0; i < n; i++) { txq[txHead] = (uint8_t)s[i]; txHead = (txHead + 1) % TXQ_SIZE; }
  txq[txHead] = '\n'; txHead = (txHead + 1) % TXQ_SIZE;
  return true;
}

// Envia até 4 pacotes por chamada. Um pacote só sai da fila depois que o enlace o aceita,
// então nada se perde quando ele está ocupado: a tentativa se repete no próximo loop().
void txFlush() {
  if (!linkUp()) return;
  uint8_t pkt[244];
  for (int k = 0; k < 4 && txTail != txHead; k++) {
    uint16_t n = txUsed();
    if (usbActive) {
      int av = Serial.availableForWrite();
      if (av <= 0) break;
      if (n > (uint16_t)av) n = (uint16_t)av;
    } else {
      uint16_t mp = mtuPayload;                // cópia local: mtuPayload é alterada pelo callback
      if (n > mp) n = mp;
    }
    if (n > sizeof(pkt)) n = sizeof(pkt);
    for (uint16_t i = 0; i < n; i++) pkt[i] = txq[(txTail + i) % TXQ_SIZE];
    if (usbActive) n = (uint16_t)Serial.write(pkt, n);
    else if (!txChar->notify(pkt, n)) break;
    txTail = (txTail + n) % TXQ_SIZE;
  }
}

// ======================= TELEMETRIA E REGISTRO =======================
const char* stateName() {
  switch (state) { case RUNNING: return "running"; case STOPPED: return "stopped";
                   case DONE: return "done"; case MANUAL: return "manual"; case SCANNING: return "scan"; default: return "idle"; }
}
float wheelDiffMm() { return (posR() - posL()) * 1000.0f; }

void sendTelemetry(bool force = false) {
  if (!linkUp() || (!force && txUsed() > TELEM_BACKLOG)) return;
  char b[200];
  snprintf(b, sizeof(b),
    "{\"state\":\"%s\",\"t\":%lu,\"sp\":%.3f,\"x\":%.4f,\"y\":%.4f,\"th\":%.1f,\"ul\":%.0f,\"ur\":%.0f,\"vl\":%.3f,\"vr\":%.3f,\"dw\":%.1f}",
    stateName(), (unsigned long)millis(), P.sp, px, py, th * 57.29578f, ulOut, urOut, vL, vR, wheelDiffMm());
  txPut(b);
}
void sendMsg(const char* s) {                  // texto livre para o console da IHM
  char b[300]; int n = snprintf(b, sizeof(b), "{\"msg\":\"");
  for (; *s && n < (int)sizeof(b) - 4; s++) b[n++] = (*s == '"' || *s == '\\' || (unsigned char)*s < 32) ? ' ' : *s;
  b[n++] = '"'; b[n++] = '}'; b[n] = 0;
  txPut(b);
}
void sendCfg() {
  char b[760]; int n = snprintf(b, sizeof(b),
    "{\"cfg\":{\"tol\":%.0f,\"tmax\":%.0f,\"pwm\":%.0f,\"hg\":%.0f,\"rm\":%d,\"wb\":%.0f,\"vm\":%.2f,"
    "\"gl\":[%.1f,%.1f,%.1f],\"gr\":[%.1f,%.1f,%.1f],\"g\":{",
    SET.tolMm, SET.tmax, SET.pwm, SET.hg, SET.rm, SET.wb, SET.vmax, GL[0], GL[1], GL[2], GR[0], GR[1], GR[2]);
  for (int i = 0; i < 6; i++)
    n += snprintf(b + n, sizeof(b) - n, "%s\"%s\":[%.2f,%.2f,%.2f]", i ? "," : "", MODE_NAMES[i], G[i][0], G[i][1], G[i][2]);
  snprintf(b + n, sizeof(b) - n, "}}}");
  txPut(b);
}

int16_t clamp16(float v) { return (int16_t)constrain(v, -32767.0f, 32767.0f); }
void logSample(float t) {
  if (logN >= MAX_LOG) return;
  Sample& s = logBuf[logN++];
  s.t  = (uint16_t)constrain(t * 1000.0f, 0.0f, 65535.0f);
  s.sp = (uint16_t)(P.sp / 0.0002f);
  s.x  = clamp16(px / 0.0002f);   s.y = clamp16(py / 0.0002f);
  s.ul = clamp16(ulOut * 10);     s.ur = clamp16(urOut * 10);
  s.vl = clamp16(vL * 1000);      s.vr = clamp16(vR * 1000);
  s.dw = clamp16(wheelDiffMm() * 10);
  s.th = clamp16(th * 572.9578f);                       // 0,1 grau
}

// O registro é enviado aos poucos pelo loop(), sem bloquear nada.
void startLog() {
  char b[40];
  if (state == RUNNING || logN < 3) { snprintf(b, sizeof(b), "{\"log\":\"begin\",\"n\":0}"); txPut(b); txPut("{\"log\":\"end\"}"); return; }
  snprintf(b, sizeof(b), "{\"log\":\"begin\",\"n\":%d}", logN); txPut(b);
  logSending = true; logPos = 0;
}
void pumpLog() {
  if (!logSending) return;
  if (state == RUNNING || !linkUp()) { logSending = false; return; }
  if (txFree() < 1400) return;                 // espera a fila esvaziar
  if (logPos >= logN) { txPut("{\"log\":\"end\"}"); logSending = false; return; }
  char b[1300];
  int len = snprintf(b, sizeof(b), "{\"log\":\"data\",\"d\":[");
  for (int j = logPos; j < logN && j < logPos + LOG_CHUNK; j++) {
    const Sample& s = logBuf[j];
    len += snprintf(b + len, sizeof(b) - len, "%s[%u,%.3f,%.4f,%.1f,%.1f,%.3f,%.3f,%.1f,%.4f,%.1f]", j > logPos ? "," : "",
                    s.t, s.sp * 0.0002f, s.x * 0.0002f, s.ul / 10.0f, s.ur / 10.0f, s.vl / 1000.0f, s.vr / 1000.0f,
                    s.dw / 10.0f, s.y * 0.0002f, s.th / 10.0f);
  }
  snprintf(b + len, sizeof(b) - len, "]}");
  txPut(b);
  logPos += LOG_CHUNK;
}

// ======================= EXECUÇÃO =======================
void setMode(const char* m) {                  // decide uma vez, em vez de comparar texto a cada ciclo
  strlcpy(P.mode, m, sizeof(P.mode));
  P.ol = !strcmp(P.mode, "OL");
  P.useP = strchr(P.mode, 'P') != nullptr;
  P.useI = strchr(P.mode, 'I') != nullptr;
  P.useD = strchr(P.mode, 'D') != nullptr;
}

void finishRun(RunState s) {
  stopMotors(); state = s;
  char m[120];
  snprintf(m, sizeof(m), "execução %s após %.2f s; x=%.3f m, y=%.3f m, rumo=%.1f graus",
           s == DONE ? "concluída" : "interrompida", (millis() - tStart) / 1000.0f, px, py, th * 57.29578f);
  sendMsg(m); sendTelemetry(true);
}

void readGains(JsonVariant v, float* g) {
  JsonArray a = v.as<JsonArray>();
  if (a.size() == 3) for (int j = 0; j < 3; j++) g[j] = max(0.0f, (float)a[j]);
}

void loadParams(JsonDocument& d) {
  if (d["mode"].is<const char*>()) setMode(d["mode"].as<const char*>());
  P.sp   = constrain((float)(d["sp"]   | P.sp),   0.0f, 1.0f);
  P.kp   = max(0.0f, (float)(d["kp"] | P.kp));
  P.ki   = max(0.0f, (float)(d["ki"] | P.ki));
  P.kd   = max(0.0f, (float)(d["kd"] | P.kd));
  P.vel  = constrain((float)(d["vel"]  | P.vel),  0.0f, 100.0f);
  P.time = constrain((float)(d["time"] | P.time), 0.0f, 10.0f);
  SET.tolMm = constrain((float)(d["tol"]  | SET.tolMm), 1.0f, 100.0f);
  SET.tmax  = constrain((float)(d["tmax"] | SET.tmax),  2.0f, 30.0f);
  SET.pwm   = constrain((float)(d["pwm"]  | SET.pwm),  20.0f, 100.0f);
  SET.hg    = constrain((float)(d["hg"]   | SET.hg),    0.0f, 1000.0f);
  SET.rm    = constrain((int)(d["rm"] | SET.rm), 0, 2);
  SET.wb    = constrain((float)(d["wb"]   | SET.wb),   50.0f, 300.0f);
  SET.vmax  = constrain((float)(d["vm"]   | SET.vmax),  0.1f, 1.5f);
  if (!d["gl"].isNull()) readGains(d["gl"], GL);
  if (!d["gr"].isNull()) readGains(d["gr"], GR);
}

void wheelReset() { W[0] = WheelPid(); W[1] = WheelPid(); }

void startRun() {
  logSending = false;
  resetOdometry();                             // cada execução começa na marca 0 m
  integ = 0; dFilt = 0; settleTime = 0; ePrev = P.sp; logN = 0; wheelReset();
  stopMotors();
  tStart = millis(); lastCtrl = tStart; lastTelem = 0;
  state = RUNNING;
  char m[90];
  snprintf(m, sizeof(m), "execução iniciada: modo %s, alvo %.2f m", P.mode, P.sp);
  sendMsg(m);
  sendTelemetry(true);
}

// PID de velocidade de uma roda: referência vref (m/s), medida v (m/s), saída em % de PWM.
// A pré-alimentação (vref/vmax) leva a roda perto da velocidade certa; o PID corrige a diferença.
float wheelPid(WheelPid& w, const float* g, float vref, float v, float dt) {
  float e = vref - v;
  float de = (e - w.ePrev) / dt; w.ePrev = e;
  w.dF = D_FILTER * w.dF + (1.0f - D_FILTER) * de;
  float uNoI = vref / SET.vmax * 100.0f + g[0] * e + g[2] * w.dF;
  float uTry = uNoI + g[1] * w.i;
  if (fabsf(uTry) < SET.pwm || uTry * e < 0) w.i += e * dt;   // anti-windup: não integra saturado
  return constrain(uNoI + g[1] * w.i, -SET.pwm, SET.pwm);
}

void controlStep() {
  const float dt = CTRL_PERIOD_MS / 1000.0f;
  float t = (millis() - tStart) / 1000.0f;
  odoStep();
  float e = P.sp - px;                         // a posição controlada é o avanço x
  bool done = false;

  if (P.ol) {
    // Malha aberta: velocidade e tempo fixos, sem usar a odometria para corrigir.
    float u = (t < P.time) ? (P.vel < SET.pwm ? P.vel : SET.pwm) : 0;
    float a = u > 0 ? deadband(u) : 0;
    driveWheels(a, a);
    done = (t >= P.time + 0.8f && fabsf(speed) < 0.01f) || t > P.time + 3.0f;
  } else {
    float de = (e - ePrev) / dt; ePrev = e;
    dFilt = D_FILTER * dFilt + (1.0f - D_FILTER) * de;
    float u = (P.useP ? P.kp * e : 0) + (P.useD ? P.kd * dFilt : 0);
    if (P.useI) {
      float uTry = u + P.ki * integ;
      bool sat = fabsf(uTry) >= SET.pwm;
      if (!sat || (uTry * e < 0)) integ += e * dt;        // anti-windup: não integra saturado
      u += P.ki * integ;
    }
    u = constrain(u, -SET.pwm, SET.pwm);

    float a, b;
    if (SET.rm == 2) {
      // Cascata: o controlador de posição gera a velocidade de referência (u como fração de vmax)
      // e cada roda tem o seu próprio PID de velocidade.
      if (fabsf(e) < SET.tolMm / 2000.0f) { wheelReset(); a = b = 0; }   // dentro de meia tolerância: para
      else {
        float vref = u / 100.0f * SET.vmax;
        a = deadband(wheelPid(W[0], GL, vref, vL, dt));
        b = deadband(wheelPid(W[1], GR, vref, vR, dt));
      }
    } else {
      float base = deadband(u), corr = 0;
      if (SET.rm == 1 && fabsf(u) > 1.0f) corr = SET.hg * (posR() - posL());   // proporcional pela diferença entre rodas
      a = base + corr; b = base - corr;
    }
    driveWheels(a, b);

    if (fabsf(e) < SET.tolMm / 1000.0f && fabsf(speed) < 0.01f) settleTime += dt; else settleTime = 0;
    done = settleTime >= 1.0f || t > SET.tmax;
  }

  logSample(t);
  if (done) finishRun(DONE);
}

// ======================= SERVO E SONAR (3ª avaliação) =======================
struct ScanCfg { int step = 5, settle = 150, n = 3; float rmax = 200, temp = 25; } SC;
int16_t scanD[MAX_SCAN_PTS];              // distâncias em 0,1 cm (-10 = sem eco)
int scanIdx = 0, scanN = 0, scanCnt = 0; uint8_t scanPhase = 0; uint32_t scanT = 0;

void servoAttach() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_SERVO, 50, SERVO_BITS);
#else
  ledcSetup(2, 50, SERVO_BITS);
  ledcAttachPin(PIN_SERVO, 2);
#endif
}
void servoWrite(float deg) {              // 0 a 180 graus (0 = direita do robô)
  deg = constrain(deg + SERVO_TRIM_DEG, 0.0f, 180.0f);
  if (SERVO_INVERT) deg = 180.0f - deg;
  float us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * deg / 180.0f;
  int duty = (int)(us / 20000.0f * ((1 << SERVO_BITS) - 1));
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_SERVO, duty);
#else
  ledcWrite(2, duty);
#endif
}

// Uma medida do HC-SR04 em cm; -1 se não houver eco válido dentro do alcance.
float measureOnce() {
  digitalWrite(PIN_TRIG, LOW); delayMicroseconds(4);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  uint32_t timeout = (uint32_t)(SC.rmax * 58.8f * 1.2f) + 2000;          // ida e volta + folga
  unsigned long us = pulseIn(PIN_ECHO, HIGH, timeout);
  if (us == 0) return -1;
  float v = 331.3f + 0.606f * SC.temp;                                    // velocidade do som (m/s)
  float cm = us * v / 20000.0f;
  return (cm < 2.0f || cm > SC.rmax) ? -1 : cm;
}
// Mediana de n medidas (rejeita ecos falsos isolados). Sem maioria de medidas válidas: -1.
float measureMedian() {
  float v[5]; int k = 0;
  for (int i = 0; i < SC.n; i++) {
    float m = measureOnce();
    if (m > 0) v[k++] = m;
    if (i < SC.n - 1) delay(20);                                         // deixa o eco anterior morrer
  }
  if (k * 2 < SC.n || k == 0) return -1;
  for (int i = 1; i < k; i++) { float x = v[i]; int j = i - 1; while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; } v[j + 1] = x; }
  return v[k / 2];
}

void readScanCfg(JsonDocument& d) {
  SC.step   = constrain((int)(d["step"]   | SC.step),   2, 10);
  SC.settle = constrain((int)(d["settle"] | SC.settle), 50, 600);
  SC.n      = constrain((int)(d["n"]      | SC.n),      1, 5);
  SC.rmax   = constrain((float)(d["rmax"] | SC.rmax),   50.0f, 400.0f);
  SC.temp   = constrain((float)(d["temp"] | SC.temp),   -10.0f, 60.0f);
}

void endScan(bool ok) {
  servoWrite(90); scanPhase = 0; state = IDLE;
  char b[900]; int n = snprintf(b, sizeof(b), "{\"scan\":\"end\",\"ok\":%d,\"step\":%d,\"d\":[", ok ? 1 : 0, SC.step);
  for (int i = 0; i < scanCnt && n < (int)sizeof(b) - 12; i++)
    n += snprintf(b + n, sizeof(b) - n, "%s%.1f", i ? "," : "", scanD[i] < 0 ? -1.0f : scanD[i] / 10.0f);
  snprintf(b + n, sizeof(b) - n, "]}");
  txPut(b);                                  // conjunto completo: a IHM não depende dos pontos avulsos
  sendMsg(ok ? "varredura concluída" : "varredura interrompida");
  sendTelemetry(true);
}

void startScan(JsonDocument& d) {
  if (state != IDLE && state != STOPPED && state != DONE) { sendMsg("pare o robô antes de varrer"); return; }
  readScanCfg(d);
  scanN = (180 + SC.step - 1) / SC.step + 1;                              // inclui o 180 graus
  if (scanN > MAX_SCAN_PTS) scanN = MAX_SCAN_PTS;
  scanIdx = 0; scanCnt = 0; scanPhase = 0; scanT = millis();
  stopMotors(); logSending = false; state = SCANNING;
  char b[100]; snprintf(b, sizeof(b), "{\"scan\":\"begin\",\"n\":%d,\"step\":%d,\"rmax\":%.0f}", scanN, SC.step, SC.rmax);
  txPut(b);
  sendMsg("varredura iniciada");
}

// Um passo da varredura por volta do loop(): aponta o servo, espera assentar, mede e envia.
void scanStep(uint32_t now) {
  if ((int32_t)(now - scanT) < 0 || txFree() < 900) return;               // espera o servo ou a fila esvaziar
  if (scanIdx >= scanN) { endScan(true); return; }
  int a = scanIdx * SC.step; if (a > 180) a = 180;
  if (scanPhase == 0) {
    servoWrite(a);
    scanT = now + SC.settle + (scanIdx == 0 ? 400 : 0);                   // o primeiro ponto vem do repouso: espera mais
    scanPhase = 1; return;
  }
  float dist = measureMedian();
  scanD[scanIdx] = dist < 0 ? -10 : (int16_t)(dist * 10 + 0.5f);
  char b[80]; snprintf(b, sizeof(b), "{\"scan\":\"pt\",\"i\":%d,\"a\":%d,\"d\":%.1f}", scanIdx, a, dist);
  txPut(b);
  scanIdx++; scanCnt++; scanPhase = 0; scanT = now;
}

// Aponta o servo para um ângulo e mede uma vez (verificação de distâncias em ângulos escolhidos).
void aimOnce(JsonDocument& d) {
  if (state != IDLE && state != STOPPED && state != DONE) { sendMsg("pare o robô antes de medir"); return; }
  readScanCfg(d);
  float a = constrain((float)(d["a"] | 90.0f), 0.0f, 180.0f);
  servoWrite(a); delay(SC.settle + 150);
  float dist = measureMedian();
  char b[90]; snprintf(b, sizeof(b), "{\"scan\":\"pt\",\"a\":%.0f,\"d\":%.1f,\"single\":1}", a, dist);
  txPut(b);
}

// ======================= COMANDOS =======================
void handleText(const char* line) {
  char b[260]; float a, c;
  if (!strcasecmp(line, "help") || !strcasecmp(line, "ajuda"))
    sendMsg("comandos: status | zero | ble | log | reboot | pwm E D (gira os motores por 1 s)");
  else if (!strcasecmp(line, "status")) {
    snprintf(b, sizeof(b), "situação=%s modo=%s x=%.3f y=%.3f rumo=%.1f graus encE=%ld encD=%ld vE=%.2f vD=%.2f m/s memória=%u",
             stateName(), P.mode, px, py, th * 57.29578f, (long)encL, (long)encR, vL, vR, (unsigned)ESP.getFreeHeap());
    sendMsg(b);
  }
  else if (!strcasecmp(line, "zero")) { resetOdometry(); sendMsg("odometria zerada (x, y e rumo)"); }
  else if (!strcasecmp(line, "ble")) {
    snprintf(b, sizeof(b), "enlace: %s | bluetooth %s, pacote %u bytes | fila %u/%u | mensagens descartadas %lu",
             usbActive ? "USB" : (bleReady ? "Bluetooth" : "nenhum"), BLE_NAME, (unsigned)mtuPayload, txUsed(), TXQ_SIZE, (unsigned long)txDrops);
    sendMsg(b);
  }
  else if (!strcasecmp(line, "log")) { snprintf(b, sizeof(b), "registro guardado: %d amostras (máximo %d)", logN, MAX_LOG); sendMsg(b); }
  else if (!strcasecmp(line, "reboot")) { sendMsg("reiniciando..."); txFlush(); delay(300); ESP.restart(); }
  else if (sscanf(line, "pwm %f %f", &a, &c) == 2) {
    if (state == RUNNING || state == SCANNING) { sendMsg("há uma execução ou varredura em andamento"); return; }
    driveWheels(constrain(a, -100.f, 100.f), constrain(c, -100.f, 100.f));
    state = MANUAL; manualUntil = millis() + 1000;
    sendMsg("teste dos motores por 1 s");
  }
  else sendMsg("comando desconhecido; digite help");
}

void handleLine(const char* line) {
  JsonDocument d;
  if (deserializeJson(d, line)) return;
  const char* cmd = d["cmd"] | "";

  if (!strcmp(cmd, "ping")) return;
  if (!strcmp(cmd, "start")) { if (state == SCANNING) sendMsg("varredura em andamento"); else if (state != RUNNING) { loadParams(d); startRun(); } }
  else if (!strcmp(cmd, "set")) { if (state != RUNNING) loadParams(d); }   // ajustes travados durante a execução
  else if (!strcmp(cmd, "stop")) {
    if (state == RUNNING) finishRun(STOPPED);
    else if (state == MANUAL) { stopMotors(); state = IDLE; }
    else if (state == SCANNING) endScan(false);
  }
  else if (!strcmp(cmd, "reset")) { if (state != SCANNING) { stopMotors(); resetOdometry(); state = IDLE; sendTelemetry(true); } }
  else if (!strcmp(cmd, "drive")) {              // condução manual: vale por 300 ms
    if (state == RUNNING || state == SCANNING) return;
    float l = constrain((float)(d["l"] | 0.0f), -100.f, 100.f), r = constrain((float)(d["r"] | 0.0f), -100.f, 100.f);
    driveWheels(l, r);
    state = (l != 0 || r != 0) ? MANUAL : IDLE;
    manualUntil = millis() + 300;
  }
  else if (!strcmp(cmd, "text")) { handleText(d["line"] | ""); }
  else if (!strcmp(cmd, "scan")) { startScan(d); }
  else if (!strcmp(cmd, "aim")) { aimOnce(d); }
  else if (!strcmp(cmd, "log")) { startLog(); }
  else if (!strcmp(cmd, "save")) {
    if (state == RUNNING) return;
    loadParams(d);
    JsonObject go = d["gains"].as<JsonObject>();
    if (!go.isNull())
      for (int i = 0; i < 6; i++) readGains(go[MODE_NAMES[i]], G[i]);
    saveStored();
    sendCfg();
  }
}

// ======================= USB / SERIAL =======================
void pollUsb(uint32_t now) {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (!usbOver && usbLen > 0 && usbBuild[0] == '{') {
        usbBuild[usbLen] = 0;
        if (!usbActive) { usbActive = true; cfgPending = true; txClear(); }   // IHM acabou de conectar por USB
        lastUsbRx = now;
        handleLine(usbBuild);
      }
      usbLen = 0; usbOver = false;
    } else if (usbLen < RX_MAX - 1) usbBuild[usbLen++] = ch;
    else usbOver = true;                                                      // linha longa demais: descarta
  }
  if (usbActive && now - lastUsbRx > USB_TIMEOUT_MS) { usbActive = false; evDropped = true; }
}

// ======================= CALLBACKS BLE (outra tarefa: só filas e flags) =======================
class ServerCb : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& ci) override {
    mtuPayload = 20;
    s->updateConnParams(ci.getConnHandle(), 12, 24, 0, 400);   // 15 a 30 ms, tolerância de 4 s
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    bleReady = false; evDropped = true;
    NimBLEDevice::startAdvertising();                          // volta a aparecer para a IHM
  }
  void onMTUChange(uint16_t mtu, NimBLEConnInfo&) override {
    mtuPayload = constrain((int)mtu - 3, 20, 244);
  }
} serverCb;

class TxCb : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic*, NimBLEConnInfo&, uint16_t sub) override {
    bleReady = (sub & 1);
    if (bleReady) cfgPending = true;
  }
} txCb;

class RxCb : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo&) override {
    NimBLEAttValue v = c->getValue();
    const uint8_t* p = v.data();
    for (size_t i = 0; i < v.size(); i++) {
      char ch = (char)p[i];
      if (ch == '\r') continue;
      if (ch == '\n') {
        if (!rxOver && rxLen > 0) {
          uint8_t nx = (lqHead + 1) % LQ_SIZE;
          if (nx != lqTail) { rxBuild[rxLen] = 0; memcpy(lineQ[lqHead], rxBuild, rxLen + 1); lqHead = nx; }
        }
        rxLen = 0; rxOver = false;
      } else if (rxLen < RX_MAX - 1) rxBuild[rxLen++] = ch;
      else rxOver = true;                                      // linha longa demais: descarta até o '\n'
    }
  }
} rxCb;

void setupBle() {
  NimBLEDevice::init(BLE_NAME);
  NimBLEDevice::setMTU(247);
  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(&serverCb, false);
  NimBLEService* svc = bleServer->createService(SERVICE_UUID);
  txChar = svc->createCharacteristic(TX_UUID, NIMBLE_PROPERTY::NOTIFY);
  txChar->setCallbacks(&txCb);
  NimBLECharacteristic* rx = svc->createCharacteristic(RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rx->setCallbacks(&rxCb);
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setName(BLE_NAME);
  adv->addServiceUUID(SERVICE_UUID);
  adv->enableScanResponse(true);
  adv->start();
}

// ======================= SETUP / LOOP =======================
void setup() {
  Serial.begin(115200);
  setMode("PID");
  loadStored();
  setupMotors();
  pinMode(PIN_TRIG, OUTPUT); digitalWrite(PIN_TRIG, LOW); pinMode(PIN_ECHO, INPUT);
  servoAttach(); servoWrite(90);
  pinMode(PIN_ENC_L, INPUT); pinMode(PIN_ENC_R, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R), isrR, CHANGE);
  setupBle();
  Serial.printf("Robo pronto: Bluetooth \"%s\" ou USB (115200)\n", BLE_NAME);
}

void loop() {
  uint32_t now = millis();

  pollUsb(now);

  if (evDropped) {                             // um enlace caiu: para tudo e limpa as filas
    evDropped = false;
    txClear(); lqTail = lqHead; rxLen = 0; rxOver = false; logSending = false; cfgPending = false;
    if (state == RUNNING) finishRun(STOPPED);
    else if (state == MANUAL) { stopMotors(); state = IDLE; }
    else if (state == SCANNING) endScan(false);
  }
  if (cfgPending && linkUp()) { cfgPending = false; sendCfg(); }

  if (lqTail != lqHead) {                      // uma linha BLE por volta
    handleLine(lineQ[lqTail]);
    lqTail = (lqTail + 1) % LQ_SIZE;
  }

  if (state != RUNNING && now - lastOdo >= CTRL_PERIOD_MS) {        // odometria também fora da execução
    lastOdo = now;
    odoStep();
  }
  if (state == MANUAL && (int32_t)(now - manualUntil) > 0) {        // vigia da condução manual
    stopMotors(); state = IDLE;
    sendMsg("condução manual encerrada (sem novos comandos)");
  }

  if (state == RUNNING && now - lastCtrl >= CTRL_PERIOD_MS) {
    lastCtrl += CTRL_PERIOD_MS;
    if (now - lastCtrl > 5 * CTRL_PERIOD_MS) lastCtrl = now;       // não tenta "recuperar" ciclos atrasados
    controlStep();
  }

  uint32_t every = state == RUNNING ? TELEM_RUN_MS : state == MANUAL ? TELEM_MANUAL_MS : TELEM_IDLE_MS;
  if (now - lastTelem >= every) { lastTelem = now; sendTelemetry(); }

  if (state == SCANNING) scanStep(now);

  pumpLog();
  txFlush();

#if DEBUG_SERIAL
  if (state != RUNNING && now - lastDbg >= 1000) {
    lastDbg = now;
    Serial.printf("encL=%ld encR=%ld | x=%.3f y=%.3f\n", (long)encL, (long)encR, px, py);
  }
#endif
}