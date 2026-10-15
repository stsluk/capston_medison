#include <Adafruit_PWMServoDriver.h>
#include <HX711.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

// 네트워크 설정
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// 서버 주소 설정 (백엔드 서버 IP 또는 도메인)
const char* serverBaseUrl = "http://172.17.91.4:5000";
const String deviceId = "drawer-controller-1";
const String deviceToken = "KPWGkmwNfafDvgLeoOyt_2Be9GQccxZB6qura8n0yyc"; // 발급받은 토큰 값

// 타이밍 제어용 변수
unsigned long lastPollTime = 0;
const unsigned long pollInterval = 3000; // 3초 주기 폴링

// ===================== 하드웨어 설정 =====================
constexpr uint8_t  PIN_SDA        = 21;
constexpr uint8_t  PIN_SCL        = 22;
constexpr uint8_t  PCA_ADDR       = 0x40;
constexpr float    SERVO_FREQ_HZ  = 50.0f;
constexpr uint16_t PULSE_MIN_US = 500;    // 공통 펄스 (필요 시 CFG에서 채널별 덮어쓰기)
constexpr uint16_t PULSE_MAX_US = 2500;

constexpr uint8_t HX_DT[3]  = {16, 18, 25};
constexpr uint8_t HX_SCK[3] = {17, 19, 26};

// 마그네틱 센서
constexpr uint8_t PIN_LED = 27;
constexpr uint8_t PIN_MAG[3] = {32, 33, 14};   // 마그네틱 센서 (A,B,C)
constexpr bool MAG_CLOSED_LEVEL = LOW;          // 닫힘 시 LOW

// 로드셀 보정값
float LC_SCALE[3] = {1.82991215, -52.82846451, 1.82991215};
constexpr uint32_t LC_SETTLE_MS   = 300;   // 서보 정지 후 안정화 대기
constexpr uint8_t  LC_AVG_N       = 5;     // 이동평균 개수
constexpr float    LC_DETECT_G    = 100.0f;  // 무게 변화 감지 임계값(g)
constexpr uint32_t LC_CONFIRM_MS = 500;
constexpr bool     DETECT_INCREASE = false;  // true: 분배 시 무게 증가 / false: 감소

// ===================== 동작별 각도/시간 =====================
constexpr float DIV_OPEN_DEG    = 80, DIV_CLOSE_DEG    = 10;
constexpr float LOCK_OPEN_DEG   = 90, LOCK_CLOSE_DEG   = 10;
constexpr float PLATE_OPEN_DEG  = 30, PLATE_CLOSE_DEG = 100;
constexpr uint32_t DIV_MS = 500, LOCK_MS = 500, PLATE_MS = 1500;

constexpr uint32_t DISP_TIMEOUT_MS  = 15000;
constexpr uint32_t USER_OPEN_TIMEOUT_MS = 30000; // 사용자 열기 대기
constexpr uint32_t USER_CLOSE_TIMEOUT_MS = 60000;


// ===================== 채널 맵 =====================
enum Ch : uint8_t {
  CH_A_DIV = 0, CH_A_LOCK = 1,
  CH_B_DIV = 2, CH_B_LOCK = 3,
  CH_C_DIV = 4, CH_C_LOCK = 5,
  CH_PLATE_L = 6, CH_PLATE_R = 7,
  CH_COUNT = 8
};

const uint8_t DIV_CH[3]  = {CH_A_DIV,  CH_B_DIV,  CH_C_DIV};
const uint8_t LOCK_CH[3] = {CH_A_LOCK, CH_B_LOCK, CH_C_LOCK};

enum class Ease : uint8_t { LINEAR, SMOOTH };
enum class St : uint8_t {
  IDLE,
  D_UNLOCK, D_DIV_OPEN, D_PLATE_OPEN, D_WAIT_WEIGHT, D_PLATE_BACK, D_DIV_CLOSE, D_LOCK,
  U_UNLOCK, U_WAIT_OPEN, U_WAIT_CLOSE, U_LOCK
};

// ===================== 서보 설정 =====================
struct ServoCfg {
  float minDeg, maxDeg, homeDeg;
  uint16_t pulseMin, pulseMax;   // 0이면 공통값
};

ServoCfg CFG[CH_COUNT] = {
  {10, 90, 0, 0, 0},  // A_DIV
  {10, 90, 0, 0, 0},  // A_LOCK
  {10, 90, 0, 0, 0},  // B_DIV
  {10, 90, 0, 0, 0},  // B_LOCK
  {10, 90, 0, 0, 0},  // C_DIV
  {10, 90, 0, 0, 0},  // C_LOCK
  {10, 60, 0, 0, 0},  // PLATE_L
  {10, 60, 0, 0, 0},  // PLATE_R
};


// ===================== 서보 (논블로킹 이동) =====================
struct Motion {
  bool     active = false;
  float    fromDeg = 0, toDeg = 0;
  uint32_t startMs = 0, durMs = 0;
  Ease     ease = Ease::SMOOTH;
};

Adafruit_PWMServoDriver pwm(PCA_ADDR);
float  curDeg[CH_COUNT];
Motion mot[CH_COUNT];


// ===================== 저수준 =====================
void writeAngle(uint8_t ch, float deg) {
  const ServoCfg& c = CFG[ch];
  deg = constrain(deg, c.minDeg, c.maxDeg);
  curDeg[ch] = deg;
  uint16_t pMin = c.pulseMin ? c.pulseMin : PULSE_MIN_US;
  uint16_t pMax = c.pulseMax ? c.pulseMax : PULSE_MAX_US;
  pwm.writeMicroseconds(ch, pMin + (uint16_t)((pMax - pMin) * (deg / 180.0f)));
}

void moveTo(uint8_t ch, float deg, uint32_t durMs, Ease e = Ease::SMOOTH) {
  deg = constrain(deg, CFG[ch].minDeg, CFG[ch].maxDeg);
  if (durMs == 0) { mot[ch].active = false; writeAngle(ch, deg); return; }
  Motion& m = mot[ch];
  m.fromDeg = curDeg[ch]; m.toDeg = deg;
  m.startMs = millis(); m.durMs = durMs; m.ease = e; m.active = true;
}

void updateServos() {
  uint32_t now = millis();
  for (uint8_t ch = 0; ch < CH_COUNT; ch++) {
    Motion& m = mot[ch];
    if (!m.active) continue;
    float t = (float)(now - m.startMs) / (float)m.durMs;
    if (t >= 1.0f) { t = 1.0f; m.active = false; }
    float k = (m.ease == Ease::SMOOTH) ? t * t * (3.0f - 2.0f * t) : t;
    writeAngle(ch, m.fromDeg + (m.toDeg - m.fromDeg) * k);
  }
}
bool isMoving(uint8_t ch) { return mot[ch].active; }
bool anyMoving() {
  for (uint8_t i = 0; i < CH_COUNT; i++) if (mot[i].active) return true;
  return false;
}


// 1) 서랍 잠금 해제
void unlockDrawer(uint8_t drawer) {
  if (drawer > 2) return;
  moveTo(LOCK_CH[drawer], LOCK_OPEN_DEG, LOCK_MS);
}

// 2) 서랍 잠금
void lockDrawer(uint8_t drawer) {
  if (drawer > 2) return;
  moveTo(LOCK_CH[drawer], LOCK_CLOSE_DEG, LOCK_MS);
}

// 3) 인식판 열기 (L/R 동일 각도, 동일 시각 시작)
void movePlate(float deg) {   // L/R 같은 시각, 같은 각도
  moveTo(CH_PLATE_L, deg, PLATE_MS);
  moveTo(CH_PLATE_R, deg, PLATE_MS);
  mot[CH_PLATE_R].startMs = mot[CH_PLATE_L].startMs;
}
void openPlate()  { movePlate(PLATE_OPEN_DEG); }
void closePlate() { movePlate(PLATE_CLOSE_DEG); }

// 4) 서랍 구분면 (open=true 열기 / false 닫기)
void moveDivider(uint8_t drawer, bool open) {
  if (drawer > 2) return;
  moveTo(DIV_CH[drawer], open ? DIV_OPEN_DEG : DIV_CLOSE_DEG, DIV_MS);
}


// ===================== LED / 마그네틱 센서 =====================
void setLed(bool on) { digitalWrite(PIN_LED, on); }

bool magStable[3] = {true, true, true};   // true = 닫힘
uint8_t magCnt[3] = {0, 0, 0};

void updateSensors() {   // 매 loop 호출 (디바운스)
  for (uint8_t d = 0; d < 3; d++) {
    bool raw = (digitalRead(PIN_MAG[d]) == MAG_CLOSED_LEVEL);
    if (raw == magStable[d]) magCnt[d] = 0;
    else if (++magCnt[d] >= 5) { magStable[d] = raw; magCnt[d] = 0; }
  }
}
bool drawerClosed(uint8_t d) { return d < 3 ? magStable[d] : false; }


// ===================== 로드셀 (서랍 3개) =====================
struct LoadCell {
  HX711 hx;
  float buf[LC_AVG_N];
  uint8_t idx = 0;
  bool filled = false, ok = false;
  float weight = 0;
};
LoadCell lc[3];
uint32_t lcLastMoveMs = 0;

void initLoadCells() {
  for (uint8_t i = 0; i < 3; i++) {
    lc[i].hx.begin(HX_DT[i], HX_SCK[i]);
    lc[i].hx.set_scale(LC_SCALE[i]);
    if (lc[i].hx.wait_ready_timeout(1000)) { lc[i].hx.tare(10); lc[i].ok = true; }
    else { lc[i].ok = false; Serial.print(F("ERR LOADCELL ")); Serial.println(i); }
  }
}

void updateLoadCells() {
  if (anyMoving()) { lcLastMoveMs = millis(); return; }
  for (uint8_t i = 0; i < 3; i++) {
    LoadCell& c = lc[i];
    if (!c.ok || !c.hx.is_ready()) continue;
    c.buf[c.idx] = c.hx.get_units(1);
    c.idx = (c.idx + 1) % LC_AVG_N;
    if (c.idx == 0) c.filled = true;
    uint8_t n = c.filled ? LC_AVG_N : c.idx;
    float sum = 0;
    for (uint8_t k = 0; k < n; k++) sum += c.buf[k];
    c.weight = sum / n;
  }
}

bool isWeightStable(uint8_t d) {
  return d < 3 && lc[d].ok && lc[d].filled && !anyMoving() &&
         (millis() - lcLastMoveMs > LC_SETTLE_MS);
}

float getWeight(uint8_t d) { return d < 3 ? lc[d].weight : 0; }

void tareLoadCell(uint8_t d) {
  if (d > 2 || !lc[d].ok) return;
  lc[d].hx.tare(10);
  lc[d].filled = false; lc[d].idx = 0;
}


// ===================== 시퀀스 상태머신 =====================
St st = St::IDLE;
uint8_t  jd = 0;
uint32_t stTs = 0, overSince = 0;
float    baseG = 0;

void evt(const char* name, uint8_t d) {
  Serial.print(F("EVT ")); Serial.print(name); Serial.print(' '); Serial.println(d);
}
void failJob(const char* why, uint8_t d) {
  Serial.print(F("EVT FAIL ")); Serial.print(d); Serial.print(' '); Serial.println(why);
  closePlate(); moveDivider(d, false); lockDrawer(d);
  st = St::IDLE;
}
void go(St s) { st = s; stTs = millis(); }

bool startDispense(uint8_t d) {
  if (st != St::IDLE || d > 2 || !lc[d].ok || !drawerClosed(d)) return false;
  jd = d; unlockDrawer(d); go(St::D_UNLOCK);
  return true;
}
bool startUserOpen(uint8_t d) {
  if (st != St::IDLE || d > 2) return false;
  jd = d; unlockDrawer(d); go(St::U_UNLOCK);
  return true;
}
void abortJob() { if (st != St::IDLE) failJob("ABORT", jd); }

void updateSequence() {
  if (st == St::IDLE) return;
  uint32_t el = millis() - stTs;
  uint8_t d = jd;

  switch (st) {
    case St::D_UNLOCK:
      if (!isMoving(LOCK_CH[d])) { moveDivider(d, true); go(St::D_DIV_OPEN); }
      break;
    case St::D_DIV_OPEN:
      if (!isMoving(DIV_CH[d])) { openPlate(); go(St::D_PLATE_OPEN); }
      break;
    case St::D_PLATE_OPEN:
      if (!anyMoving()) { baseG = getWeight(d); overSince = 0; go(St::D_WAIT_WEIGHT); }
      break;
    case St::D_WAIT_WEIGHT: {
      float delta = DETECT_INCREASE ? (getWeight(d) - baseG) : (baseG - getWeight(d));
      if (isWeightStable(d) && delta > LC_DETECT_G) {
        if (!overSince) overSince = millis();
        else if (millis() - overSince > LC_CONFIRM_MS) { closePlate(); go(St::D_PLATE_BACK); }
      } else overSince = 0;
      if (st == St::D_WAIT_WEIGHT && el > DISP_TIMEOUT_MS) failJob("NO_WEIGHT", d);
      break;
    }
    case St::D_PLATE_BACK:
      if (!anyMoving()) { moveDivider(d, false); go(St::D_DIV_CLOSE); }
      break;
    case St::D_DIV_CLOSE:
      if (!anyMoving()) { lockDrawer(d); go(St::D_LOCK); }
      break;
    case St::D_LOCK:
      if (!anyMoving()) { evt("DONE", d); st = St::IDLE; }
      break;

    case St::U_UNLOCK:
      if (!isMoving(LOCK_CH[d])) { evt("UNLOCKED", d); go(St::U_WAIT_OPEN); }
      break;
    case St::U_WAIT_OPEN:
      if (!drawerClosed(d)) { evt("OPENED", d); go(St::U_WAIT_CLOSE); }
      else if (el > USER_OPEN_TIMEOUT_MS) failJob("NOT_OPENED", d);
      break;
    case St::U_WAIT_CLOSE:
      if (drawerClosed(d)) { evt("CLOSED", d); lockDrawer(d); go(St::U_LOCK); }
      else if (el > USER_CLOSE_TIMEOUT_MS) failJob("NOT_CLOSED", d);
      break;
    case St::U_LOCK:
      if (!isMoving(LOCK_CH[d])) { evt("LOCKED", d); st = St::IDLE; }
      break;
    default: break;
  }
}


// ===================== 시리얼 명령 =====================
// U<n> 잠금해제  L<n> 잠금  O<n> 구분면 열기  C<n> 구분면 닫기
// P 인식판 열기   Q 인식판 닫기
// W<n> 무게(g)    T<n> 영점     E1/E0 LED
// D<n> 약 분배    K<n> 사용자 개방   X 중단   S 상태
void printStatus() {
  Serial.print(F("STAT st=")); Serial.print((uint8_t)st);
  for (uint8_t i = 0; i < 3; i++) {
    Serial.print(F(" | ")); Serial.print(i);
    Serial.print(F(" w=")); Serial.print(lc[i].weight, 1);
    Serial.print(drawerClosed(i) ? F(" closed") : F(" open"));
  }
  Serial.println();
}

void handleSerial() {
  static char buf[16];
  static uint8_t len = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (len < sizeof(buf) - 1) buf[len++] = c;
      else len = 0;
      continue;
    }
    buf[len] = '\0';
    len = 0;
    if (buf[0] == '\0') continue;

    char cmd = buf[0];
    bool hasNum = (buf[1] >= '0' && buf[1] <= '2' && buf[2] == '\0');
    bool noArg  = (buf[1] == '\0');
    uint8_t n = hasNum ? buf[1] - '0' : 0;
    bool ok = true;

    switch (cmd) {
      case 'U': if (hasNum) unlockDrawer(n);       else ok = false; break;
      case 'L': if (hasNum) lockDrawer(n);         else ok = false; break;
      case 'O': if (hasNum) moveDivider(n, true);  else ok = false; break;
      case 'C': if (hasNum) moveDivider(n, false); else ok = false; break;
      case 'P': if (noArg) openPlate();            else ok = false; break;
      case 'Q': if (noArg) closePlate();           else ok = false; break;
      case 'W': if (hasNum) { Serial.println(getWeight(n), 1); continue; } else ok = false; break;
      case 'T': if (hasNum) tareLoadCell(n);       else ok = false; break;
      case 'E':
        if ((buf[1] == '0' || buf[1] == '1') && buf[2] == '\0') setLed(buf[1] == '1');
        else ok = false;
        break;
      case 'D': if (hasNum) ok = startDispense(n); else ok = false; break;
      case 'K': if (hasNum) ok = startUserOpen(n); else ok = false; break;
      case 'X': if (noArg) abortJob();             else ok = false; break;
      case 'S': if (noArg) { printStatus(); continue; } else ok = false; break;
      default:  ok = false;
    }
    Serial.println(ok ? F("OK") : F("ERR"));
  }
}


// ===================== setup / loop =====================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_LED, OUTPUT); setLed(false);
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(PIN_MAG[i], INPUT_PULLUP);
    magStable[i] = (digitalRead(PIN_MAG[i]) == MAG_CLOSED_LEVEL);
  }

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);   // 불안정하면 100000
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ_HZ);
  delay(10);

  for (uint8_t ch = 0; ch < CH_COUNT; ch++) {   // 시차 기동
    writeAngle(ch, CFG[ch].homeDeg);
    delay(150);
  }

  initLoadCells();   // 빈 상태에서 부팅할 것 (자동 영점)
  Serial.println(F("READY"));
}

void loop() {
  updateServos();
  updateSensors();
  updateLoadCells();
  updateSequence();
  handleSerial();
}