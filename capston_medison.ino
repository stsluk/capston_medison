#include <Wire.h>
#include <esp_eap_client.h>
#include <Adafruit_PWMServoDriver.h>
#include <HX711.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <math.h>

// ============================================================
// 1. 네트워크 설정
// ============================================================

// 일반적인 SSID + 비밀번호 방식: 0
// 학교에서 PEAP 방식의 아이디 + 비밀번호 인증을 사용하는 경우: 1
// TLS/TTLS나 웹 로그인 방식은 이 설정만으로 처리되지 않습니다.
#define USE_ENTERPRISE_WIFI 1
#define ENABLE_NETWORK 1  // 네트워크 사용 여부

const char* WIFI_SSID = "wifi";
const char* WIFI_PASSWORD = "password";

// USE_ENTERPRISE_WIFI가 1일 때 사용합니다.
// 학교에서 안내한 identity / username 형식으로 설정하세요.
const char* EAP_IDENTITY = "anonymous";
const char* EAP_USERNAME = "name";
const char* EAP_PASSWORD = "password";

// 학교에서 서버 인증서 검증을 요구하면
// 공식 안내에 따라 올바른 CA 인증서를 설정해야 합니다.
const char* EAP_CA_CERT = nullptr;

// ============================================================
// 2. 서버 설정
// ============================================================

const char* SERVER_BASE_URL = "http://...";

const String DEVICE_ID = "drawer-controller-1";
const String DEVICE_TOKEN = "token";

// 확인된 조회 API
const char* DRAWERS_PATH = "/api/v1/drawers";

// 아래 두 명령 API는 기존 ESP32 코드의 경로를 유지했습니다.
// 서버의 flask routes 결과와 반드시 비교하세요.
const char* COMMAND_NEXT_PATH = "/api/v1/drawer/commands/next";
const char* COMMAND_ACK_PREFIX = "/api/v1/drawer/commands/";

// 기존 센서 API
const char* SENSOR_PATH = "/api/v1/drawers/sensor-state";

// ============================================================
// 3. 통신 주기 및 제한 시간
// ============================================================

constexpr uint32_t WIFI_RETRY_MS = 30000;
constexpr uint32_t COMMAND_POLL_MS = 3000;
constexpr uint32_t SENSOR_SEND_MS = 5000;
constexpr uint32_t ACK_RETRY_MS = 5000;

constexpr int HTTP_CONNECT_TIMEOUT_MS = 3000;
constexpr uint16_t HTTP_READ_TIMEOUT_MS = 3000;

uint32_t lastWifiAttemptMs = 0;
uint32_t lastPollMs = 0;
uint32_t lastSensorMs = 0;
uint32_t lastAckAttemptMs = 0;

bool wifiWasConnected = false;
bool serverTestNeeded = false;

// ============================================================
// 4. 하드웨어 핀 설정
// ============================================================

constexpr uint8_t PIN_SDA = 21;
constexpr uint8_t PIN_SCL = 22;
constexpr uint8_t PCA_ADDR = 0x40;

constexpr float SERVO_FREQ_HZ = 50.0f;
constexpr uint16_t SERVOMIN = 150;
constexpr uint16_t SERVOMAX = 600;

constexpr uint8_t HX_DT[3] = {16, 18, 25};
constexpr uint8_t HX_SCK[3] = {17, 19, 26};

constexpr uint8_t PIN_LED = 27;
constexpr uint8_t PIN_MAG[3] = {32, 33, 14};
constexpr bool MAG_CLOSED_LEVEL = LOW;

// ============================================================
// 5. 로드셀 설정
// ============================================================

// 로드 보정값 유지
float LC_SCALE[3] = {
  87.0,
  106.0,
  90.0
};

constexpr uint32_t LC_SETTLE_MS = 300;
constexpr uint8_t LC_AVG_N = 10;

// 기존 분배 시퀀스의 무게 변화 기준 유지
constexpr float LC_DETECT_G = 100.0f;
constexpr uint32_t LC_CONFIRM_MS = 500;
constexpr bool DETECT_INCREASE = true;

// 서버에 보낼 occupied 판정 기준
// 임시 시작값이므로 실제 빈 서랍 노이즈와 약 무게로 조정하세요.
//
// 비어 있음 -> 차 있음: 350mg 이상
// 차 있음 -> 비어 있음: 200mg 이하
//
// 서로 다른 기준을 사용해서 경계에서 0/1이 반복되는 것을 줄입니다.
constexpr float OCCUPIED_ON_G = 350.0f;
constexpr float OCCUPIED_OFF_G = 200.0f;

bool ENABLE_SENSOR_UPLOAD = false;

int occupiedState[3] = {-1, -1, -1};
uint8_t nextSensorIndex = 0;

// ============================================================
// 6. 동작 각도 및 시간
// ============================================================

// 서보모터 각도 설정
constexpr float DIV_OPEN_DEG[3] = {30, 30, 180};
constexpr float DIV_CLOSE_DEG[3] = {60, 80, 120};

constexpr float LOCK_OPEN_DEG = 10;
constexpr float LOCK_CLOSE_DEG = 100;

constexpr float PLATE_OPEN_DEG = 130;
constexpr float PLATE_CLOSE_DEG = 90;

constexpr uint32_t DIV_MS = 500;
constexpr uint32_t LOCK_MS = 500;
constexpr uint32_t PLATE_MS = 1500;

constexpr uint32_t DISP_TIMEOUT_MS = 15000;


// ============================================================
// 7. 서보 채널
// ============================================================

enum Ch : uint8_t {
  CH_A_DIV = 0,
  CH_A_LOCK = 1,
  CH_B_DIV = 2,
  CH_B_LOCK = 3,
  CH_C_DIV = 4,
  CH_C_LOCK = 5,
  CH_PLATE_L = 6,
  CH_PLATE_R = 7,
  CH_COUNT = 8
};

const uint8_t DIV_CH[3] = {
  CH_A_DIV, CH_B_DIV, CH_C_DIV
};

const uint8_t LOCK_CH[3] = {
  CH_A_LOCK, CH_B_LOCK, CH_C_LOCK
};

enum class Ease : uint8_t {
  LINEAR,
  SMOOTH
};

enum class St : uint8_t {
  IDLE,
  D_UNLOCK,
  D_DIV_OPEN,
  D_PLATE_OPEN,
  D_WAIT_WEIGHT,
  D_PLATE_BACK,
  D_DIV_CLOSE,
  D_LOCK,
  U_UNLOCK,
  U_WAIT_OPEN,
  U_WAIT_CLOSE,
  U_LOCK
};

// ============================================================
// 8. 서보 설정과 상태
// ============================================================

struct ServoCfg {
  float minDeg;
  float maxDeg;
  float homeDeg;
  uint16_t pulseMin;
  uint16_t pulseMax;
};

// 기존 각도 제한 유지
ServoCfg CFG[CH_COUNT] = {
  {10, 180, 60, 0, 0},
  {10, 180, 100, 0, 0},
  {10, 180, 80, 0, 0},
  {10, 180, 100, 0, 0},
  {10, 180, 120, 0, 0},
  {10, 180, 100, 0, 0},
  {10, 180, 90, 0, 0},
  {10, 180, 90, 0, 0}
};

struct Motion {
  bool active = false;
  float fromDeg = 0;
  float toDeg = 0;
  uint32_t startMs = 0;
  uint32_t durMs = 0;
  Ease ease = Ease::SMOOTH;
};

Adafruit_PWMServoDriver pwm(PCA_ADDR);

float curDeg[CH_COUNT];
Motion mot[CH_COUNT];

// ============================================================
// 9. 마그네틱 센서 상태
// ============================================================

bool magStable[3] = {true, true, true};
uint8_t magCnt[3] = {0, 0, 0};

uint32_t lastMagSampleMs = 0;

// ============================================================
// 10. 로드셀 상태
// ============================================================

struct LoadCell {
  HX711 hx;
  float buf[LC_AVG_N] = {};
  uint8_t idx = 0;
  bool filled = false;
  bool ok = false;
  float weight = 0;
};

LoadCell lc[3];

uint32_t lcLastMoveMs = 0;

// ============================================================
// 11. 동작 시퀀스 상태
// ============================================================

St st = St::IDLE;

uint8_t jd = 0;
uint32_t stTs = 0;
uint32_t overSince = 0;
float baseG = 0;

// ============================================================
// 12. 서버 명령 및 ACK 상태
// ============================================================

// 명령을 받은 직후에는 완료 처리하지 않고,
// U_UNLOCK에서 서보 이동이 끝났을 때 ACK를 준비합니다.
String activeCommandId = "";

String pendingAckId = "";
String pendingAckStatus = "";
String pendingAckMessage = "";
bool ackReady = false;

// 같은 명령을 반복 수신할 때 중복 동작을 막는 용도
String lastAckedCommandId = "";
String lastAckedStatus = "";
String lastAckedMessage = "";

// ============================================================
// 13. 서보 저수준 제어
// ============================================================

void writeAngle(uint8_t ch, float deg) {
  const ServoCfg& c = CFG[ch];

  deg = constrain(deg, c.minDeg, c.maxDeg);
  curDeg[ch] = deg;

  uint16_t pulse = map((long)deg, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(ch, 0, pulse);
}

void moveTo(uint8_t ch, float deg, uint32_t durMs, Ease e = Ease::SMOOTH) {
  deg = constrain(deg, CFG[ch].minDeg, CFG[ch].maxDeg);

  if (durMs == 0) {
    mot[ch].active = false;
    writeAngle(ch, deg);
    return;
  }

  Motion& m = mot[ch];

  m.fromDeg = curDeg[ch];
  m.toDeg = deg;
  m.startMs = millis();
  m.durMs = durMs;
  m.ease = e;
  m.active = true;
}

void updateServos() {
  uint32_t now = millis();

  for (uint8_t ch = 0; ch < CH_COUNT; ch++) {
    Motion& m = mot[ch];

    if (!m.active) {
      continue;
    }

    float t = (float)(now - m.startMs) / (float)m.durMs;

    if (t >= 1.0f) {
      t = 1.0f;
      m.active = false;
    }

    float k = (
      m.ease == Ease::SMOOTH
      ? t * t * (3.0f - 2.0f * t)
      : t
    );

    writeAngle(
      ch,
      m.fromDeg + (m.toDeg - m.fromDeg) * k
    );
  }
}

bool isMoving(uint8_t ch) {
  return mot[ch].active;
}

bool anyMoving() {
  for (uint8_t i = 0; i < CH_COUNT; i++) {
    if (mot[i].active) {
      return true;
    }
  }

  return false;
}

void unlockDrawer(uint8_t drawer) {
  if (drawer > 2) {
    return;
  }

  moveTo(LOCK_CH[drawer], LOCK_OPEN_DEG, LOCK_MS);
}

void lockDrawer(uint8_t drawer) {
  if (drawer > 2) {
    return;
  }

  moveTo(LOCK_CH[drawer], LOCK_CLOSE_DEG, LOCK_MS);
}

void movePlate(float deg) {
  moveTo(CH_PLATE_L, deg, PLATE_MS);
  moveTo(CH_PLATE_R, deg, PLATE_MS);

  mot[CH_PLATE_R].startMs = mot[CH_PLATE_L].startMs;
}

void openPlate() {
  movePlate(PLATE_OPEN_DEG);
}

void closePlate() {
  movePlate(PLATE_CLOSE_DEG);
}

void moveDivider(uint8_t drawer, bool open) {
  if (drawer > 2) {
    return;
  }

  moveTo(
    DIV_CH[drawer],
    open ? DIV_OPEN_DEG[drawer] : DIV_CLOSE_DEG[drawer],
    DIV_MS
  );
}

// ============================================================
// 14. LED / 마그네틱 센서
// ============================================================

void setLed(bool on) {
  digitalWrite(PIN_LED, on ? HIGH : LOW);
}

void updateSensors() {
  uint32_t now = millis();

  // HTTP 요청 전후의 loop 속도 차이에 덜 영향을 받도록
  // 일정 시간 간격으로 샘플링합니다.
  if (now - lastMagSampleMs < 5) {
    return;
  }

  lastMagSampleMs = now;

  for (uint8_t d = 0; d < 3; d++) {
    bool raw = (
      digitalRead(PIN_MAG[d]) == MAG_CLOSED_LEVEL
    );

    if (raw == magStable[d]) {
      magCnt[d] = 0;
    } else {
      magCnt[d]++;

      if (magCnt[d] >= 5) {
        magStable[d] = raw;
        magCnt[d] = 0;
      }
    }
  }
}

bool drawerClosed(uint8_t d) {
  return d < 3 ? magStable[d] : false;
}

// ============================================================
// 15. 로드셀
// ============================================================

void initLoadCells() {
  Serial.println(
    "[LOADCELL] Init start. All drawers must be empty."
  );

  constexpr uint8_t TARE_SAMPLES = 10;

  for (uint8_t i = 0; i < 3; i++) {
    LoadCell& c = lc[i];

    c.ok = false;
    c.filled = false;
    c.idx = 0;
    c.weight = 0;

    Serial.printf(
      "[LOADCELL] drawer %u BEGIN: DT=%u SCK=%u\n", i + 1, HX_DT[i], HX_SCK[i]
    );

    Serial.println("[LOADCELL] before hx.begin");

    c.hx.begin(HX_DT[i], HX_SCK[i]);

    Serial.println("[LOADCELL] after hx.begin");

    c.hx.set_scale(LC_SCALE[i]);

    Serial.println("[LOADCELL] after set_scale");

    long long sum = 0;
    bool success = true;

    for (uint8_t sample = 0; sample < TARE_SAMPLES; sample++) {
      Serial.printf(
        "[LOADCELL] drawer %u waiting sample %u/%u\n",
        i + 1,
        sample + 1,
        TARE_SAMPLES
      );

      if (!c.hx.wait_ready_timeout(1000, 1)) {
        Serial.printf(
          "[LOADCELL] drawer %u TIMEOUT at sample %u\n",
          i + 1,
          sample + 1
        );

        success = false;
        break;
      }

      long raw = c.hx.read();
      sum += raw;

      Serial.printf(
        "[LOADCELL] drawer %u sample %u raw=%ld\n",
        i + 1,
        sample + 1,
        raw
      );

      delay(1);
    }

    if (!success) {
      Serial.printf(
        "[LOADCELL] drawer %u NOT READY; "
        "check wiring, power and pin numbers\n",
        i + 1
      );

      continue;
    }

    long offset = (long)(sum / TARE_SAMPLES);

    c.hx.set_offset(offset);
    c.ok = true;

    Serial.printf(
      "[LOADCELL] drawer %u READY offset=%ld\n",
      i + 1,
      offset
    );
  }

  Serial.println("[LOADCELL] Init finished.");
}

void updateLoadCells() {
  if (anyMoving()) {
    lcLastMoveMs = millis();
    return;
  }

  for (uint8_t i = 0; i < 3; i++) {
    LoadCell& c = lc[i];

    if (!c.ok || !c.hx.is_ready()) {
      continue;
    }

    float value = c.hx.get_units(1);

    if (!isfinite(value)) {
      continue;
    }

    c.buf[c.idx] = value;
    c.idx = (c.idx + 1) % LC_AVG_N;

    if (c.idx == 0) {
      c.filled = true;
    }

    uint8_t n = c.filled ? LC_AVG_N : c.idx;

    if (n == 0) {
      continue;
    }

    float sum = 0;

    for (uint8_t k = 0; k < n; k++) {
      sum += c.buf[k];
    }

    c.weight = sum / n;
  }
}

bool isWeightStable(uint8_t d) {
  // 현재 구현은 이동 후 대기 및 평균 버퍼 준비 여부를 확인합니다.
  // 측정 분산을 이용한 정밀 안정성 판정은 아닙니다.
  return (
    d < 3 &&
    lc[d].ok &&
    lc[d].filled &&
    !anyMoving() &&
    millis() - lcLastMoveMs > LC_SETTLE_MS
  );
}

float getWeight(uint8_t d) {
  return d < 3 ? lc[d].weight : 0;
}

void tareLoadCell(uint8_t d) {
  if (d > 2 || !lc[d].ok) {
    return;
  }

  lc[d].hx.tare(10);
  lc[d].filled = false;
  lc[d].idx = 0;
  lc[d].weight = 0;

  occupiedState[d] = -1;

  Serial.printf("[LOADCELL] drawer %u tared\n", d + 1);
}

// ============================================================
// 16. ACK 준비
// ============================================================

void queueAck(
  const String& commandId,
  const String& status,
  const String& message
) {
  pendingAckId = commandId;
  pendingAckStatus = status;
  pendingAckMessage = message;

  ackReady = true;

  // 다음 통신 가능 시점에 바로 시도하도록 설정
  lastAckAttemptMs = millis() - ACK_RETRY_MS;
}

// ============================================================
// 17. 시퀀스 제어
// ============================================================

void evt(const char* name, uint8_t d) {
  Serial.print("EVT ");
  Serial.print(name);

  Serial.print(" hardware_index=");
  Serial.print(d);

  Serial.print(" server_drawer_no=");
  Serial.println(d + 1);
}

void go(St s) {
  st = s;
  stTs = millis();
}

void failJob(const char* why, uint8_t d) {
  Serial.printf(
    "EVT FAIL hardware_index=%u reason=%s\n",
    d,
    why
  );

  // 잠금 해제 완료 전에 실패했다면 실패 ACK를 준비합니다.
  if (activeCommandId.length() > 0) {
    queueAck(activeCommandId, "failed", why);
    activeCommandId = "";
  }

  // 기존 담당자 코드의 실패 시 동작 유지
  closePlate();
  moveDivider(d, false);
  lockDrawer(d);

  st = St::IDLE;
}

bool startDispense(uint8_t d) {
  if (
    st != St::IDLE || anyMoving() || d > 2 || !lc[d].ok || !drawerClosed(d)
  ) {
    return false;
  }

  jd = d;
  baseG = getWeight(d);
  overSince = 0;

  Serial.printf("[DISPENSE] Initial weight: %.2f g\n", baseG);

  unlockDrawer(d);
  go(St::D_UNLOCK);

  return true;
}

bool startUserOpen(uint8_t d) {
  if (st != St::IDLE || anyMoving() || d > 2) {
    return false;
  }

  jd = d;

  unlockDrawer(d);
  go(St::U_UNLOCK);

  return true;
}

void abortJob() {
  if (st != St::IDLE) {
    failJob("ABORT", jd);
  }
}

void updateSequence() {
  if (st == St::IDLE) {
    return;
  }

  uint32_t el = millis() - stTs;
  uint8_t d = jd;

  switch (st) {
    case St::D_UNLOCK:
      if (!isMoving(LOCK_CH[d])) {
        moveDivider(d, true);
        go(St::D_DIV_OPEN);
      }
      break;

    case St::D_DIV_OPEN:
      if (!isMoving(DIV_CH[d])) {
        openPlate();
        go(St::D_PLATE_OPEN);
      }
      break;

    case St::D_PLATE_OPEN:
      if (!anyMoving()) {
        go(St::D_WAIT_WEIGHT);
      }
      break;

    case St::D_WAIT_WEIGHT: {
      float delta = (
        DETECT_INCREASE
        ? getWeight(d) - baseG
        : baseG - getWeight(d)
      );

      if (isWeightStable(d) && delta > LC_DETECT_G) {
        if (overSince == 0) {
          overSince = millis();
        } else if (millis() - overSince > LC_CONFIRM_MS) {
          closePlate();
          go(St::D_PLATE_BACK);
        }
      } else {
        overSince = 0;
      }

      if (
        st == St::D_WAIT_WEIGHT &&
        el > DISP_TIMEOUT_MS
      ) {
        failJob("NO_WEIGHT", d);
      }

      break;
    }

    case St::D_PLATE_BACK:
      if (!anyMoving()) {
        moveDivider(d, false);
        go(St::D_DIV_CLOSE);
      }
      break;

    case St::D_DIV_CLOSE:
      if (!anyMoving()) {
        lockDrawer(d);
        go(St::D_LOCK);
      }
      break;

    case St::D_LOCK:
      if (!anyMoving()) {
        evt("DONE", d);
        st = St::IDLE;
      }
      break;

    case St::U_UNLOCK:
      if (!isMoving(LOCK_CH[d])) {
        evt("UNLOCK_MOTION_FINISHED", d);

        if (activeCommandId.length() > 0) {
          queueAck(
            activeCommandId,
            "completed",
            "unlock servo motion finished; "
            "physical latch position not independently verified"
          );

          activeCommandId = "";
        }

        go(St::U_WAIT_OPEN);
      }
      break;

    case St::U_WAIT_OPEN:
      if (!drawerClosed(d)) {
        evt("OPENED", d);
        go(St::U_WAIT_CLOSE);
      } 
      break;

    case St::U_WAIT_CLOSE:
      if (drawerClosed(d)) {
        evt("CLOSED", d);

        lockDrawer(d);
        go(St::U_LOCK);
      } 
      break;

    case St::U_LOCK:
      if (!isMoving(LOCK_CH[d])) {
        evt("LOCK_MOTION_FINISHED", d);
        st = St::IDLE;
      }
      break;

    default:
      break;
  }
}

// ============================================================
// 18. HTTP 공통 출력
// ============================================================

void configureHttp(HTTPClient& http) {
  http.setConnectTimeout(HTTP_CONNECT_TIMEOUT_MS);
  http.setTimeout(HTTP_READ_TIMEOUT_MS);
}

void printHttpResult(
  const char* label,
  int code,
  const String& body
) {
  Serial.printf("[%s] HTTP CODE: %d\n", label, code);

  if (code > 0) {
    Serial.print("[");
    Serial.print(label);
    Serial.println("] BODY:");

    if (body.length() > 0) {
      Serial.println(body);
    } else {
      Serial.println("(empty)");
    }
  } else {
    Serial.print("[");
    Serial.print(label);
    Serial.print("] NETWORK ERROR: ");
    Serial.println(HTTPClient::errorToString(code));
  }
}

// ============================================================
// 19. 와이파이 연결 관리
// ============================================================

void beginWifiConnection() {
  lastWifiAttemptMs = millis();

  Serial.print("[WIFI] Connecting to ");
  Serial.println(WIFI_SSID);

#if USE_ENTERPRISE_WIFI
  Serial.println("[WIFI] Authentication: EAP-TTLS / PAP");

  WiFi.begin(
    WIFI_SSID,
    WPA2_AUTH_TTLS,
    EAP_IDENTITY,
    EAP_USERNAME,
    EAP_PASSWORD,
    EAP_CA_CERT,
    nullptr,
    nullptr,
    ESP_EAP_TTLS_PHASE2_PAP
  );
#else
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
#endif
}

void updateWifi() {
  bool connected = WiFi.status() == WL_CONNECTED;

  if (connected) {
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      serverTestNeeded = true;

      Serial.println("[WIFI] Connected");

      Serial.print("[WIFI] ESP32 IP: ");
      Serial.println(WiFi.localIP());

      Serial.print("[WIFI] Gateway: ");
      Serial.println(WiFi.gatewayIP());

      Serial.print("[WIFI] RSSI: ");
      Serial.println(WiFi.RSSI());
    }

    return;
  }

  if (wifiWasConnected) {
    wifiWasConnected = false;
    Serial.println("[WIFI] Disconnected");
  }

  if (
    !anyMoving() &&
    st == St::IDLE &&
    millis() - lastWifiAttemptMs >= WIFI_RETRY_MS
  ) {
    Serial.printf(
      "[WIFI] Not connected. status=%d; retrying\n",
      (int)WiFi.status()
    );

    WiFi.disconnect();
    beginWifiConnection();
  }
}

// ============================================================
// 20. 接続確認 GET /api/v1/drawers
// ============================================================

void testServerConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  WiFiClient client;
  HTTPClient http;

  String url = String(SERVER_BASE_URL) + DRAWERS_PATH;

  Serial.print("[SERVER TEST] URL: ");
  Serial.println(url);

  configureHttp(http);

  if (!http.begin(client, url)) {
    Serial.println("[SERVER TEST] HTTP initialization failed");
    return;
  }

  int code = http.GET();
  String body = code > 0 ? http.getString() : "";

  printHttpResult("SERVER TEST", code, body);

  http.end();
}

// ============================================================
// 21. 명령 결과 전송
// ============================================================

bool sendAck(
  const String& commandId,
  const String& status,
  const String& message
) {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  WiFiClient client;
  HTTPClient http;

  String url = (
    String(SERVER_BASE_URL) +
    COMMAND_ACK_PREFIX +
    commandId +
    "/ack"
  );

  configureHttp(http);

  if (!http.begin(client, url)) {
    Serial.println("[ACK] HTTP initialization failed");
    return false;
  }

  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-Device-Token", DEVICE_TOKEN);

  JsonDocument doc;

  doc["device_id"] = DEVICE_ID;
  doc["status"] = status;
  doc["message"] = message;

  String requestBody;
  serializeJson(doc, requestBody);

  int code = http.POST(requestBody);
  String body = code > 0 ? http.getString() : "";

  printHttpResult("ACK", code, body);

  http.end();

  return code >= 200 && code < 300;
}

void processPendingAck() {
  if (!ackReady) {
    return;
  }

  if (millis() - lastAckAttemptMs < ACK_RETRY_MS) {
    return;
  }

  lastAckAttemptMs = millis();

  bool success = sendAck(
    pendingAckId,
    pendingAckStatus,
    pendingAckMessage
  );

  if (success) {
    lastAckedCommandId = pendingAckId;
    lastAckedStatus = pendingAckStatus;
    lastAckedMessage = pendingAckMessage;

    pendingAckId = "";
    pendingAckStatus = "";
    pendingAckMessage = "";

    ackReady = false;
  }
}

// ============================================================
// 22. 서버 명령 조회
// ============================================================

void pollCommands() {
  if (
    WiFi.status() != WL_CONNECTED ||
    st != St::IDLE ||
    anyMoving() ||
    ackReady ||
    activeCommandId.length() > 0
  ) {
    return;
  }

  WiFiClient client;
  HTTPClient http;

  String url = (
    String(SERVER_BASE_URL) +
    COMMAND_NEXT_PATH +
    "?device_id=" +
    DEVICE_ID
  );

  configureHttp(http);

  if (!http.begin(client, url)) {
    Serial.println("[COMMAND GET] HTTP initialization failed");
    return;
  }

  http.addHeader("X-Device-Token", DEVICE_TOKEN);

  int code = http.GET();
  String body = code > 0 ? http.getString() : "";

  printHttpResult("COMMAND GET", code, body);

  // 응답을 읽은 뒤 연결부터 정리합니다.
  http.end();

  if (code == 204) {
    Serial.println("[COMMAND GET] No command");
    return;
  }

  if (code != 200) {
    return;
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, body);

  if (error) {
    Serial.print("[COMMAND GET] JSON error: ");
    Serial.println(error.c_str());
    return;
  }

  // 서버가 success:false를 보낸 경우 실행하지 않습니다.
  if (
    doc["success"].is<bool>() &&
    !doc["success"].as<bool>()
  ) {
    Serial.println("[COMMAND GET] Server reported success=false");
    return;
  }

  // 두 응답 형태를 처리합니다.
  //
  // {"command": {...}}
  // {"success": true, "data": {"command": {...}}}
  //
  // 실제 서버 구조가 다르면 로그를 보고 맞춰야 합니다.
  JsonVariant command = doc["command"];

  if (command.isNull()) {
    command = doc["data"]["command"];
  }

  if (command.isNull()) {
    Serial.println(
      "[COMMAND GET] command is null/missing; "
      "idle response or response schema needs checking"
    );
    return;
  }

  if (!command.is<JsonObject>()) {
    Serial.println("[COMMAND GET] command must be an object");
    return;
  }

  String commandId = "";

  if (!command["command_id"].isNull()) {
    commandId = command["command_id"].as<String>();
  } else if (!command["id"].isNull()) {
    commandId = command["id"].as<String>();
  }

  String action = command["action"].as<String>();

  if (
    commandId.length() == 0 ||
    action.length() == 0 ||
    command["drawer_no"].isNull()
  ) {
    Serial.println(
      "[COMMAND GET] Missing command_id/id, action, or drawer_no"
    );
    return;
  }

  int drawerNo = command["drawer_no"].as<int>();

  Serial.printf(
    "[COMMAND] id=%s action=%s drawer_no=%d\n",
    commandId.c_str(),
    action.c_str(),
    drawerNo
  );

  // 이미 처리한 명령이면 서보를 다시 움직이지 않고
  // 이전 결과만 다시 전송합니다.
  if (commandId == lastAckedCommandId) {
    Serial.println("[COMMAND] Duplicate; resending previous ACK");

    queueAck(
      lastAckedCommandId,
      lastAckedStatus,
      lastAckedMessage
    );

    return;
  }

  if (drawerNo < 1 || drawerNo > 3) {
    queueAck(
      commandId,
      "failed",
      "drawer_no must be 1, 2, or 3"
    );

    return;
  }

  // 기존 코드에서 확인된 action만 실행합니다.
  if (action != "unlock_drawer") {
    Serial.println(
      "[COMMAND] Unsupported action; check server API specification"
    );

    return;
  }

  // 서버 1,2,3 -> 하드웨어 배열 0,1,2
  uint8_t hardwareIndex = (uint8_t)(drawerNo - 1);

  bool started = startUserOpen(hardwareIndex);

  if (started) {
    activeCommandId = commandId;

    Serial.println(
      "[COMMAND] Unlock started; ACK waits for servo motion end"
    );
  } else {
    queueAck(
      commandId,
      "failed",
      "drawer controller is busy or cannot start"
    );
  }
}

// ============================================================
// 23. 센서 상태 전송
// ============================================================

bool sendSensorState(
  int drawerNo,
  int occupied,
  float currentWeight
) {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  WiFiClient client;
  HTTPClient http;

  String url = String(SERVER_BASE_URL) + SENSOR_PATH;

  configureHttp(http);

  if (!http.begin(client, url)) {
    Serial.println("[SENSOR POST] HTTP initialization failed");
    return false;
  }

  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-Device-Token", DEVICE_TOKEN);

  JsonDocument doc;

  doc["drawer_no"] = drawerNo;
  doc["occupied"] = occupied;
  doc["current_weight_g"] = currentWeight;

  String requestBody;
  serializeJson(doc, requestBody);

  Serial.print("[SENSOR POST] REQUEST: ");
  Serial.println(requestBody);

  int code = http.POST(requestBody);
  String body = code > 0 ? http.getString() : "";

  printHttpResult("SENSOR POST", code, body);

  http.end();

  return code >= 200 && code < 300;
}

void uploadNextSensor() {
  if (
    !ENABLE_SENSOR_UPLOAD ||
    st != St::IDLE ||
    anyMoving()
  ) {
    return;
  }

  // 한 번에 하나씩 전송해서 연속 HTTP 대기를 줄입니다.
  for (uint8_t checked = 0; checked < 3; checked++) {
    uint8_t d = nextSensorIndex;
    nextSensorIndex = (nextSensorIndex + 1) % 3;

    if (!isWeightStable(d)) {
      continue;
    }

    float rawWeight = getWeight(d);

    if (!isfinite(rawWeight)) {
      continue;
    }

    // 미세한 음수 영점 노이즈는 서버에 0으로 보냅니다.
    // 큰 음수가 계속되면 보정값/배선/영점을 확인해야 합니다.
    float weight = rawWeight < 0 ? 0.0f : rawWeight;

    if (occupiedState[d] < 0) {
      // 최초 값이 판정 경계 안에 있으면 확정하지 않습니다.
      if (weight >= OCCUPIED_ON_G) {
        occupiedState[d] = 1;
      } else if (weight <= OCCUPIED_OFF_G) {
        occupiedState[d] = 0;
      } else {
        continue;
      }
    } else if (
      occupiedState[d] == 0 &&
      weight >= OCCUPIED_ON_G
    ) {
      occupiedState[d] = 1;
    } else if (
      occupiedState[d] == 1 &&
      weight <= OCCUPIED_OFF_G
    ) {
      occupiedState[d] = 0;
    }

    sendSensorState(
      d + 1,
      occupiedState[d],
      weight
    );

    return;
  }
}

// ============================================================
// 24. 네트워크 작업 스케줄링
// ============================================================

void updateNetwork() {
  updateWifi();

  if (
    WiFi.status() != WL_CONNECTED ||
    anyMoving()
  ) {
    return;
  }

  // ACK는 잠금 해제 후 사용자 대기 중에도 보냅니다.
  if (ackReady) {
    processPendingAck();
    return;
  }

  // 나머지 통신은 하드웨어 작업이 없을 때만 시작합니다.
  if (st != St::IDLE) {
    return;
  }

  if (serverTestNeeded) {
    serverTestNeeded = false;

    testServerConnection();
    return;
  }

  uint32_t now = millis();

  if (now - lastPollMs >= COMMAND_POLL_MS) {
    lastPollMs = now;

    pollCommands();
    return;
  }

  if (
    ENABLE_SENSOR_UPLOAD &&
    now - lastSensorMs >= SENSOR_SEND_MS
  ) {
    lastSensorMs = now;

    uploadNextSensor();
  }
}

// ============================================================
// 25. 시리얼 명령
// ============================================================
//
// 아래 직접 제어 명령의 번호는 기존처럼 0,1,2입니다.
// 서버 API의 drawer_no는 1,2,3입니다.
//
// U0/U1/U2 : 직접 잠금 해제
// L0/L1/L2 : 직접 잠금
// O0/O1/O2 : 구분면 열기
// C0/C1/C2 : 구분면 닫기
// P       : 인식판 열기
// Q       : 인식판 닫기
// W0/W1/W2 : 무게 확인
// T0/T1/T2 : 빈 상태에서 영점 잡기
// E1/E0   : LED
// D0/D1/D2 : 기존 분배 시퀀스
// K0/K1/K2 : 사용자 개방 시퀀스
// X       : 동작 중단
// S       : 상태 출력
// N       : 서버 조회 테스트 예약
// V1/V0   : 센서 서버 전송 켜기/끄기
//
// 줄바꿈을 포함해 전송하세요.
// ============================================================

void printStatus() {
  Serial.printf(
    "STAT st=%u wifi_status=%d sensor_upload=%d ack_pending=%d\n",
    (unsigned int)st,
    (int)WiFi.status(),
    ENABLE_SENSOR_UPLOAD ? 1 : 0,
    ackReady ? 1 : 0
  );

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());
  }

  for (uint8_t i = 0; i < 3; i++) {
    Serial.printf(
      "hardware_index=%u drawer_no=%u "
      "loadcell_ok=%d weight=%.2f occupied=%d %s\n",
      i,
      i + 1,
      lc[i].ok ? 1 : 0,
      lc[i].weight,
      occupiedState[i],
      drawerClosed(i) ? "closed" : "open"
    );
  }
}

void handleSerial() {
  static char buf[16];
  static uint8_t len = 0;

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c != '\n') {
      if (len < sizeof(buf) - 1) {
        buf[len++] = c;
      } else {
        len = 0;
      }

      continue;
    }

    buf[len] = '\0';
    len = 0;

    if (buf[0] == '\0') {
      continue;
    }

    char cmd = buf[0];

    bool hasNum = (
      buf[1] >= '0' &&
      buf[1] <= '2' &&
      buf[2] == '\0'
    );

    bool noArg = buf[1] == '\0';

    uint8_t n = hasNum ? buf[1] - '0' : 0;

    // 서버 명령 중 직접 제어가 끼어드는 것을 막습니다.
    bool manualMotionCommand = (
      cmd == 'U' ||
      cmd == 'L' ||
      cmd == 'O' ||
      cmd == 'C' ||
      cmd == 'P' ||
      cmd == 'Q' ||
      cmd == 'D' ||
      cmd == 'K' ||
      cmd == 'T'
    );

    if (
      manualMotionCommand &&
      (
        st != St::IDLE ||
        anyMoving() ||
        ackReady ||
        activeCommandId.length() > 0
      )
    ) {
      Serial.println("ERR BUSY");
      continue;
    }

    bool ok = true;

    switch (cmd) {
      case 'U':
        if (hasNum) unlockDrawer(n);
        else ok = false;
        break;

      case 'L':
        if (hasNum) lockDrawer(n);
        else ok = false;
        break;

      case 'O':
        if (hasNum) moveDivider(n, true);
        else ok = false;
        break;

      case 'C':
        if (hasNum) moveDivider(n, false);
        else ok = false;
        break;

      case 'P':
        if (noArg) openPlate();
        else ok = false;
        break;

      case 'Q':
        if (noArg) closePlate();
        else ok = false;
        break;

      case 'W':
        if (hasNum) {
          Serial.println(getWeight(n), 2);
          continue;
        }

        ok = false;
        break;

      case 'T':
        if (hasNum && lc[n].ok) tareLoadCell(n);
        else ok = false;
        break;

      case 'E':
        if (
          (buf[1] == '0' || buf[1] == '1') &&
          buf[2] == '\0'
        ) {
          setLed(buf[1] == '1');
        } else {
          ok = false;
        }
        break;

      case 'D':
        if (hasNum) ok = startDispense(n);
        else ok = false;
        break;

      case 'K':
        if (hasNum) ok = startUserOpen(n);
        else ok = false;
        break;

      case 'X':
        if (noArg) abortJob();
        else ok = false;
        break;

      case 'S':
        if (noArg) {
          printStatus();
          continue;
        }

        ok = false;
        break;

      case 'N':
        if (noArg) serverTestNeeded = true;
        else ok = false;
        break;

      case 'V':
        if (
          (buf[1] == '0' || buf[1] == '1') &&
          buf[2] == '\0'
        ) {
          ENABLE_SENSOR_UPLOAD = buf[1] == '1';
        } else {
          ok = false;
        }
        break;

      default:
        ok = false;
        break;
    }

    Serial.println(ok ? "OK" : "ERR");
  }
}

// ============================================================
// 26. setup
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("===== SUNNY HX CHECK V2 =====");
  Serial.printf("BUILD: %s %s\n", __DATE__, __TIME__);
  Serial.flush();

  Serial.println();
  Serial.println("===== YAKJIKYO ESP32 START =====");
  Serial.println("Server drawer numbers: 1, 2, 3");
  Serial.println("Serial hardware indexes: 0, 1, 2");

  Serial.print("SERVER: ");
  Serial.println(SERVER_BASE_URL);

  pinMode(PIN_LED, OUTPUT);
  setLed(false);

  for (uint8_t i = 0; i < 3; i++) {
    pinMode(PIN_MAG[i], INPUT_PULLUP);

    magStable[i] = (
      digitalRead(PIN_MAG[i]) == MAG_CLOSED_LEVEL
    );
  }

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ_HZ);

  delay(10);

  for (uint8_t ch = 0; ch < CH_COUNT; ch++) {
    writeAngle(ch, CFG[ch].homeDeg);
    delay(150);
  }

  // 기존 코드와 동일하게 부팅 시 자동 영점
  initLoadCells();

  lcLastMoveMs = millis();

  if (ENABLE_NETWORK) {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    beginWifiConnection();
  } else {
    Serial.println("[WIFI] Network disabled - offline test mode");
  }

  Serial.println("READY");
  Serial.println("Open Serial Monitor at 115200 baud.");
  Serial.println("Send S for status, N for server test.");
}

// ============================================================
// 27. loop
// ============================================================

void loop() {
  updateServos();
  updateSensors();
  updateLoadCells();
  updateSequence();
  handleSerial();

  if (ENABLE_NETWORK) {
    updateNetwork();
  }
}
