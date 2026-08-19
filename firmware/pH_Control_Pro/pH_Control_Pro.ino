/*
 * ============================================================================
 *  pH Control Pro v1.0  —  ระบบควบคุม pH อัตโนมัติ (ESP32 + Firebase RTDB)
 * ============================================================================
 *  พัฒนาต่อจาก Water Quality Control System v9.0 โดย:
 *    - ตัดส่วน EC / ปุ๋ย A-B ออกทั้งหมด เหลือคุมเฉพาะ pH
 *    - ตัดระบบ KNN Machine Learning ออก ใช้ตรรกะกฎธรรมดา (rule-based)
 *    - เปลี่ยนจาก Blynk มาใช้ Google Firebase Realtime Database
 *
 *  ฮาร์ดแวร์  : ESP32 DevKit v1 (esp32:esp32:esp32)
 *  ไลบรารี    : Firebase Arduino Client for ESP8266/ESP32 (mobizt) >= 4.4.17
 *               OneWire, DallasTemperature, LiquidCrystal I2C
 *
 *  ก่อนคอมไพล์: คัดลอก secrets.h.example เป็น secrets.h แล้วใส่ค่าจริง
 * ============================================================================
 */

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <time.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_task_wdt.h>
#include <cmath>
#include <algorithm>

#include "secrets.h"

// ============================================================================
//  กำหนดขา (Pin Definitions)
// ============================================================================
#define PH_PIN             36     // ADC1_CH0 — ขา input only ใช้กับ ADC ได้
#define ONE_WIRE_BUS       4      // DS18B20
#define CALIB_BUTTON       0      // ปุ่ม BOOT บนบอร์ด

#define PH_UP_RELAY        26
#define PH_DOWN_RELAY      27
#define MIXER_RELAY        16

#define RELAY_ON           LOW    // รีเลย์โมดูลทั่วไปเป็น active LOW
#define RELAY_OFF          HIGH

// ============================================================================
//  ค่าคงที่ระบบ
// ============================================================================
#define FW_VERSION         "1.0.0"

// น้ำยา buffer มาตรฐานสำหรับ calibrate 3 จุด
#define CALIB_PH_1         4.01
#define CALIB_PH_2         6.86
#define CALIB_PH_3         9.18
#define CALIB_STEP_TIMEOUT 45000UL   // ถ้าไม่กดปุ่มภายใน 45 วิ จะเก็บค่าให้เอง
#define CALIB_MIN_R2       90.0      // R² ต่ำกว่านี้ถือว่า calibrate ไม่ผ่าน

#define WDT_TIMEOUT_SEC    30
#define MAX_SENSOR_RETRIES 3
#define SENSOR_ERROR_LIMIT 5         // อ่านพลาดติดกันกี่ครั้งถึงจะหยุดฉุกเฉิน

// ── ขีดจำกัดความปลอดภัย ──
// กันกรณีเซนเซอร์เพี้ยนแล้วระบบสั่งจ่ายสารเคมีรัวๆ จนเทลงถังทั้งขวด
#define MAX_DOSES_PER_HOUR 6
#define DOSE_WINDOW_MS     3600000UL

// ── ค่า default (ปรับได้จาก Dashboard ภายหลัง) ──
#define DEF_TARGET_PH      6.0
#define DEF_PH_TOLERANCE   0.3       // เดิม 1.2 กว้างเกินไปจนระบบแทบไม่ทำงาน
#define DEF_DOSING_MS      3000
#define DEF_COOLDOWN_MS    180000UL  // เดิม 30 วิ ไม่พอให้น้ำผสมกันจน pH นิ่ง

// ── EEPROM ──
#define EEPROM_MAGIC       0x50484331UL   // "PHC1"
#define EEPROM_VERSION     1

// ── จังหวะส่งข้อมูลขึ้น Firebase ──
#define FB_STATUS_ACTIVE_MS    5000
#define FB_STATUS_IDLE_MS      15000
#define FB_HISTORY_MS          300000UL   // เก็บกราฟย้อนหลังทุก 5 นาที
#define FB_NOT_READY_BACKOFF   5000
#define FB_WIFI_RETRY_MS       30000
#define FB_MAX_CONSEC_FAIL     5
#define FB_SIGNUP_RETRY        3
#define FB_SIGNUP_DELAY_MS     3000

// ============================================================================
//  ออบเจกต์ฮาร์ดแวร์
// ============================================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);
OneWire           oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);

// ============================================================================
//  ออบเจกต์ Firebase
// ============================================================================
FirebaseData   fbdo;         // ใช้ส่งข้อมูลขาออก
FirebaseData   fbdoStream;   // ใช้เฉพาะ stream /control (1 stream ต่อ 1 object)
FirebaseAuth   fbAuth;
FirebaseConfig fbConfig;
FirebaseJson   gJson;

// ============================================================================
//  โครงสร้างข้อมูล
// ============================================================================

// บันทึกลง EEPROM — เปลี่ยน layout เมื่อไหร่ต้องขยับ EEPROM_VERSION ด้วย
struct Config {
  uint32_t magic;
  uint16_t version;
  uint16_t _pad0;

  float    phSlope;             // pH = slope * โวลต์ + intercept
  float    phIntercept;
  float    phCalibVoltage[3];   // โวลต์ที่วัดได้ตอน calibrate แต่ละจุด
  float    phR2;                // ความแม่นของเส้น calibration (%)
  int32_t  calibrationCount;
  uint8_t  isPhCalibrated;
  uint8_t  autoMode;
  uint8_t  _pad1[2];

  float    targetPH;
  float    phTolerance;
  int32_t  dosingTimeMs;
  int32_t  cooldownMs;

  uint32_t checksum;            // ต้องเป็นฟิลด์สุดท้ายเสมอ
};

struct SystemState {
  float         currentPH;
  float         currentTemp;
  bool          dosingInProgress;
  bool          calibMode;
  int           calibStep;
  char          lastAction[16];
  unsigned long lastDosingTime;
  unsigned long lastDosingEnd;
  int           sensorErrorCount;
  bool          emergencyStop;
  bool          doseLimitHit;
};

// คำสั่งที่รับมาจาก Dashboard ผ่าน stream
enum CmdType : uint8_t {
  CMD_PH_UP, CMD_PH_DOWN, CMD_MIXER, CMD_START_CALIB,
  CMD_ESTOP_ON, CMD_ESTOP_OFF, CMD_REBOOT
};

#define ALERT_MSG_LEN 96
struct AlertMsg {
  char    message[ALERT_MSG_LEN];
  uint8_t priority;   // 0=info 1=warn 2=error
};

// ============================================================================
//  ตัวแปรกลาง
// ============================================================================
Config      cfg;
SystemState state;

SemaphoreHandle_t lcdMutex;
SemaphoreHandle_t sensorMutex;
SemaphoreHandle_t dosingMutex;
SemaphoreHandle_t calibMutex;

QueueHandle_t xCmdQueue;
QueueHandle_t xAlertQueue;

TaskHandle_t hTaskSensor   = NULL;
TaskHandle_t hTaskControl  = NULL;
TaskHandle_t hTaskDisplay  = NULL;
TaskHandle_t hTaskFirebase = NULL;

// สถานะเครือข่าย
volatile bool     gWiFiConnected  = false;
volatile bool     gFirebaseReady  = false;
volatile bool     gSignUpOK       = false;
volatile bool     gStreamStarted  = false;
volatile uint32_t gFbSendCount    = 0;
volatile uint32_t gFbFailCount    = 0;

// ค่าตั้งที่รับมาจาก Dashboard รอนำไปใช้ (stream callback ห้ามบล็อกยาว
// จึงแค่พักค่าไว้ตรงนี้ แล้วให้ taskControl เอาไปเขียนลง cfg + EEPROM)
volatile uint32_t gSensorSeq      = 0;    // เพิ่มขึ้น 1 ทุกครั้งที่อ่านเซนเซอร์รอบใหม่

volatile bool  gCfgDirty          = false;
volatile float gPendingTargetPH   = NAN;
volatile float gPendingTolerance  = NAN;
volatile int   gPendingDosingSec  = -1;
volatile int   gPendingCooldownSec = -1;
volatile int   gPendingAutoMode   = -1;   // -1 = ไม่เปลี่ยน

// ตัวนับจำนวนครั้งที่จ่ายสารในรอบ 1 ชั่วโมง
unsigned long doseTimestamps[MAX_DOSES_PER_HOUR];
int           doseTsCount = 0;

// ตัวแปรระหว่าง calibrate
unsigned long calibStartTime = 0;

// ============================================================================
//  ประกาศฟังก์ชันล่วงหน้า
// ============================================================================
void  saveConfig();
void  loadConfig();
void  sendAlert(const char* msg, uint8_t priority = 1);
void  stopAllDosing();
bool  isDosingInProgress();

// ============================================================================
//  ยูทิลิตี้ — เวลา / checksum
// ============================================================================

// นับเวลาแบบทนการวนรอบของ millis() (ทุก ~49.7 วัน)
bool isTimeElapsed(unsigned long startTime, unsigned long interval) {
  return (unsigned long)(millis() - startTime) >= interval;
}

unsigned long getElapsedTime(unsigned long startTime) {
  return (unsigned long)(millis() - startTime);
}

uint32_t calculateChecksum(const Config* data) {
  uint32_t sum = 0;
  const uint8_t* ptr = (const uint8_t*)data;
  size_t len = sizeof(Config) - sizeof(uint32_t);
  for (size_t i = 0; i < len; i++) sum += ptr[i];
  return sum;
}

// ============================================================================
//  ระบบ Log
// ============================================================================
void logMessage(const char* level, const char* message) {
  unsigned long seconds = millis() / 1000;
  Serial.printf("[%02lu:%02lu:%02lu] [%s] %s\n",
                (seconds / 3600) % 24, (seconds / 60) % 60, seconds % 60,
                level, message);
}

void logInfo(const char* m)    { logMessage("INFO",  m); }
void logWarning(const char* m) { logMessage("WARN",  m); }
void logError(const char* m)   { logMessage("ERROR", m); }

// ============================================================================
//  LCD — ทุกฟังก์ชันจับ mutex ก่อนเสมอ
// ============================================================================
void updateLine(int line, const String& text) {
  String padded = text.substring(0, 16);
  while (padded.length() < 16) padded += " ";

  if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(1000))) {
    lcd.setCursor(0, line);
    lcd.print(padded);
    xSemaphoreGive(lcdMutex);
  }
}

void showMessage(const String& l1, const String& l2) {
  updateLine(0, l1);
  updateLine(1, l2);
}

// ============================================================================
//  อ่านเซนเซอร์
// ============================================================================

float getMedianValue(float* values, int count) {
  std::sort(values, values + count);
  if (count % 2 == 0) return (values[count / 2 - 1] + values[count / 2]) / 2.0;
  return values[count / 2];
}

float readTemperature() {
  for (int i = 0; i < MAX_SENSOR_RETRIES; i++) {
    tempSensor.requestTemperatures();
    float t = tempSensor.getTempCByIndex(0);
    if (t > -55 && t < 125) return t;
    vTaskDelay(pdMS_TO_TICKS(300));
  }
  logError("อ่านอุณหภูมิไม่สำเร็จ (DS18B20)");
  return NAN;
}

// อ่าน pH: เก็บหลายตัวอย่าง → ตัดค่าสุดขอบ 20% → หา median → แปลงเป็น pH
float readPH() {
  if (!cfg.isPhCalibrated) return NAN;

  for (int retry = 0; retry < MAX_SENSOR_RETRIES; retry++) {
    const int NUM_SAMPLES = 15;
    float voltages[NUM_SAMPLES];
    int   validSamples = 0;

    for (int i = 0; i < NUM_SAMPLES; i++) {
      int raw = analogRead(PH_PIN);
      if (raw > 10 && raw < 4090) voltages[validSamples++] = raw * (3.3 / 4095.0);
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (validSamples >= 10) {
      std::sort(voltages, voltages + validSamples);

      int startIdx = validSamples / 5;
      int endIdx   = validSamples - startIdx;
      int n        = endIdx - startIdx;

      float medianVoltage = getMedianValue(voltages + startIdx, n);
      float phValue = cfg.phSlope * medianVoltage + cfg.phIntercept;

      if (phValue >= 0.0 && phValue <= 14.0) return phValue;
    }

    vTaskDelay(pdMS_TO_TICKS(300));
  }

  logError("อ่านค่า pH ไม่สำเร็จ");
  return NAN;
}

// เก็บค่านิ่งๆ สำหรับใช้ตอน calibrate — ตัดค่าสุดขอบ 20% แล้วเฉลี่ย
float collectStableReading(int pin, int samples) {
  const int MAX_SAMPLES = 30;
  if (samples > MAX_SAMPLES) samples = MAX_SAMPLES;

  float readings[MAX_SAMPLES];
  for (int i = 0; i < samples; i++) {
    readings[i] = analogRead(pin) * (3.3 / 4095.0);
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  std::sort(readings, readings + samples);
  int startIdx = samples / 5;
  int endIdx   = samples - startIdx;

  float sum = 0;
  for (int i = startIdx; i < endIdx; i++) sum += readings[i];
  return sum / (endIdx - startIdx);
}

// ============================================================================
//  เข้าถึงสถานะระบบแบบ thread-safe
// ============================================================================

void updateSensorReadings(float ph, float temp) {
  if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(1000))) {
    state.currentPH   = ph;
    state.currentTemp = temp;
    gSensorSeq = gSensorSeq + 1;
    xSemaphoreGive(sensorMutex);
  }
}

void getSensorReadings(float* ph, float* temp) {
  if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(1000))) {
    *ph   = state.currentPH;
    *temp = state.currentTemp;
    xSemaphoreGive(sensorMutex);
  } else {
    *ph = NAN; *temp = NAN;
  }
}

bool isDosingInProgress() {
  bool result = false;
  if (xSemaphoreTake(dosingMutex, pdMS_TO_TICKS(1000))) {
    result = state.dosingInProgress;
    xSemaphoreGive(dosingMutex);
  }
  return result;
}

void setLastAction(const char* action) {
  strncpy(state.lastAction, action, sizeof(state.lastAction) - 1);
  state.lastAction[sizeof(state.lastAction) - 1] = '\0';
}

// ============================================================================
//  ควบคุมรีเลย์
// ============================================================================

void stopAllDosing() {
  if (xSemaphoreTake(dosingMutex, pdMS_TO_TICKS(1000))) {
    digitalWrite(PH_UP_RELAY,   RELAY_OFF);
    digitalWrite(PH_DOWN_RELAY, RELAY_OFF);
    digitalWrite(MIXER_RELAY,   RELAY_OFF);
    state.dosingInProgress = false;
    xSemaphoreGive(dosingMutex);
  }
}

void runMixer(unsigned int durationMs) {
  digitalWrite(MIXER_RELAY, RELAY_ON);
  vTaskDelay(pdMS_TO_TICKS(durationMs));
  digitalWrite(MIXER_RELAY, RELAY_OFF);
}

// เข้าเป้าแล้วหรือยัง (เทียบกับ tolerance)
bool isTargetReached() {
  float ph, temp;
  getSensorReadings(&ph, &temp);
  if (isnan(ph)) return false;
  return fabs(ph - cfg.targetPH) <= cfg.phTolerance;
}

// ค่านิ่งพอที่จะเชื่อถือได้ไหม — ดูส่วนเบี่ยงเบนมาตรฐานของ 5 ค่าล่าสุด
bool isReadingStable() {
  static float    window[5]  = {0};
  static int      index      = 0;
  static int      filled     = 0;
  static uint32_t lastSeq    = 0;
  static bool     lastResult = false;

  float ph, temp;
  getSensorReadings(&ph, &temp);
  if (isnan(ph)) { filled = 0; lastResult = false; return false; }

  // ฟังก์ชันนี้ถูกเรียกถี่กว่าที่เซนเซอร์อ่านค่าจริง ถ้ายัดค่าซ้ำลงหน้าต่าง
  // ส่วนเบี่ยงเบนจะเกือบศูนย์แล้วตอบว่า "นิ่ง" ทั้งที่ค่ายังไม่เข้าที่
  uint32_t seq = gSensorSeq;
  if (seq == lastSeq) return lastResult;
  lastSeq = seq;

  window[index] = ph;
  index = (index + 1) % 5;
  if (filled < 5) filled++;
  if (filled < 5) { lastResult = false; return false; }

  float mean = 0;
  for (int i = 0; i < 5; i++) mean += window[i];
  mean /= 5.0;

  float variance = 0;
  for (int i = 0; i < 5; i++) variance += pow(window[i] - mean, 2);

  lastResult = sqrt(variance / 5.0) < 0.1;
  return lastResult;
}

// ── ตัวนับจำนวนครั้งที่จ่ายสารต่อชั่วโมง ──
// กันเซนเซอร์เพี้ยนแล้วระบบสั่งจ่ายสารเคมีรัวๆ ลงถัง
bool canDoseNow() {
  unsigned long now = millis();

  // ตัดรายการที่เก่ากว่า 1 ชั่วโมงออกจากหน้าต่าง
  int keep = 0;
  for (int i = 0; i < doseTsCount; i++) {
    if ((unsigned long)(now - doseTimestamps[i]) < DOSE_WINDOW_MS) {
      doseTimestamps[keep++] = doseTimestamps[i];
    }
  }
  doseTsCount = keep;

  if (doseTsCount >= MAX_DOSES_PER_HOUR) {
    if (!state.doseLimitHit) {
      state.doseLimitHit = true;
      cfg.autoMode = 0;
      saveConfig();
      char msg[ALERT_MSG_LEN];
      snprintf(msg, sizeof(msg),
               "จ่ายสารครบ %d ครั้งใน 1 ชม. หยุดโหมดอัตโนมัติเพื่อความปลอดภัย",
               MAX_DOSES_PER_HOUR);
      logWarning(msg);
      sendAlert(msg, 2);
    }
    return false;
  }

  state.doseLimitHit = false;
  return true;
}

void recordDose() {
  if (doseTsCount < MAX_DOSES_PER_HOUR) doseTimestamps[doseTsCount++] = millis();
}

int dosesThisHour() {
  unsigned long now = millis();
  int count = 0;
  for (int i = 0; i < doseTsCount; i++) {
    if ((unsigned long)(now - doseTimestamps[i]) < DOSE_WINDOW_MS) count++;
  }
  return count;
}

// ── เริ่มจ่ายสาร ──
// manual = true คือสั่งจากหน้าเว็บ ข้ามการเช็ค cooldown ได้ แต่ยังเช็คลิมิตต่อชั่วโมง
bool startDosing(bool isUp, bool manual) {
  if (state.emergencyStop) {
    logWarning("อยู่ในโหมดหยุดฉุกเฉิน ไม่จ่ายสาร");
    return false;
  }

  if (isDosingInProgress()) {
    logWarning("กำลังจ่ายสารอยู่แล้ว ข้ามคำสั่งนี้");
    return false;
  }

  if (!manual && !isTimeElapsed(state.lastDosingEnd, (unsigned long)cfg.cooldownMs)) {
    return false;
  }

  if (!canDoseNow()) return false;

  stopAllDosing();
  vTaskDelay(pdMS_TO_TICKS(100));

  if (xSemaphoreTake(dosingMutex, pdMS_TO_TICKS(1000))) {
    digitalWrite(isUp ? PH_UP_RELAY : PH_DOWN_RELAY, RELAY_ON);
    digitalWrite(MIXER_RELAY, RELAY_ON);
    state.lastDosingTime   = millis();
    state.dosingInProgress = true;
    setLastAction(isUp ? "pH Up" : "pH Down");
    xSemaphoreGive(dosingMutex);
  }

  recordDose();

  char msg[ALERT_MSG_LEN];
  snprintf(msg, sizeof(msg), "เริ่มจ่าย %s (%s) นาน %d วิ",
           isUp ? "pH Up" : "pH Down", manual ? "สั่งมือ" : "อัตโนมัติ",
           cfg.dosingTimeMs / 1000);
  logInfo(msg);
  sendAlert(msg, 0);
  return true;
}

// ============================================================================
//  EEPROM
// ============================================================================

void saveConfig() {
  cfg.magic    = EEPROM_MAGIC;
  cfg.version  = EEPROM_VERSION;
  cfg.checksum = calculateChecksum(&cfg);
  EEPROM.put(0, cfg);
  EEPROM.commit();
  logInfo("บันทึกค่าตั้งลง EEPROM แล้ว");
}

void loadDefaults() {
  memset(&cfg, 0, sizeof(Config));
  cfg.magic            = EEPROM_MAGIC;
  cfg.version          = EEPROM_VERSION;
  cfg.phSlope          = -3.5;      // ค่าเริ่มต้นคร่าวๆ ต้อง calibrate จริงอยู่ดี
  cfg.phIntercept      = 15.0;
  cfg.phR2             = 0.0;
  cfg.isPhCalibrated   = 0;
  cfg.autoMode         = 0;         // เริ่มต้นปิดไว้ก่อน กันจ่ายสารตั้งแต่ยังไม่ calibrate
  cfg.calibrationCount = 0;
  cfg.targetPH         = DEF_TARGET_PH;
  cfg.phTolerance      = DEF_PH_TOLERANCE;
  cfg.dosingTimeMs     = DEF_DOSING_MS;
  cfg.cooldownMs       = DEF_COOLDOWN_MS;
}

void loadConfig() {
  Config loaded;
  EEPROM.get(0, loaded);

  bool valid = (loaded.magic == EEPROM_MAGIC)
            && (loaded.version == EEPROM_VERSION)
            && (loaded.checksum == calculateChecksum(&loaded))
            && !isnan(loaded.phSlope)
            && !isnan(loaded.phIntercept);

  if (valid) {
    cfg = loaded;
    char msg[128];
    snprintf(msg, sizeof(msg), "โหลดค่าตั้งจาก EEPROM: calibrated=%s R2=%.1f%% ครั้งที่ %d",
             cfg.isPhCalibrated ? "ใช่" : "ยัง", cfg.phR2, (int)cfg.calibrationCount);
    logInfo(msg);
  } else {
    logWarning("ไม่พบค่าตั้งเดิม (หรือ layout เปลี่ยน) — ใช้ค่า default และต้อง calibrate ใหม่");
    loadDefaults();
    saveConfig();
  }
}

// ============================================================================
//  Calibration — 3 จุด (pH 4.01 / 6.86 / 9.18) แล้วหาเส้นตรงด้วย linear regression
// ============================================================================

void calculatePHCalibration() {
  float knownPH[3] = {CALIB_PH_1, CALIB_PH_2, CALIB_PH_3};

  float sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
  for (int i = 0; i < 3; i++) {
    sum_x  += cfg.phCalibVoltage[i];
    sum_y  += knownPH[i];
    sum_xy += cfg.phCalibVoltage[i] * knownPH[i];
    sum_x2 += cfg.phCalibVoltage[i] * cfg.phCalibVoltage[i];
  }

  float denom = (3 * sum_x2 - sum_x * sum_x);
  if (fabs(denom) < 1e-9) {
    // ทั้ง 3 จุดได้โวลต์เท่ากันหมด = เซนเซอร์ไม่ตอบสนอง
    cfg.phR2           = 0.0;
    cfg.isPhCalibrated = 0;
    logError("Calibrate ล้มเหลว: ทั้ง 3 จุดได้แรงดันเท่ากัน ตรวจสายเซนเซอร์");
    showMessage("CALIB FAILED", "Check sensor");
    vTaskDelay(pdMS_TO_TICKS(3000));
    return;
  }

  cfg.phSlope     = (3 * sum_xy - sum_x * sum_y) / denom;
  cfg.phIntercept = (sum_y - cfg.phSlope * sum_x) / 3;

  // คำนวณ R² ดูว่าเส้นที่ได้ทาบกับ 3 จุดดีแค่ไหน
  float ss_tot = 0, ss_res = 0;
  float mean_y = sum_y / 3;
  for (int i = 0; i < 3; i++) {
    float predicted = cfg.phSlope * cfg.phCalibVoltage[i] + cfg.phIntercept;
    ss_res += pow(knownPH[i] - predicted, 2);
    ss_tot += pow(knownPH[i] - mean_y, 2);
  }

  cfg.phR2           = (ss_tot > 0) ? (1 - ss_res / ss_tot) * 100.0 : 0.0;
  cfg.isPhCalibrated = (cfg.phR2 > CALIB_MIN_R2) ? 1 : 0;

  char msg[128];
  snprintf(msg, sizeof(msg), "Calibration: slope=%.3f intercept=%.3f R2=%.1f%% -> %s",
           cfg.phSlope, cfg.phIntercept, cfg.phR2,
           cfg.isPhCalibrated ? "ผ่าน" : "ไม่ผ่าน");
  logInfo(msg);
  sendAlert(msg, cfg.isPhCalibrated ? 0 : 2);

  showMessage(cfg.isPhCalibrated ? "CALIB OK" : "CALIB FAILED",
              "R2: " + String(cfg.phR2, 1) + "%");
  vTaskDelay(pdMS_TO_TICKS(3000));
}

void startCalibration() {
  if (xSemaphoreTake(calibMutex, pdMS_TO_TICKS(1000))) {
    state.calibMode = true;
    state.calibStep = 1;
    calibStartTime  = millis();
    xSemaphoreGive(calibMutex);
  }

  stopAllDosing();
  showMessage("CALIBRATION", "Starting...");
  logInfo("=== เริ่ม Calibration ===");
  sendAlert("เริ่ม calibrate pH 3 จุด", 0);
  vTaskDelay(pdMS_TO_TICKS(1500));
}

// FSM ของ calibration — เรียกซ้ำจาก taskSensor ทุกรอบ
void processCalibration() {
  if (!xSemaphoreTake(calibMutex, pdMS_TO_TICKS(1000))) return;

  // ── ขั้นที่ 1-3: จุ่มน้ำยา buffer แล้วกดปุ่ม BOOT ──
  if (state.calibStep >= 1 && state.calibStep <= 3) {
    const char* solutions[3] = {"Put pH 4.01", "Put pH 6.86", "Put pH 9.18"};
    const char* stepNames[3] = {"pH STEP 1/3", "pH STEP 2/3", "pH STEP 3/3"};
    int idx = state.calibStep - 1;

    int   timeLeft = max(0, (int)(CALIB_STEP_TIMEOUT / 1000) - (int)(getElapsedTime(calibStartTime) / 1000));
    float voltage  = analogRead(PH_PIN) * (3.3 / 4095.0);

    // สลับข้อความบรรทัดบนทุก 2 วิ ให้เห็นทั้งเลขขั้นและน้ำยาที่ต้องใช้
    static unsigned long lastToggle = 0;
    static bool showInstruction = false;
    if (isTimeElapsed(lastToggle, 2000)) {
      showInstruction = !showInstruction;
      lastToggle = millis();
    }

    updateLine(0, showInstruction ? solutions[idx] : stepNames[idx]);
    updateLine(1, "V:" + String(voltage, 3) + " T:" + String(timeLeft) + "s");

    bool captured = false;
    if (digitalRead(CALIB_BUTTON) == LOW) {
      vTaskDelay(pdMS_TO_TICKS(300));
      cfg.phCalibVoltage[idx] = collectStableReading(PH_PIN, 30);
      captured = true;
    } else if (isTimeElapsed(calibStartTime, CALIB_STEP_TIMEOUT)) {
      cfg.phCalibVoltage[idx] = collectStableReading(PH_PIN, 20);
      captured = true;
    }

    if (captured) {
      char msg[64];
      snprintf(msg, sizeof(msg), "จุดที่ %d: %.4f V", state.calibStep, cfg.phCalibVoltage[idx]);
      logInfo(msg);

      showMessage("pH " + String(state.calibStep) + " SAVED",
                  "V: " + String(cfg.phCalibVoltage[idx], 4));
      vTaskDelay(pdMS_TO_TICKS(2000));

      state.calibStep++;
      calibStartTime = millis();
    }
  }
  // ── ขั้นที่ 4: คำนวณและบันทึก ──
  else if (state.calibStep >= 4) {
    calculatePHCalibration();
    cfg.calibrationCount++;
    saveConfig();

    showMessage("CALIBRATION", "COMPLETED!");
    logInfo("=== Calibration เสร็จสิ้น ===");
    vTaskDelay(pdMS_TO_TICKS(2000));

    state.calibMode = false;
    state.calibStep = 0;
  }

  xSemaphoreGive(calibMutex);
}

// ============================================================================
//  WiFi + NTP
// ============================================================================

bool wifiConnect() {
  Serial.printf("[WiFi] กำลังเชื่อมต่อ %s ", WIFI_SSID);

  WiFi.mode(WIFI_STA);
  // ล้าง state เก่าก่อน — ESP32 core 3.x มีอาการจำสถานะค้างแล้วต่อไม่ติด
  WiFi.disconnect(true, true);
  vTaskDelay(pdMS_TO_TICKS(1000));

  // ลดกำลังส่งตอนเริ่มต่อ กันไฟตกจากพอร์ต USB (brownout reset)
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
    esp_task_wdt_reset();
    attempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    gWiFiConnected = true;
    WiFi.setTxPower(WIFI_POWER_15dBm);   // ต่อติดแล้วเร่งกำลังกลับ ให้คุยกับ Firebase เสถียร
    Serial.printf("[WiFi] เชื่อมต่อสำเร็จ IP: %s  RSSI: %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    return true;
  }

  gWiFiConnected = false;
  logError("เชื่อมต่อ WiFi ไม่สำเร็จ");
  return false;
}

// ★ สำคัญมาก: ESP32 บูตมาด้วยเวลา 1 ม.ค. 1970 ทำให้ mbedTLS มองว่า certificate
//   ของ Firebase หมดอายุทุกใบ → SSL handshake ล้มเหลว
//   ต้อง sync เวลาจริงจาก NTP ก่อนเรียก firebaseInit() เสมอ
bool syncNTP() {
  logInfo("กำลัง sync เวลาจาก NTP...");
  configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com", "time.nist.gov");

  struct tm timeinfo = {0};
  time_t    now      = 0;

  for (int retry = 0; retry < 30; retry++) {
    time(&now);
    localtime_r(&now, &timeinfo);

    if (timeinfo.tm_year > (2024 - 1900)) {
      char buf[64];
      strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
      Serial.printf("[NTP] sync สำเร็จ: %s (ICT)\n", buf);
      return true;
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    esp_task_wdt_reset();
  }

  logError("sync เวลาไม่สำเร็จ — SSL อาจใช้งานไม่ได้");
  return false;
}

// ============================================================================
//  Firebase — stream callback
// ============================================================================

volatile bool gMixerWant = false;

static void queueCmd(CmdType c) {
  if (xCmdQueue) xQueueSend(xCmdQueue, &c, 0);
}

// แปลงค่าที่รับมาจาก /control ทีละคีย์
// หมายเหตุ: ฟังก์ชันนี้ทำงานใน context ของ stream task — ห้ามบล็อกยาว
// จึงแค่พักค่า/ยัดคิว แล้วให้ task อื่นไปทำงานจริง
void applyControlKV(const String& key, const String& val) {
  bool bv = (val == "true" || val == "1");

  if      (key == "targetPH")      { gPendingTargetPH    = val.toFloat(); gCfgDirty = true; }
  else if (key == "phTolerance")   { gPendingTolerance   = val.toFloat(); gCfgDirty = true; }
  else if (key == "dosingTimeSec") { gPendingDosingSec   = val.toInt();   gCfgDirty = true; }
  else if (key == "cooldownSec")   { gPendingCooldownSec = val.toInt();   gCfgDirty = true; }
  else if (key == "autoMode")      { gPendingAutoMode    = bv ? 1 : 0;    gCfgDirty = true; }
  else if (key == "phUp"          && bv) { queueCmd(CMD_PH_UP); }
  else if (key == "phDown"        && bv) { queueCmd(CMD_PH_DOWN); }
  else if (key == "startCalib"    && bv) { queueCmd(CMD_START_CALIB); }
  else if (key == "reboot"        && bv) { queueCmd(CMD_REBOOT); }
  else if (key == "mixer")               { gMixerWant = bv; queueCmd(CMD_MIXER); }
  else if (key == "emergencyStop")       { queueCmd(bv ? CMD_ESTOP_ON : CMD_ESTOP_OFF); }
}

void streamCallback(FirebaseStream data) {
  String path = data.dataPath();

  if (data.dataTypeEnum() == fb_esp_rtdb_data_type_json) {
    // อัปเดตมาทั้งก้อน — ไล่อ่านทีละคีย์
    FirebaseJson* json = data.to<FirebaseJson*>();
    if (json == NULL) return;

    size_t len = json->iteratorBegin();
    for (size_t i = 0; i < len; i++) {
      FirebaseJson::IteratorValue it = json->valueAt(i);
      applyControlKV(it.key, it.value);
    }
    json->iteratorEnd();
  } else if (path.length() > 1) {
    // อัปเดตคีย์เดียว เช่น "/phUp" — แปลงเป็นสตริงตามชนิดจริงก่อนส่งต่อ
    String val;
    switch (data.dataTypeEnum()) {
      case fb_esp_rtdb_data_type_boolean: val = data.to<bool>() ? "true" : "false"; break;
      case fb_esp_rtdb_data_type_integer: val = String(data.to<int>());              break;
      case fb_esp_rtdb_data_type_float:   val = String(data.to<float>(), 4);         break;
      case fb_esp_rtdb_data_type_double:  val = String((float)data.to<double>(), 4); break;
      case fb_esp_rtdb_data_type_string:  val = data.to<String>();                   break;
      default: return;
    }
    applyControlKV(path.substring(1), val);
  }
}

void streamTimeoutCallback(bool timeout) {
  if (timeout) logWarning("Firebase stream timeout — กำลังเชื่อมต่อใหม่");
  if (!fbdoStream.httpConnected()) {
    Serial.printf("[FB] stream error: %s\n", fbdoStream.errorReason().c_str());
  }
}

// ============================================================================
//  Firebase — init / อัปโหลด
// ============================================================================

void firebaseInit() {
  logInfo("กำลังเริ่มต้น Firebase...");

  fbConfig.api_key               = FIREBASE_API_KEY;
  fbConfig.database_url          = FIREBASE_DATABASE_URL;
  fbConfig.token_status_callback = tokenStatusCallback;

  // certificate chain ของ Firebase ยาว ต้องขยาย buffer ไม่งั้น handshake พัง
  fbdo.setBSSLBufferSize(4096, 1024);
  fbdo.setResponseSize(2048);
  fbdoStream.setBSSLBufferSize(2048, 1024);
  fbdoStream.setResponseSize(2048);

  // signUp ครั้งแรกมัก handshake ล้ม จึงต้องลองซ้ำ
  bool signedUp = false;
  for (int i = 0; i < FB_SIGNUP_RETRY && !signedUp; i++) {
    Serial.printf("[FB] SignUp ครั้งที่ %d/%d...\n", i + 1, FB_SIGNUP_RETRY);
    if (Firebase.signUp(&fbConfig, &fbAuth, "", "")) {
      signedUp = true;
      logInfo("Firebase Sign-Up สำเร็จ");
    } else {
      Serial.printf("[FB] Sign-Up ล้มเหลว: %s\n", fbConfig.signer.signupError.message.c_str());
      if (i < FB_SIGNUP_RETRY - 1) {
        vTaskDelay(pdMS_TO_TICKS(FB_SIGNUP_DELAY_MS));
        esp_task_wdt_reset();
      }
    }
  }
  gSignUpOK = signedUp;

  Firebase.begin(&fbConfig, &fbAuth);
  Firebase.reconnectNetwork(true);
}

void sendAlert(const char* msg, uint8_t priority) {
  if (xAlertQueue == NULL) return;
  AlertMsg a;
  strncpy(a.message, msg, ALERT_MSG_LEN - 1);
  a.message[ALERT_MSG_LEN - 1] = '\0';
  a.priority = priority;
  xQueueSend(xAlertQueue, &a, pdMS_TO_TICKS(50));
}

bool firebaseUploadAlert(const AlertMsg& a) {
  gJson.clear();
  gJson.set("message",       a.message);
  gJson.set("priority",      (int)a.priority);
  gJson.set("timestamp/.sv", "timestamp");
  bool ok = Firebase.RTDB.pushJSON(&fbdo, "/alerts", &gJson);
  gJson.clear();
  fbdo.clear();
  return ok;
}

bool firebaseUploadStatus() {
  float ph, temp;
  getSensorReadings(&ph, &temp);

  unsigned long cooldownLeft = 0;
  if (!isTimeElapsed(state.lastDosingEnd, (unsigned long)cfg.cooldownMs)) {
    cooldownLeft = ((unsigned long)cfg.cooldownMs - getElapsedTime(state.lastDosingEnd)) / 1000;
  }

  gJson.clear();
  gJson.set("ph",               isnan(ph) ? -1.0 : (double)ph);
  gJson.set("temp",             isnan(temp) ? -127.0 : (double)temp);
  gJson.set("phValid",          !isnan(ph));
  gJson.set("dosing",           isDosingInProgress());
  gJson.set("lastAction",       state.lastAction);
  gJson.set("cooldownSec",      (int)cooldownLeft);
  gJson.set("phUp",             (bool)(digitalRead(PH_UP_RELAY)   == RELAY_ON));
  gJson.set("phDown",           (bool)(digitalRead(PH_DOWN_RELAY) == RELAY_ON));
  gJson.set("mixer",            (bool)(digitalRead(MIXER_RELAY)   == RELAY_ON));
  gJson.set("calibrated",       (bool)cfg.isPhCalibrated);
  gJson.set("calibMode",        state.calibMode);
  gJson.set("calibStep",        state.calibStep);
  gJson.set("phR2",             (double)cfg.phR2);
  gJson.set("calibCount",       (int)cfg.calibrationCount);
  gJson.set("targetPH",         (double)cfg.targetPH);
  gJson.set("phTolerance",      (double)cfg.phTolerance);
  gJson.set("dosingTimeSec",    (int)(cfg.dosingTimeMs / 1000));
  gJson.set("cooldownConfSec",  (int)(cfg.cooldownMs / 1000));
  gJson.set("autoMode",         (bool)cfg.autoMode);
  gJson.set("emergencyStop",    state.emergencyStop);
  gJson.set("doseLimitHit",     state.doseLimitHit);
  gJson.set("dosesThisHour",    dosesThisHour());
  gJson.set("maxDosesPerHour",  MAX_DOSES_PER_HOUR);
  gJson.set("sensorErrorCount", state.sensorErrorCount);
  gJson.set("freeHeap",         (int)ESP.getFreeHeap());
  gJson.set("uptimeMin",        (int)(millis() / 60000));
  gJson.set("wifiRSSI",         WiFi.RSSI());
  gJson.set("fwVersion",        FW_VERSION);
  gJson.set("timestamp/.sv",    "timestamp");

  bool ok = Firebase.RTDB.setJSON(&fbdo, "/status", &gJson);
  gJson.clear();
  fbdo.clear();

  if (ok) gFbSendCount = gFbSendCount + 1;
  else    gFbFailCount = gFbFailCount + 1;
  return ok;
}

bool firebaseUploadReading(const char* path, bool push, float ph, float temp) {
  if (isnan(ph)) return false;

  gJson.clear();
  gJson.set("ph",            (double)ph);
  gJson.set("temp",          isnan(temp) ? -127.0 : (double)temp);
  gJson.set("timestamp/.sv", "timestamp");

  bool ok = push ? Firebase.RTDB.pushJSON(&fbdo, path, &gJson)
                 : Firebase.RTDB.setJSON(&fbdo, path, &gJson);
  gJson.clear();
  fbdo.clear();
  return ok;
}

// คีย์นี้ยังไม่มีบน /control ใช่ไหม (อ่านไม่ได้ก็ถือว่ายังไม่มี ปลอดภัยกว่าเดาว่ามี)
static bool controlKeyMissing(const char* key) {
  String path = String("/control/") + key;
  if (!Firebase.RTDB.get(&fbdo, path.c_str())) return true;
  return fbdo.dataType() == "null";
}

// เคลียร์ปุ่มค้างและเติมค่าตั้งเริ่มต้น ต้องทำ "ก่อน" เปิด stream
// ถ้าอุปกรณ์รีบูตตอนที่ /control/phUp ยังเป็น true อยู่ พอ stream เปิดจะสั่งจ่ายสารทันที
void firebaseSeedControl() {
  const char* buttons[] = {"phUp", "phDown", "mixer", "startCalib", "reboot"};
  for (int i = 0; i < 5; i++) {
    String p = String("/control/") + buttons[i];
    Firebase.RTDB.setBool(&fbdo, p.c_str(), false);
    esp_task_wdt_reset();
  }

  // เติมค่าตั้งเฉพาะคีย์ที่ยังไม่มีบน Firebase
  // เช็คทีละคีย์ ไม่ใช่เช็คคีย์เดียวแล้วข้ามทั้งก้อน — ถ้า /control ถูกเขียนไว้
  // บางส่วน (เช่นทดสอบด้วย REST หรือ Rules ปฏิเสธบางคีย์ไป) คีย์ที่เหลือจะหายถาวร
  int seeded = 0;
  if (controlKeyMissing("targetPH")) {
    Firebase.RTDB.setFloat(&fbdo, "/control/targetPH", cfg.targetPH); seeded++;
  }
  if (controlKeyMissing("phTolerance")) {
    Firebase.RTDB.setFloat(&fbdo, "/control/phTolerance", cfg.phTolerance); seeded++;
  }
  esp_task_wdt_reset();

  if (controlKeyMissing("dosingTimeSec")) {
    Firebase.RTDB.setInt(&fbdo, "/control/dosingTimeSec", (int)(cfg.dosingTimeMs / 1000)); seeded++;
  }
  if (controlKeyMissing("cooldownSec")) {
    Firebase.RTDB.setInt(&fbdo, "/control/cooldownSec", (int)(cfg.cooldownMs / 1000)); seeded++;
  }
  esp_task_wdt_reset();

  if (controlKeyMissing("autoMode")) {
    Firebase.RTDB.setBool(&fbdo, "/control/autoMode", (bool)cfg.autoMode); seeded++;
  }
  if (controlKeyMissing("emergencyStop")) {
    Firebase.RTDB.setBool(&fbdo, "/control/emergencyStop", false); seeded++;
  }
  esp_task_wdt_reset();

  if (seeded > 0) {
    char msg[64];
    snprintf(msg, sizeof(msg), "เติมค่าตั้งที่ขาดลง /control แล้ว %d คีย์", seeded);
    logInfo(msg);
  }

  fbdo.clear();
}

void firebaseStartStream() {
  if (Firebase.RTDB.beginStream(&fbdoStream, "/control")) {
    Firebase.RTDB.setStreamCallback(&fbdoStream, streamCallback, streamTimeoutCallback);
    gStreamStarted = true;
    logInfo("เปิด stream /control สำเร็จ");
  } else {
    Serial.printf("[FB] เปิด stream ไม่สำเร็จ: %s\n", fbdoStream.errorReason().c_str());
  }
}

// ============================================================================
//  Task 1: อ่านเซนเซอร์ + FSM calibration   (Core 1, prio 2)
// ============================================================================
void taskSensor(void* pv) {
  esp_task_wdt_add(NULL);

  for (;;) {
    esp_task_wdt_reset();

    // กดปุ่ม BOOT ค้าง 3 วินาที = เข้าโหมด calibrate
    if (digitalRead(CALIB_BUTTON) == LOW && !state.calibMode) {
      unsigned long pressStart = millis();
      bool longPress = false;
      while (digitalRead(CALIB_BUTTON) == LOW) {
        if (isTimeElapsed(pressStart, 3000)) { longPress = true; break; }
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_task_wdt_reset();
      }
      if (longPress) startCalibration();
    }

    if (state.calibMode) {
      processCalibration();
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }

    float temp = readTemperature();
    float ph   = readPH();
    updateSensorReadings(ph, temp);

    if (isnan(ph) || isnan(temp)) {
      // ยังไม่ calibrate ก็อ่าน pH ไม่ได้เป็นเรื่องปกติ ไม่นับเป็น error
      if (cfg.isPhCalibrated) {
        state.sensorErrorCount++;
        if (state.sensorErrorCount > SENSOR_ERROR_LIMIT && !state.emergencyStop) {
          state.emergencyStop = true;
          stopAllDosing();
          logError("เซนเซอร์อ่านพลาดติดกันหลายครั้ง — หยุดฉุกเฉิน");
          sendAlert("เซนเซอร์อ่านค่าไม่ได้ติดกันหลายครั้ง ระบบหยุดฉุกเฉิน", 2);
        }
      }
    } else {
      if (state.sensorErrorCount > 0) state.sensorErrorCount = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ============================================================================
//  Task 2: ตรรกะควบคุมการจ่ายสาร   (Core 1, prio 3)
// ============================================================================

// นำค่าตั้งที่รับมาจาก Dashboard ไปใช้จริงแล้วบันทึกลง EEPROM
void applyPendingConfig() {
  if (!gCfgDirty) return;
  gCfgDirty = false;

  bool changed = false;

  if (!isnan(gPendingTargetPH)) {
    float v = constrain(gPendingTargetPH, 3.0f, 11.0f);
    if (fabs(v - cfg.targetPH) > 0.001) { cfg.targetPH = v; changed = true; }
    gPendingTargetPH = NAN;
  }
  if (!isnan(gPendingTolerance)) {
    float v = constrain(gPendingTolerance, 0.05f, 2.0f);
    if (fabs(v - cfg.phTolerance) > 0.001) { cfg.phTolerance = v; changed = true; }
    gPendingTolerance = NAN;
  }
  if (gPendingDosingSec >= 0) {
    int32_t v = constrain(gPendingDosingSec, 1, 60) * 1000;
    if (v != cfg.dosingTimeMs) { cfg.dosingTimeMs = v; changed = true; }
    gPendingDosingSec = -1;
  }
  if (gPendingCooldownSec >= 0) {
    int32_t v = constrain(gPendingCooldownSec, 30, 3600) * 1000;
    if (v != cfg.cooldownMs) { cfg.cooldownMs = v; changed = true; }
    gPendingCooldownSec = -1;
  }
  if (gPendingAutoMode >= 0) {
    uint8_t v = gPendingAutoMode ? 1 : 0;
    if (v != cfg.autoMode) {
      cfg.autoMode = v;
      changed = true;
      // เปิดอัตโนมัติใหม่ = ให้โอกาสจ่ายสารอีกรอบ
      if (v) { state.doseLimitHit = false; doseTsCount = 0; }
      sendAlert(v ? "เปิดโหมดอัตโนมัติ" : "ปิดโหมดอัตโนมัติ", 0);
    }
    gPendingAutoMode = -1;
  }

  if (changed) {
    saveConfig();
    char msg[128];
    snprintf(msg, sizeof(msg), "ค่าตั้งใหม่: target=%.2f +/-%.2f dose=%ds cooldown=%ds auto=%s",
             cfg.targetPH, cfg.phTolerance, (int)(cfg.dosingTimeMs / 1000),
             (int)(cfg.cooldownMs / 1000), cfg.autoMode ? "ON" : "OFF");
    logInfo(msg);
  }
}

void taskControl(void* pv) {
  esp_task_wdt_add(NULL);

  for (;;) {
    esp_task_wdt_reset();
    applyPendingConfig();

    if (state.emergencyStop) {
      if (isDosingInProgress()) stopAllDosing();
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    if (state.calibMode) {
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    if (isDosingInProgress()) {
      // ── กำลังจ่ายอยู่: หยุดเมื่อหมดเวลา หรือเข้าเป้าแล้วค่านิ่ง ──
      bool timeUp  = isTimeElapsed(state.lastDosingTime, (unsigned long)cfg.dosingTimeMs);
      bool reached = isTargetReached();
      bool stable  = isReadingStable();

      if (timeUp || (reached && stable)) {
        if (xSemaphoreTake(dosingMutex, pdMS_TO_TICKS(1000))) {
          digitalWrite(PH_UP_RELAY,   RELAY_OFF);
          digitalWrite(PH_DOWN_RELAY, RELAY_OFF);
          digitalWrite(MIXER_RELAY,   RELAY_OFF);
          state.dosingInProgress = false;
          state.lastDosingEnd    = millis();
          xSemaphoreGive(dosingMutex);
        }

        logInfo(timeUp ? "จ่ายสารครบเวลา — หยุด" : "เข้าเป้าแล้ว — หยุดจ่าย");
        runMixer(3000);   // กวนให้เข้ากันก่อนเริ่มนับ cooldown
      }
    } else if (cfg.autoMode && cfg.isPhCalibrated) {
      // ── ตรรกะอัตโนมัติ (กฎธรรมดา ไม่ใช้ ML) ──
      if (isTimeElapsed(state.lastDosingEnd, (unsigned long)cfg.cooldownMs)) {
        float ph, temp;
        getSensorReadings(&ph, &temp);

        if (!isnan(ph) && isReadingStable()) {
          if (ph < cfg.targetPH - cfg.phTolerance) {
            startDosing(true, false);
          } else if (ph > cfg.targetPH + cfg.phTolerance) {
            startDosing(false, false);
          }
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ============================================================================
//  Task 3: จอ LCD   (Core 1, prio 1)
// ============================================================================
void taskDisplay(void* pv) {
  esp_task_wdt_add(NULL);

  for (;;) {
    esp_task_wdt_reset();

    if (state.calibMode) {
      // ตอน calibrate ให้ processCalibration() คุมจอเอง
      vTaskDelay(pdMS_TO_TICKS(300));
      continue;
    }

    float ph, temp;
    getSensorReadings(&ph, &temp);

    if (isDosingInProgress()) {
      int remain = max(0, (int)(((unsigned long)cfg.dosingTimeMs -
                    getElapsedTime(state.lastDosingTime)) / 1000));
      updateLine(0, String("DOSE:") + state.lastAction);
      updateLine(1, "LEFT: " + String(remain) + "s");
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }

    String line1 = "pH:";
    line1 += isnan(ph) ? "----" : String(ph, 2);
    line1 += cfg.autoMode ? " [A]" : " [M]";
    if (gFirebaseReady) line1 += "*";
    updateLine(0, line1);

    String line2 = "T:";
    line2 += isnan(temp) ? "--" : String(temp, 1);
    line2 += "C ";

    if (state.emergencyStop) {
      line2 += "ESTOP!";
    } else if (!cfg.isPhCalibrated) {
      line2 += "CALIB?";
    } else if (state.doseLimitHit) {
      line2 += "LIMIT!";
    } else if (!isTimeElapsed(state.lastDosingEnd, (unsigned long)cfg.cooldownMs)) {
      int cd = ((unsigned long)cfg.cooldownMs - getElapsedTime(state.lastDosingEnd)) / 1000;
      line2 += "W:" + String(cd) + "s";
    } else {
      line2 += "T:" + String(cfg.targetPH, 1);
    }
    updateLine(1, line2);

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// ============================================================================
//  Task 4: WiFi + Firebase   (Core 0, prio 2, stack 16 KB สำหรับ TLS)
// ============================================================================

// รับคำสั่งจาก Dashboard แล้วเขียน false กลับ เพื่อให้ปุ่มทำงานแบบกดครั้งเดียว
void processCommands() {
  CmdType cmd;
  while (xQueueReceive(xCmdQueue, &cmd, 0) == pdTRUE) {
    const char* ackPath = NULL;

    switch (cmd) {
      case CMD_PH_UP:
        startDosing(true, true);
        ackPath = "/control/phUp";
        break;

      case CMD_PH_DOWN:
        startDosing(false, true);
        ackPath = "/control/phDown";
        break;

      case CMD_MIXER:
        digitalWrite(MIXER_RELAY, gMixerWant ? RELAY_ON : RELAY_OFF);
        logInfo(gMixerWant ? "เปิดเครื่องกวน (สั่งมือ)" : "ปิดเครื่องกวน (สั่งมือ)");
        break;

      case CMD_START_CALIB:
        if (!state.calibMode) startCalibration();
        ackPath = "/control/startCalib";
        break;

      case CMD_ESTOP_ON:
        if (!state.emergencyStop) {
          state.emergencyStop = true;
          stopAllDosing();
          logWarning("สั่งหยุดฉุกเฉินจากหน้าเว็บ");
          sendAlert("สั่งหยุดฉุกเฉินจากหน้าเว็บ", 2);
        }
        break;

      case CMD_ESTOP_OFF:
        if (state.emergencyStop) {
          state.emergencyStop     = false;
          state.sensorErrorCount  = 0;
          logInfo("ยกเลิกหยุดฉุกเฉิน");
          sendAlert("ยกเลิกหยุดฉุกเฉิน", 0);
        }
        break;

      case CMD_REBOOT:
        logWarning("สั่งรีบูตจากหน้าเว็บ");
        Firebase.RTDB.setBool(&fbdo, "/control/reboot", false);
        vTaskDelay(pdMS_TO_TICKS(1500));
        ESP.restart();
        break;
    }

    if (ackPath && Firebase.ready()) {
      Firebase.RTDB.setBool(&fbdo, ackPath, false);
      fbdo.clear();
    }
    esp_task_wdt_reset();
  }
}

void taskFirebase(void* pv) {
  esp_task_wdt_add(NULL);

  unsigned long lastStatus  = 0;
  unsigned long lastHistory = 0;
  unsigned long lastWiFiTry = 0;
  float         lastSentPH  = -999.0;
  uint32_t      consecFails = 0;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_task_wdt_reset();

    unsigned long now = millis();

    // ── WiFi หลุด: พยายามต่อใหม่ ──
    if (WiFi.status() != WL_CONNECTED) {
      gWiFiConnected = false;
      gFirebaseReady = false;
      if (isTimeElapsed(lastWiFiTry, FB_WIFI_RETRY_MS)) {
        lastWiFiTry = now;
        if (wifiConnect()) syncNTP();   // ต้อง sync เวลาใหม่ทุกครั้งที่ต่อ WiFi ใหม่
      }
      continue;
    }
    gWiFiConnected = true;

    // ── Firebase ยังไม่พร้อม: รอแบบมี backoff ไม่ retry SSL ถี่เกินไป ──
    if (!Firebase.ready()) {
      gFirebaseReady = false;
      vTaskDelay(pdMS_TO_TICKS(FB_NOT_READY_BACKOFF));
      continue;
    }

    if (!gFirebaseReady) {
      gFirebaseReady = true;
      logInfo("Firebase พร้อมใช้งาน");
    }

    // เปิด stream ครั้งแรกหลัง Firebase พร้อม (ต้องเคลียร์ปุ่มค้างก่อนเสมอ)
    if (!gStreamStarted) {
      firebaseSeedControl();
      firebaseStartStream();
    }

    processCommands();

    // ── ส่ง alert ที่ค้างอยู่ในคิว ──
    AlertMsg alert;
    while (xQueueReceive(xAlertQueue, &alert, 0) == pdTRUE) {
      firebaseUploadAlert(alert);
      esp_task_wdt_reset();
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    // ── ส่งสถานะ: ถี่ขึ้นตอนกำลังทำงาน ห่างขึ้นตอนอยู่เฉย ──
    uint32_t interval = (isDosingInProgress() || state.calibMode)
                        ? FB_STATUS_ACTIVE_MS : FB_STATUS_IDLE_MS;

    if (isTimeElapsed(lastStatus, interval)) {
      lastStatus = now;
      if (firebaseUploadStatus()) {
        consecFails = 0;
      } else {
        consecFails++;
        if (consecFails >= FB_MAX_CONSEC_FAIL) {
          Serial.printf("[FB] ส่งพลาดติดกัน %lu ครั้ง: %s\n",
                        (unsigned long)consecFails, fbdo.errorReason().c_str());
          fbdo.clear();
          consecFails = 0;
        }
      }
    }

    // ── ค่าล่าสุด + กราฟย้อนหลัง ──
    float ph, temp;
    getSensorReadings(&ph, &temp);

    if (!isnan(ph) && fabs(ph - lastSentPH) > 0.02) {
      if (firebaseUploadReading("/lastReading", false, ph, temp)) lastSentPH = ph;
    }

    if (!isnan(ph) && isTimeElapsed(lastHistory, FB_HISTORY_MS)) {
      lastHistory = now;
      firebaseUploadReading("/history", true, ph, temp);
    }
  }
}

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  vTaskDelay(pdMS_TO_TICKS(500));
  Serial.println("\n\n===========================================================");
  Serial.println("  pH Control Pro v" FW_VERSION "  (ESP32 + Firebase RTDB)");
  Serial.println("===========================================================");

  memset(&state, 0, sizeof(SystemState));
  state.currentPH   = NAN;
  state.currentTemp = NAN;
  setLastAction("Idle");

  // ── Watchdog: core 3.x init TWDT มาให้แล้ว ต้องใช้ reconfigure แทน init ──
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms     = WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = 0,
    .trigger_panic  = true
  };
  if (esp_task_wdt_init(&wdt_config) == ESP_ERR_INVALID_STATE) {
    esp_task_wdt_reconfigure(&wdt_config);
  }
  esp_task_wdt_add(NULL);

  // ── Mutex + Queue (ต้องสร้างก่อนแตะฮาร์ดแวร์) ──
  lcdMutex    = xSemaphoreCreateMutex();
  sensorMutex = xSemaphoreCreateMutex();
  dosingMutex = xSemaphoreCreateMutex();
  calibMutex  = xSemaphoreCreateMutex();
  xCmdQueue   = xQueueCreate(10, sizeof(CmdType));
  xAlertQueue = xQueueCreate(10, sizeof(AlertMsg));

  if (!lcdMutex || !sensorMutex || !dosingMutex || !calibMutex || !xCmdQueue || !xAlertQueue) {
    Serial.println("FATAL: สร้าง mutex/queue ไม่สำเร็จ");
    while (1) delay(1000);
  }

  // ── รีเลย์: ปิดทั้งหมดก่อนเป็นอันดับแรก ──
  pinMode(PH_UP_RELAY,   OUTPUT);
  pinMode(PH_DOWN_RELAY, OUTPUT);
  pinMode(MIXER_RELAY,   OUTPUT);
  digitalWrite(PH_UP_RELAY,   RELAY_OFF);
  digitalWrite(PH_DOWN_RELAY, RELAY_OFF);
  digitalWrite(MIXER_RELAY,   RELAY_OFF);
  pinMode(CALIB_BUTTON, INPUT_PULLUP);

  analogSetAttenuation(ADC_11db);   // ให้วัดได้ถึง ~3.3V

  tempSensor.begin();

  EEPROM.begin(512);

  if (xSemaphoreTake(lcdMutex, pdMS_TO_TICKS(1000))) {
    lcd.init();
    lcd.backlight();
    lcd.clear();
    xSemaphoreGive(lcdMutex);
  }
  showMessage("pH Control Pro", "v" FW_VERSION);
  vTaskDelay(pdMS_TO_TICKS(1500));

  loadConfig();

  // ── เครือข่าย: WiFi -> NTP -> Firebase (ลำดับนี้ห้ามสลับ) ──
  showMessage("CONNECTING", WIFI_SSID);
  if (wifiConnect()) {
    showMessage("WiFi OK", WiFi.localIP().toString());
    vTaskDelay(pdMS_TO_TICKS(1000));

    syncNTP();
    firebaseInit();

    showMessage("FIREBASE", gSignUpOK ? "Sign-up OK" : "Sign-up FAIL");
  } else {
    showMessage("WiFi FAILED", "Offline mode");
  }
  vTaskDelay(pdMS_TO_TICKS(1500));

  // ── สร้าง Task ──
  xTaskCreatePinnedToCore(taskControl,  "Control",   8192, NULL, 3, &hTaskControl,  1);
  xTaskCreatePinnedToCore(taskSensor,   "Sensor",    8192, NULL, 2, &hTaskSensor,   1);
  xTaskCreatePinnedToCore(taskDisplay,  "Display",   4096, NULL, 1, &hTaskDisplay,  1);
  xTaskCreatePinnedToCore(taskFirebase, "Firebase", 16384, NULL, 2, &hTaskFirebase, 0);

  Serial.println("\n--- สรุปสถานะระบบ ---");
  Serial.printf("  Calibrate pH : %s (R2 %.1f%%, ครั้งที่ %d)\n",
                cfg.isPhCalibrated ? "แล้ว" : "ยัง", cfg.phR2, (int)cfg.calibrationCount);
  Serial.printf("  เป้าหมาย pH  : %.2f +/- %.2f\n", cfg.targetPH, cfg.phTolerance);
  Serial.printf("  เวลาจ่ายสาร  : %d วินาที\n", (int)(cfg.dosingTimeMs / 1000));
  Serial.printf("  พักระหว่างจ่าย: %d วินาที\n", (int)(cfg.cooldownMs / 1000));
  Serial.printf("  โหมดอัตโนมัติ: %s\n", cfg.autoMode ? "เปิด" : "ปิด");
  Serial.printf("  Free heap    : %d bytes\n", ESP.getFreeHeap());
  Serial.println("\n  กดปุ่ม BOOT ค้าง 3 วินาที เพื่อเริ่ม calibrate");
  Serial.println("===========================================================\n");

  showMessage("SYSTEM READY", cfg.isPhCalibrated ? "OK" : "Need calib!");

  esp_task_wdt_delete(NULL);   // ปล่อย loop task ออกจาก WDT ก่อนจะลบทิ้ง
}

// ============================================================================
//  LOOP — ไม่ใช้ ทุกอย่างอยู่ใน FreeRTOS task
// ============================================================================
void loop() {
  vTaskDelete(NULL);
}
