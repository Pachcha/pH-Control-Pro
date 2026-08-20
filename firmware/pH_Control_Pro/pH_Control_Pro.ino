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
#include <esp_heap_caps.h>
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
#define MIXER_RELAY        14     // ย้ายมาจากขา 16 ที่ไม่ตอบสนอง (ขา 14 เดิมเป็นปุ๋ย A ที่ตัดทิ้งแล้ว)
                                  // ขา 14 มีพัลส์สั้นๆ ออกตอนบูต เครื่องกวนจะกระตุกหนึ่งครั้งทุกครั้งที่รีเซ็ต
                                  // ถ้าไม่อยากให้กระตุกเลย ย้ายไปขา 25 ซึ่งไม่มีพฤติกรรมนี้

// ═══ ขั้วสัญญาณของโมดูลขับโหลด — ตั้งทีละช่องได้ ═══
//  ตั้งค่าปัจจุบัน: โมดูล MOSFET 4 ช่อง (active HIGH → ใส่ 0)
//  ยืนยันจากโค้ดตัวอย่างของโมดูล: digitalWrite(pin, HIGH) = เปิด, LOW = ปิด
//
//  รีเลย์ทั่วไป   : active LOW  → ใส่ 1
//  โมดูล MOSFET  : active HIGH → ใส่ 0
//
//  ★★ ตั้งผิด = ตอนบูตโค้ดสั่ง "ปิด" แต่โมดูลเข้าใจว่า "เปิด"
//     ปั๊มจ่ายสารเคมีจะทำงานค้างทันทีที่เสียบไฟและหยุดไม่ได้จนกว่าจะดึงปลั๊ก
//     ★ ถ้ายังต่อรีเลย์อยู่ ห้ามแฟลชเวอร์ชันนี้ ให้เปลี่ยนกลับเป็น 1 ก่อน
//
//  ทดสอบทุกครั้งหลังเปลี่ยนโมดูล: ถอดสายยางออกจากขวดสารเคมี แล้วดู LED
//  ประจำช่องตอนบูต ต้องดับทั้งหมด (ดูขั้นตอนเต็มใน docs/wiring.md)
#define PH_UP_ACTIVE_LOW    0
#define PH_DOWN_ACTIVE_LOW  0
#define MIXER_ACTIVE_LOW    0

#define ON_LEVEL(activeLow)   ((activeLow) ? LOW  : HIGH)
#define OFF_LEVEL(activeLow)  ((activeLow) ? HIGH : LOW)

#define PH_UP_ON     ON_LEVEL(PH_UP_ACTIVE_LOW)
#define PH_UP_OFF    OFF_LEVEL(PH_UP_ACTIVE_LOW)
#define PH_DOWN_ON   ON_LEVEL(PH_DOWN_ACTIVE_LOW)
#define PH_DOWN_OFF  OFF_LEVEL(PH_DOWN_ACTIVE_LOW)
#define MIXER_ON     ON_LEVEL(MIXER_ACTIVE_LOW)
#define MIXER_OFF    OFF_LEVEL(MIXER_ACTIVE_LOW)

// ============================================================================
//  ค่าคงที่ระบบ
// ============================================================================
#define FW_VERSION         "1.0.0"

// ★ ค่า pH ของน้ำยา buffer ที่ใช้ — ต้องตรงกับขวดที่คุณมีจริง ไม่งั้นเส้นจะบิด
//   ชุดที่ใช้อยู่: pH 4.00 / 7.00 / 10.01 (แบบที่ขายทั่วไป ระบุที่ 25°C)
//   ถ้าเปลี่ยนไปใช้ชุด NIST/DIN (4.01 / 6.86 / 9.18) ให้แก้ 3 บรรทัดนี้
//   แล้วอย่าลืมแก้ข้อความบนจอ LCD กับใน docs/index.html (ตัวแปร BUFFERS) ให้ตรงกันด้วย
#define CALIB_PH_1         4.00
#define CALIB_PH_2         7.00
#define CALIB_PH_3         10.01
#define CALIB_ABORT_MS     300000UL  // ไม่ได้เก็บค่าภายใน 5 นาที = ยกเลิก (ไม่เก็บค่าขยะ)

// ── เก็บค่าอัตโนมัติเมื่อนิ่ง ──
// ต้องผ่านทั้ง 2 เงื่อนไข ไม่ใช่แค่ "นิ่ง ณ ตอนนี้"
//   1) อยู่ในขั้นนี้มานานพอ — เผื่อเวลาล้างหัววัด ซับแห้ง แล้วจุ่มน้ำยาใหม่
//      ถ้าไม่มีข้อนี้ ตอนหัววัดยังแช่น้ำล้างอยู่แล้วบังเอิญนิ่ง จะเก็บค่าผิดทันที
//   2) นิ่งต่อเนื่องกันจริง ไม่ใช่นิ่งแวบเดียวแล้วไหลต่อ
#define CALIB_MIN_DWELL_MS    30000UL
#define CALIB_STABLE_HOLD_MS  10000UL
// ── ตัวกรองสัญญาณ (EMA แบบอิงเวลาจริง) ──
// pH ในถังเปลี่ยนช้ามาก การเฉลี่ยย้อนหลังสิบกว่าวินาทีจึงไม่เสียข้อมูลอะไรเลย
// แต่ตัดสัญญาณรบกวนแบบสุ่มได้เยอะ (วัดจริงได้ส่วนเบี่ยงเบน 22 mV)
//
// ★ จุดสำคัญ: EMA กรอง "noise แบบสุ่ม" ออก แต่ยัง "คงแนวโน้ม" ไว้
//   หัววัดที่ยังไหลเข้าหาค่าจริงจะยังเห็นเป็นแนวโน้มอยู่ เราจึงตัดสิน
//   ความนิ่งจากสัญญาณที่กรองแล้วได้ = "นิ่ง" แปลว่าหยุดไหลจริง ไม่ใช่แค่ noise น้อย
#define PH_FILTER_TAU_MS   15000     // ค่าคงที่เวลาของตัวกรอง

// ── การเก็บตัวอย่าง ADC ──
#define MAINS_PERIOD_MS    20        // ไฟบ้านไทย 50 Hz = คาบละ 20 ms
#define ADC_SAMPLE_GAP_MS  2         // ระยะห่างเป้าหมายระหว่างตัวอย่าง
#define PH_SAMPLES_NORMAL  50        // ตอนทำงานปกติ (~100 ms = 5 คาบ)
#define PH_SAMPLES_CALIB   50        // ตอน calibrate ต้องแม่นที่สุด
#define PH_STABLE_WINDOW   30        // จำนวนตัวอย่างที่ใช้ตัดสินว่านิ่ง
#define PH_STABLE_MV       0.0015    // เกณฑ์นิ่ง 1.5 mV — เข้มได้เพราะวัดบนสัญญาณที่กรองแล้ว
#define CALIB_MIN_R2       90.0      // R² ต่ำกว่านี้ถือว่า calibrate ไม่ผ่าน

// ★ R² อย่างเดียวไม่พอ! R² วัดแค่ว่า 3 จุดเรียงเป็นเส้นตรงไหม ไม่ได้วัดว่าหัววัดไวพอไหม
//   หัววัดที่เสีย/สายหลวม จะให้แรงดันเดินสุ่มทีละนิด ถ้าบังเอิญไต่ขึ้นตามลำดับ
//   R² จะสูงถึง 99% ทั้งที่เป็นแค่ noise แล้วได้ slope ชันมากจนสัญญาณรบกวนไม่กี่ mV
//   กลายเป็น pH แกว่งเป็นหน่วย → ระบบสั่งจ่ายสารเคมีมั่ว
//   ทฤษฎี Nernst ที่ 25°C ให้ 59 mV/pH แม้ไม่มีวงจรขยาย = 355 mV ในช่วง pH 4.00-10.01
//   ตั้งเกณฑ์ขั้นต่ำไว้ต่ำกว่านั้นมาก เพื่อจับเฉพาะกรณีที่พังจริงๆ
#define CALIB_MIN_SPAN_V   0.20      // ช่วงแรงดันขั้นต่ำระหว่างจุดแรกกับจุดสุดท้าย
#define NERNST_MV_PER_PH   59.16     // ทฤษฎี Nernst ที่ 25 องศา
#define CALIB_MAX_ASYMMETRY 0.30     // ความชันสองฝั่งต่างกันได้ไม่เกิน 30%
#define CALIB_PH_RANGE     (CALIB_PH_3 - CALIB_PH_1)

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
#define EEPROM_VERSION     2   // v2: เพิ่ม calibTempC + เปลี่ยนไปใช้ piecewise

// ── จังหวะส่งข้อมูลขึ้น Firebase ──
#define FB_STATUS_ACTIVE_MS    5000
#define FB_STATUS_CALIB_MS     4000      // ตอน calibrate ต้องเห็นแรงดันสด แต่ถี่กว่านี้ SSL เอาไม่อยู่

// mbedTLS ต้องการบล็อกหน่วยความจำ "ต่อเนื่อง" ก้อนใหญ่ตอน handshake
// ยอด free heap รวมเยอะไม่ได้แปลว่าจองได้ ถ้าหน่วยความจำแตกเป็นเสี่ยงๆ
// ต่ำกว่านี้ให้ข้ามการส่งรอบนั้นไป ดีกว่าปล่อยให้ SSL พังแล้วหลุดทั้งเส้น
#define FB_MIN_FREE_BLOCK      24000

// ═══ ระบบเฝ้าระวังเพื่อให้ทำงานต่อเนื่อง 24 ชั่วโมง ═══
// หลักการ: อย่าปล่อยให้ระบบอยู่ในสภาพ "ไม่ตาย แต่ก็ไม่ทำงาน" นานๆ
// ถ้ากู้เองไม่ได้ภายในเวลาที่กำหนด ให้รีบูตดีกว่าค้างเงียบๆ ไปทั้งคืน
#define WIFI_DEAD_REBOOT_MS   900000UL   // WiFi ต่อไม่ติดนานเกิน 15 นาที = รีบูต
#define FB_REINIT_AFTER_MS    600000UL   // Firebase ไม่พร้อมเกิน 10 นาที = init ใหม่
#define FB_DEAD_REBOOT_MS    1200000UL   // init ใหม่แล้วยังไม่พร้อมอีกจน 20 นาที = รีบูต
#define LOWMEM_REBOOT_MS      300000UL   // หน่วยความจำต่อเนื่องต่ำค้างเกิน 5 นาที = รีบูต
#define STREAM_DEAD_MS        120000UL   // stream หลุดเกิน 2 นาที = เปิดใหม่
#define TASK_BEAT_TIMEOUT_MS   60000UL   // task ไหนไม่เต้นเกิน 1 นาที = รีบูต
#define DAILY_REBOOT_MS     86400000UL   // รีบูตกันเหนียวทุก 24 ชม. (ใส่ 0 = ปิด)
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

  float    phSlope;             // จาก linear fit เส้นเดียว — เก็บไว้ใช้เป็นตัววัดคุณภาพเท่านั้น
  float    phIntercept;         // ไม่ได้ใช้คำนวณ pH แล้ว (ใช้ piecewise แทน)
  float    phCalibVoltage[3];   // โวลต์ที่วัดได้ตอน calibrate แต่ละจุด
  float    phR2;                // ความแม่นของเส้น calibration (%)
  float    calibTempC;          // อุณหภูมิขณะ calibrate — ใช้ชดเชยตามสมการ Nernst
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
  CMD_PH_UP, CMD_PH_DOWN, CMD_MIXER, CMD_START_CALIB, CMD_CALIB_CAPTURE,
  CMD_ESTOP_ON, CMD_ESTOP_OFF, CMD_REBOOT
};

#define ALERT_MSG_LEN 224   // ภาษาไทยกินตัวละ 3 ไบต์ ข้อความอธิบายยาวๆ ต้องเผื่อมาก
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

// ── ชีพจรของแต่ละ task ──
// WDT จับได้เฉพาะ task ที่ "ค้าง" แต่จับไม่ได้ถ้า task "ตายหายไปเลย"
// (เช่นโดน stack overflow ฆ่า) เพราะมันถูกถอดออกจาก WDT ไปด้วย
// ชีพจรนี้จึงเป็นตาข่ายชั้นที่สอง
#define TASK_COUNT 4
enum { T_SENSOR = 0, T_CONTROL, T_DISPLAY, T_FIREBASE };
static const char* kTaskName[TASK_COUNT] = {"Sensor", "Control", "Display", "Firebase"};
volatile unsigned long gTaskBeat[TASK_COUNT] = {0};

// เก็บข้ามการรีบูต (อยู่รอดถ้าเป็น soft reset แต่หายเมื่อไฟดับ)
RTC_NOINIT_ATTR uint32_t gRtcMagic;
RTC_NOINIT_ATTR uint32_t gRebootCount;
RTC_NOINIT_ATTR char     gLastRebootReason[128];
#define RTC_MAGIC 0x50484252UL   // "PHBR"

char gResetReason[64] = "unknown";
volatile uint32_t gFbSendCount    = 0;
volatile uint32_t gFbFailCount    = 0;

// ค่าตั้งที่รับมาจาก Dashboard รอนำไปใช้ (stream callback ห้ามบล็อกยาว
// จึงแค่พักค่าไว้ตรงนี้ แล้วให้ taskControl เอาไปเขียนลง cfg + EEPROM)
volatile uint32_t gSensorSeq      = 0;    // เพิ่มขึ้น 1 ทุกครั้งที่อ่านเซนเซอร์รอบใหม่

// แรงดันดิบจากขา pH — เผยแพร่ขึ้น /status เสมอ ใช้ดูว่าต่อสายติดไหม
// และใช้ดูว่าค่านิ่งพอจะกดเก็บตอน calibrate หรือยัง
volatile float gPhVoltage         = NAN;   // ผ่านตัวกรองแล้ว — ใช้คำนวณ pH และเก็บตอน calibrate
volatile float gPhVoltageRaw      = NAN;   // ดิบๆ ก่อนกรอง — ไว้ดูว่าสัญญาณรบกวนแรงแค่ไหน
volatile float gPhNoiseMv         = 0;     // ขนาดสัญญาณรบกวนที่ตัวกรองกำจัดออกไป (mV)
volatile bool  gPhStable          = false;
volatile bool  gCalibCaptureReq   = false;   // คำสั่งเก็บค่าจากหน้าเว็บ
volatile int   gCalibDwellSec     = 0;       // อยู่ในขั้นนี้มากี่วินาทีแล้ว
volatile int   gCalibStableSec    = 0;       // นิ่งต่อเนื่องมากี่วินาทีแล้ว

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
unsigned long calibStartTime   = 0;
unsigned long calibStableSince = 0;   // เริ่มนิ่งตั้งแต่เมื่อไหร่ (0 = ยังไม่นิ่ง)
int           calibBtnPrev     = LOW; // สถานะปุ่มรอบก่อน ใช้จับ "ขอบขาลง"

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
//  ระบบเฝ้าระวัง
// ============================================================================

// คัดลอกสตริงโดยไม่ผ่ากลางตัวอักษร UTF-8
// ภาษาไทยกินตัวละ 3 ไบต์ ตัดผิดที่จะได้ไบต์เสียแล้วขึ้นเป็น "?" บนหน้าเว็บ
// ไบต์ต่อเนื่องของ UTF-8 มีรูปแบบ 10xxxxxx ให้ถอยจนพ้นมัน
void copyUtf8(char* dst, size_t cap, const char* src) {
  size_t n = strlen(src);
  if (n >= cap) {
    n = cap - 1;
    while (n > 0 && ((uint8_t)src[n] & 0xC0) == 0x80) n--;
  }
  memcpy(dst, src, n);
  dst[n] = '\0';
}

void beat(int taskIndex) {
  if (taskIndex >= 0 && taskIndex < TASK_COUNT) gTaskBeat[taskIndex] = millis();
}

// แปลงสาเหตุการรีเซ็ตเป็นข้อความอ่านออก — สำคัญมากกับระบบที่รัน 24 ชม.
// เพราะถ้ารีบูตเองตอนตีสาม เราต้องรู้ว่าเพราะอะไร
const char* resetReasonText(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "เปิดไฟใหม่";
    case ESP_RST_SW:       return "สั่งรีบูตจากโค้ด";
    case ESP_RST_PANIC:    return "โค้ด crash (panic)";
    case ESP_RST_INT_WDT:  return "watchdog ระดับ interrupt";
    case ESP_RST_TASK_WDT: return "watchdog จับ task ค้าง";
    case ESP_RST_WDT:      return "watchdog อื่น";
    case ESP_RST_BROWNOUT: return "ไฟตก (brownout)";
    case ESP_RST_DEEPSLEEP:return "ตื่นจาก deep sleep";
    case ESP_RST_EXT:      return "รีเซ็ตจากขา EN";
    default:               return "ไม่ทราบสาเหตุ";
  }
}

// รีบูตแบบปลอดภัย — ปิดปั๊มก่อนเสมอ และพยายามบันทึกสาเหตุขึ้น Firebase ให้ทัน
void safeReboot(const char* reason) {
  stopAllDosing();
  logWarning(reason);

  gRtcMagic    = RTC_MAGIC;
  gRebootCount = gRebootCount + 1;
  copyUtf8(gLastRebootReason, sizeof(gLastRebootReason), reason);

  // ส่งตรงไม่ผ่านคิว เพราะกำลังจะรีบูต คิวไม่มีใครมาไล่ส่งแล้ว
  if (Firebase.ready()) {
    gJson.clear();
    gJson.set("message",       String("รีบูตอัตโนมัติ: ") + reason);
    gJson.set("priority",      2);
    gJson.set("timestamp/.sv", "timestamp");
    Firebase.RTDB.pushJSON(&fbdo, "/alerts", &gJson);
    gJson.clear();
  }

  vTaskDelay(pdMS_TO_TICKS(1500));
  ESP.restart();
}

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
// อ่านแรงดันจากขา pH
//
// ★ ใช้ analogReadMilliVolts() ไม่ใช่ analogRead()
//   ADC ของ ESP32 ไม่เป็นเชิงเส้น (โค้งเป็นรูปตัว S) และคลาดเคลื่อนได้ ±6%
//   ระหว่างชิปแต่ละตัว สูตรเดิม raw*(3.3/4095) สมมติว่าเป็นเชิงเส้นและ
//   แรงดันอ้างอิงเท่ากับ 3.3V เป๊ะ ซึ่งผิดทั้งคู่
//   analogReadMilliVolts() อ่านค่าสอบเทียบจาก eFuse ของชิปมาชดเชยความโค้งให้
//   สำคัญมากเพราะเราฟิตเส้นตรงกับแรงดัน ความโค้งของ ADC จะกลายเป็น
//   ความคลาดเคลื่อนของ pH ที่หนักสุดตรงกลางช่วง คือย่าน pH 7 พอดี
//
// ★ เก็บตัวอย่างให้ครอบคลุมจำนวนเต็มคาบของไฟบ้าน (50 Hz = 20 ms)
//   การเฉลี่ยครบคาบทำให้ hum จากไฟบ้านหักล้างกันเองเกือบหมด
//   แล้วใช้ trimmed mean แทน median — ได้ทั้งการตัด outlier และการลด
//   สัญญาณรบกวนตาม sqrt(N) (median ลด noise ได้แย่กว่า mean ราว 25%)
float readPhVoltage(int samples) {
  const int MAXS = 64;
  if (samples > MAXS) samples = MAXS;
  if (samples < 4)    samples = 4;

  // กระจายตัวอย่างให้เต็มจำนวนเต็มคาบของไฟบ้าน
  int cycles    = (samples * ADC_SAMPLE_GAP_MS + MAINS_PERIOD_MS - 1) / MAINS_PERIOD_MS;
  if (cycles < 1) cycles = 1;
  int windowMs  = cycles * MAINS_PERIOD_MS;
  int gapMs     = windowMs / samples;
  if (gapMs < 1) gapMs = 1;

  float v[MAXS];
  int   n = 0;
  for (int i = 0; i < samples; i++) {
    uint32_t mv = analogReadMilliVolts(PH_PIN);
    if (mv > 10 && mv < 3250) v[n++] = mv / 1000.0f;
    vTaskDelay(pdMS_TO_TICKS(gapMs));
  }
  if (n < samples / 2) return NAN;

  // ตัดค่าสุดขอบ 20% บนล่าง แล้วเฉลี่ยส่วนที่เหลือ
  std::sort(v, v + n);
  int lo = n / 5, hi = n - lo;
  if (hi <= lo) { lo = 0; hi = n; }

  float sum = 0;
  for (int i = lo; i < hi; i++) sum += v[i];
  return sum / (hi - lo);
}

// กรองสัญญาณ + ติดตามว่านิ่งแล้วหรือยัง
// เรียกทุกครั้งที่อ่านแรงดันดิบได้ ทั้งตอนทำงานปกติและตอน calibrate
void trackPhVoltage(float raw) {
  static float         win[PH_STABLE_WINDOW];   // หน้าต่างของค่าที่กรองแล้ว
  static int           idx = 0, filled = 0;
  static float         rawWin[PH_STABLE_WINDOW];
  static unsigned long lastMs = 0;

  gPhVoltageRaw = raw;

  if (isnan(raw)) {
    filled = 0; lastMs = 0;
    gPhStable = false; gPhNoiseMv = 0;
    return;
  }

  // ── EMA อิงเวลาจริง ──
  // คำนวณ alpha จาก dt ที่ผ่านไปจริง ทำให้ได้ค่าคงที่เวลาเท่ากันเสมอ
  // ไม่ว่าจะถูกเรียกถี่ (ตอน calibrate ~200 ms) หรือห่าง (ตอนปกติ ~1 วิ)
  unsigned long now = millis();
  unsigned long dt  = lastMs ? (unsigned long)(now - lastMs) : PH_FILTER_TAU_MS;
  lastMs = now;
  if (dt > PH_FILTER_TAU_MS) dt = PH_FILTER_TAU_MS;   // กันกระโดดหลังหยุดไปนาน

  if (isnan(gPhVoltage)) {
    gPhVoltage = raw;                                  // ค่าแรก เริ่มจากตรงนั้นเลย
  } else {
    float alpha = (float)dt / (float)(PH_FILTER_TAU_MS + dt);
    gPhVoltage = gPhVoltage + alpha * (raw - gPhVoltage);
  }

  // ── หน้าต่างสำหรับตัดสินความนิ่ง (ใช้ค่าที่กรองแล้ว) ──
  win[idx]    = gPhVoltage;
  rawWin[idx] = raw;
  idx = (idx + 1) % PH_STABLE_WINDOW;
  if (filled < PH_STABLE_WINDOW) filled++;

  if (filled < PH_STABLE_WINDOW) { gPhStable = false; return; }

  float mean = 0, rawMean = 0;
  for (int i = 0; i < PH_STABLE_WINDOW; i++) { mean += win[i]; rawMean += rawWin[i]; }
  mean    /= PH_STABLE_WINDOW;
  rawMean /= PH_STABLE_WINDOW;

  float var = 0, rawVar = 0;
  for (int i = 0; i < PH_STABLE_WINDOW; i++) {
    var    += (win[i]    - mean)    * (win[i]    - mean);
    rawVar += (rawWin[i] - rawMean) * (rawWin[i] - rawMean);
  }

  gPhNoiseMv = sqrt(rawVar / PH_STABLE_WINDOW) * 1000.0;   // noise ก่อนกรอง ไว้โชว์ให้ดู
  gPhStable  = sqrt(var / PH_STABLE_WINDOW) < PH_STABLE_MV;
}

// แปลงแรงดันเป็น pH แบบแบ่งช่วง (piecewise) พร้อมชดเชยอุณหภูมิตามสมการ Nernst
//
// ★ ทำไมต้องแบ่งช่วง: หัววัด pH จริงมีความชันฝั่งกรดกับฝั่งด่างไม่เท่ากัน
//   ต่างกันได้ 2-5% การฟิตเส้นตรงเส้นเดียวจึงไม่ผ่านจุด calibration สักจุด
//   เกิดความคลาดเคลื่อนกลางช่วงราว 0.05-0.1 pH
//   แบ่งสองช่วงแล้วเส้นจะผ่านทั้ง 3 จุดพอดีเป๊ะ (เครื่องวัดมืออาชีพทำแบบนี้)
//
// ★ จุดกลางเราใช้ buffer pH 7.00 ซึ่งเป็นจุด isopotential ของหัววัดพอดี
//   phCalibVoltage[1] จึงเป็น V_iso โดยตรง ไม่ต้องฟิตหา
//
// ★ ชดเชยอุณหภูมิ: ความชันแปรตามอุณหภูมิสัมบูรณ์
//   S(T) = S(T_cal) x (273.15 + T) / (273.15 + T_cal)
float voltageToPH(float v, float tempC) {
  if (isnan(v) || !cfg.isPhCalibrated) return NAN;

  const float vIso  = cfg.phCalibVoltage[1];
  float       sAcid = (cfg.phCalibVoltage[0] - vIso) / (CALIB_PH_2 - CALIB_PH_1);  // โวลต์ต่อ pH
  float       sBase = (vIso - cfg.phCalibVoltage[2]) / (CALIB_PH_3 - CALIB_PH_2);

  if (fabs(sAcid) < 1e-6 || fabs(sBase) < 1e-6) return NAN;

  // เลือกช่วง: ลองด้วยฝั่งกรดก่อน ถ้าได้ผลเกิน pH 7 แปลว่าจริงๆ อยู่ฝั่งด่าง
  // เขียนแบบนี้ทนแม้หัววัดต่อกลับขั้ว (แรงดันเพิ่มขึ้นตาม pH)
  float s = ((vIso - v) / sAcid > 0.0f) ? sBase : sAcid;

  // ชดเชยอุณหภูมิ — อ่านอุณหภูมิไม่ได้ก็ใช้ค่าตอน calibrate = ไม่ชดเชย ไม่พัง
  float tCal = isnan(cfg.calibTempC) ? 25.0f : cfg.calibTempC;
  float tNow = isnan(tempC)          ? tCal  : tempC;
  s *= (273.15f + tNow) / (273.15f + tCal);

  return CALIB_PH_2 + (vIso - v) / s;
}

float readPH(float tempC) {
  for (int retry = 0; retry < MAX_SENSOR_RETRIES; retry++) {
    float v = readPhVoltage(PH_SAMPLES_NORMAL);
    trackPhVoltage(v);

    if (!isnan(v)) {
      // ยังไม่ calibrate = มีแรงดันให้ดูได้ แต่แปลงเป็น pH ยังไม่ได้
      if (!cfg.isPhCalibrated) return NAN;

      // ใช้ค่าที่ผ่านตัวกรองแล้ว ไม่ใช่ค่าดิบ — pH ในถังเปลี่ยนช้า
      // การเฉลี่ยย้อนหลังไม่เสียข้อมูล แต่ตัด noise ได้เยอะ
      float phValue = voltageToPH(gPhVoltage, tempC);
      if (phValue >= 0.0 && phValue <= 14.0) return phValue;
    }

    vTaskDelay(pdMS_TO_TICKS(300));
  }

  logError("อ่านค่า pH ไม่สำเร็จ");
  return NAN;
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
    digitalWrite(PH_UP_RELAY,   PH_UP_OFF);
    digitalWrite(PH_DOWN_RELAY, PH_DOWN_OFF);
    digitalWrite(MIXER_RELAY,   MIXER_OFF);
    state.dosingInProgress = false;
    xSemaphoreGive(dosingMutex);
  }
}

void runMixer(unsigned int durationMs) {
  digitalWrite(MIXER_RELAY, MIXER_ON);
  vTaskDelay(pdMS_TO_TICKS(durationMs));
  digitalWrite(MIXER_RELAY, MIXER_OFF);
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
    digitalWrite(isUp ? PH_UP_RELAY : PH_DOWN_RELAY, isUp ? PH_UP_ON : PH_DOWN_ON);
    digitalWrite(MIXER_RELAY, MIXER_ON);
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
  cfg.calibTempC       = 25.0;
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
    char msg[ALERT_MSG_LEN];
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
//  Calibration — 3 จุด (pH 4.00 / 7.00 / 10.01) แล้วหาเส้นตรงด้วย linear regression
// ============================================================================

// รายงานว่า calibrate ไม่ผ่านพร้อมเหตุผล แล้วล้างสถานะให้ระบบไม่เอาไปใช้
void calibFailed(const char* reason) {
  cfg.phR2           = 0.0;
  cfg.isPhCalibrated = 0;

  char msg[ALERT_MSG_LEN];
  snprintf(msg, sizeof(msg), "Calibrate ไม่ผ่าน: %s", reason);
  logError(msg);
  sendAlert(msg, 2);

  showMessage("CALIB FAILED", reason);
  vTaskDelay(pdMS_TO_TICKS(4000));
}

void calculatePHCalibration() {
  float knownPH[3] = {CALIB_PH_1, CALIB_PH_2, CALIB_PH_3};

  float v1 = cfg.phCalibVoltage[0];
  float v2 = cfg.phCalibVoltage[1];
  float v3 = cfg.phCalibVoltage[2];

  // ── ด่านที่ 1: หัววัดไวพอไหม ──
  float spanV   = fabs(v3 - v1);
  float mvPerPh = (spanV * 1000.0) / CALIB_PH_RANGE;

  char detail[ALERT_MSG_LEN];
  snprintf(detail, sizeof(detail), "V1=%.4f V2=%.4f V3=%.4f span=%.0fmV (%.1f mV/pH)",
           v1, v2, v3, spanV * 1000.0, mvPerPh);
  logInfo(detail);
  sendAlert(detail, 0);

  if (spanV < CALIB_MIN_SPAN_V) {
    char r[64];
    snprintf(r, sizeof(r), "span %.0fmV noisy", spanV * 1000.0);
    calibFailed(r);
    sendAlert("หัววัดแทบไม่ตอบสนองต่อ pH — ตรวจข้อต่อ BNC, สภาพหัววัด, ไฟเลี้ยงโมดูล", 2);
    return;
  }

  // ── ด่านที่ 2: แรงดันต้องไปทางเดียวกันทั้ง 3 จุด ──
  bool monotonic = ((v2 > v1 && v3 > v2) || (v2 < v1 && v3 < v2));
  if (!monotonic) {
    calibFailed("not monotonic");
    sendAlert("แรงดันไม่ไล่ไปทางเดียวกันทั้ง 3 จุด — น่าจะเก็บค่าตอนยังไม่นิ่ง หรือไม่ได้ล้างหัววัด", 2);
    return;
  }

  // ── ด่านที่ 3: ความชันสองฝั่งต้องไม่ต่างกันมากเกินไป ──
  // หัววัดที่ดีมีความชันฝั่งกรดกับด่างต่างกันไม่กี่เปอร์เซ็นต์
  // ถ้าต่างกันมาก แปลว่าหัววัดเสื่อม หรือมีจุดใดจุดหนึ่งเก็บค่าเพี้ยน
  float sAcidMv = fabs(v1 - v2) * 1000.0 / (CALIB_PH_2 - CALIB_PH_1);
  float sBaseMv = fabs(v2 - v3) * 1000.0 / (CALIB_PH_3 - CALIB_PH_2);
  float bigger  = max(sAcidMv, sBaseMv);
  float asym    = (bigger > 0) ? fabs(sAcidMv - sBaseMv) / bigger : 1.0;

  char detail2[ALERT_MSG_LEN];
  snprintf(detail2, sizeof(detail2), "ความชัน: ฝั่งกรด %.1f mV/pH  ฝั่งด่าง %.1f mV/pH  ต่างกัน %.0f%%",
           sAcidMv, sBaseMv, asym * 100.0);
  logInfo(detail2);
  sendAlert(detail2, 0);

  if (asym > CALIB_MAX_ASYMMETRY) {
    char r[64];
    snprintf(r, sizeof(r), "asym %.0f%%", asym * 100.0);
    calibFailed(r);
    sendAlert("ความชันสองฝั่งต่างกันมากเกินไป — หัววัดเสื่อม หรือมีจุดที่เก็บค่าเพี้ยน", 2);
    return;
  }

  // เก็บอุณหภูมิขณะ calibrate ไว้ใช้ชดเชยตอนวัดจริง
  float tCal = readTemperature();
  cfg.calibTempC = isnan(tCal) ? 25.0 : tCal;

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

  char msg[ALERT_MSG_LEN];
  snprintf(msg, sizeof(msg), "Calibration: R2=%.1f%%  %.1f mV/pH (%.0f%% ของ Nernst)  ที่ %.1fC -> %s",
           cfg.phR2, mvPerPh, mvPerPh / NERNST_MV_PER_PH * 100.0, cfg.calibTempC,
           cfg.isPhCalibrated ? "ผ่าน" : "ไม่ผ่าน (R2 ต่ำ)");
  logInfo(msg);
  sendAlert(msg, cfg.isPhCalibrated ? 0 : 2);

  showMessage(cfg.isPhCalibrated ? "CALIB OK" : "CALIB FAILED",
              "R2: " + String(cfg.phR2, 1) + "%");
  vTaskDelay(pdMS_TO_TICKS(3000));
}

void startCalibration() {
  if (xSemaphoreTake(calibMutex, pdMS_TO_TICKS(1000))) {
    state.calibMode  = true;
    state.calibStep  = 1;
    calibStartTime   = millis();
    calibStableSince = 0;
    gCalibDwellSec   = 0;
    gCalibStableSec  = 0;
    // ตั้งเป็น LOW ไว้ เพื่อบังคับให้ต้องปล่อยปุ่มก่อนถึงจะนับเป็นการกดครั้งใหม่
    // ไม่งั้นนิ้วที่ยังกดค้างจากตอนเข้าโหมดจะเก็บค่าจุดแรกทันทีตั้งแต่ยังไม่จุ่มน้ำยา
    calibBtnPrev     = LOW;
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
    const char* solutions[3] = {"Put pH 4.00", "Put pH 7.00", "Put pH 10.01"};
    const char* stepNames[3] = {"pH STEP 1/3", "pH STEP 2/3", "pH STEP 3/3"};
    int idx = state.calibStep - 1;

    float voltage = readPhVoltage(PH_SAMPLES_CALIB);
    trackPhVoltage(voltage);

    // สลับข้อความบรรทัดบนทุก 2 วิ ให้เห็นทั้งเลขขั้นและน้ำยาที่ต้องใช้
    static unsigned long lastToggle = 0;
    static bool showInstruction = false;
    if (isTimeElapsed(lastToggle, 2000)) {
      showInstruction = !showInstruction;
      lastToggle = millis();
    }

    updateLine(0, showInstruction ? solutions[idx] : stepNames[idx]);
    // บรรทัดล่าง: แรงดัน + ความคืบหน้าของการนิ่ง เช่น "V:1.876 S:7/10"
    String l2 = "V:" + String(isnan(voltage) ? 0.0 : voltage, 3);
    if (gPhStable) l2 += " S:" + String(gCalibStableSec) + "/" + String((int)(CALIB_STABLE_HOLD_MS / 1000));
    else           l2 += " wait..";
    updateLine(1, l2);

    // ── นับเวลาว่านิ่งต่อเนื่องมานานแค่ไหน ──
    if (gPhStable) {
      if (calibStableSince == 0) calibStableSince = millis();
    } else {
      calibStableSince = 0;   // สะดุดเมื่อไหร่ นับใหม่หมด
    }

    unsigned long dwellMs  = getElapsedTime(calibStartTime);
    unsigned long stableMs = calibStableSince ? getElapsedTime(calibStableSince) : 0;
    gCalibDwellSec  = dwellMs / 1000;
    gCalibStableSec = stableMs / 1000;

    // เก็บอัตโนมัติเมื่ออยู่ในขั้นนี้นานพอ "และ" นิ่งต่อเนื่องนานพอ
    bool autoReady = (dwellMs >= CALIB_MIN_DWELL_MS) && (stableMs >= CALIB_STABLE_HOLD_MS);

    // จับ "ขอบขาลง" ของปุ่ม ไม่ใช่ระดับ — กันกดค้างแล้วเก็บรัวหลายจุดติดกัน
    int  btnNow  = digitalRead(CALIB_BUTTON);
    bool btnEdge = (calibBtnPrev == HIGH && btnNow == LOW);
    calibBtnPrev = btnNow;

    // ยังกดเองได้ตลอด ถ้าไม่อยากรอ
    bool wantCapture = btnEdge || gCalibCaptureReq || autoReady;
    gCalibCaptureReq = false;

    bool captured = false;
    if (wantCapture) {
      showMessage("READING...", "hold still");
      // เก็บค่าที่ผ่านตัวกรองแล้ว ซึ่งเป็นค่าเฉลี่ยย้อนหลังราว 15 วินาที
      // ซึ่งทนสัญญาณรบกวนกว่าการเฉลี่ยสดๆ ไม่กี่วินาทีมาก
      cfg.phCalibVoltage[idx] = gPhVoltage;
      captured = true;
      calibStableSince = 0;
    } else if (isTimeElapsed(calibStartTime, CALIB_ABORT_MS)) {
      // ไม่เก็บค่าขยะ — ยกเลิกไปเลยดีกว่าได้เส้น calibration ที่เชื่อไม่ได้
      logWarning("ค่าไม่นิ่งภายใน 5 นาที — ยกเลิก calibration");
      sendAlert("ยกเลิก calibration: ค่าไม่นิ่งพอภายใน 5 นาที ตรวจหัววัดกับข้อต่อ BNC", 1);
      calibStableSince = 0;
      gCalibDwellSec   = 0;
      gCalibStableSec  = 0;
      showMessage("CALIB", "CANCELLED");
      vTaskDelay(pdMS_TO_TICKS(2500));
      state.calibMode = false;
      state.calibStep = 0;
    }

    if (captured) {
      char msg[ALERT_MSG_LEN];   // ตัวอักษรไทยกินตัวละ 3 ไบต์ 64 ไบต์ไม่พอ ข้อความจะโดนตัด
      snprintf(msg, sizeof(msg), "เก็บจุดที่ %d/3 ได้ %.4f V (%s)", state.calibStep,
               cfg.phCalibVoltage[idx], autoReady ? "อัตโนมัติ" : "สั่งเอง");
      logInfo(msg);
      sendAlert(msg, 0);

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

    if (cfg.isPhCalibrated) {
      cfg.calibrationCount++;          // นับเฉพาะครั้งที่ผ่านจริง
      showMessage("CALIBRATION", "COMPLETED!");
      logInfo("=== Calibration สำเร็จ ===");
    } else {
      showMessage("CALIB FAILED", "please retry");
      logWarning("=== Calibration ไม่ผ่าน ต้องทำใหม่ ===");
    }
    saveConfig();
    vTaskDelay(pdMS_TO_TICKS(2500));

    state.calibMode  = false;
    state.calibStep  = 0;
    gCalibDwellSec   = 0;
    gCalibStableSec  = 0;
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
  else if (key == "calibCapture"  && bv) { queueCmd(CMD_CALIB_CAPTURE); }
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
  copyUtf8(a.message, ALERT_MSG_LEN, msg);
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
  gJson.set("phVoltage",        isnan(gPhVoltage) ? -1.0 : (double)gPhVoltage);
  gJson.set("phStable",         (bool)gPhStable);
  gJson.set("phVoltageRaw",     isnan(gPhVoltageRaw) ? -1.0 : (double)gPhVoltageRaw);
  gJson.set("phNoiseMv",        (double)gPhNoiseMv);
  gJson.set("calibDwellSec",    (int)gCalibDwellSec);
  gJson.set("calibStableSec",   (int)gCalibStableSec);
  gJson.set("calibNeedDwell",   (int)(CALIB_MIN_DWELL_MS / 1000));
  gJson.set("calibNeedStable",  (int)(CALIB_STABLE_HOLD_MS / 1000));
  gJson.set("calibV1",          (double)cfg.phCalibVoltage[0]);
  gJson.set("calibV2",          (double)cfg.phCalibVoltage[1]);
  gJson.set("calibV3",          (double)cfg.phCalibVoltage[2]);
  gJson.set("calibSpanMv",      (double)(fabs(cfg.phCalibVoltage[2] - cfg.phCalibVoltage[0]) * 1000.0));
  gJson.set("calibMvPerPh",     (double)(fabs(cfg.phCalibVoltage[2] - cfg.phCalibVoltage[0]) * 1000.0 / CALIB_PH_RANGE));
  gJson.set("calibTempC",       (double)cfg.calibTempC);
  gJson.set("slopeAcidMv",      (double)(fabs(cfg.phCalibVoltage[0] - cfg.phCalibVoltage[1]) * 1000.0 / (CALIB_PH_2 - CALIB_PH_1)));
  gJson.set("slopeBaseMv",      (double)(fabs(cfg.phCalibVoltage[1] - cfg.phCalibVoltage[2]) * 1000.0 / (CALIB_PH_3 - CALIB_PH_2)));
  gJson.set("nernstPct",        (double)(fabs(cfg.phCalibVoltage[2] - cfg.phCalibVoltage[0]) * 1000.0 / CALIB_PH_RANGE / NERNST_MV_PER_PH * 100.0));
  gJson.set("temp",             isnan(temp) ? -127.0 : (double)temp);
  gJson.set("phValid",          !isnan(ph));
  gJson.set("dosing",           isDosingInProgress());
  gJson.set("lastAction",       state.lastAction);
  gJson.set("cooldownSec",      (int)cooldownLeft);
  gJson.set("phUp",             (bool)(digitalRead(PH_UP_RELAY)   == PH_UP_ON));
  gJson.set("phDown",           (bool)(digitalRead(PH_DOWN_RELAY) == PH_DOWN_ON));
  gJson.set("mixer",            (bool)(digitalRead(MIXER_RELAY)   == MIXER_ON));
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
  gJson.set("minHeap",          (int)esp_get_minimum_free_heap_size());          // จุดต่ำสุดตั้งแต่บูต
  gJson.set("maxBlock",         (int)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)); // บล็อกต่อเนื่องใหญ่สุด
  gJson.set("uptimeMin",        (int)(millis() / 60000));
  gJson.set("wifiRSSI",         WiFi.RSSI());
  gJson.set("fwVersion",        FW_VERSION);
  gJson.set("resetReason",      gResetReason);
  gJson.set("rebootCount",      (int)gRebootCount);
  gJson.set("lastRebootReason", gLastRebootReason[0] ? gLastRebootReason : "-");
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
  const char* buttons[] = {"phUp", "phDown", "mixer", "startCalib", "calibCapture", "reboot"};
  for (int i = 0; i < 6; i++) {
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
    char msg[ALERT_MSG_LEN];
    snprintf(msg, sizeof(msg), "เติมค่าตั้งที่ขาดลง /control แล้ว %d คีย์", seeded);
    logInfo(msg);
  }

  fbdo.clear();
}

void firebaseStartStream() {
  if (Firebase.RTDB.beginStream(&fbdoStream, "/control")) {
    // ★ stack ของ stream task ค่า default คือ 8192 ซึ่งตึงมาก เพราะ callback ของเรา
    //   ต้องวน FirebaseJson + สร้าง String ซ้อนอยู่บนงาน SSL/parse ของไลบรารีเอง
    //   ขยายเป็น 16384 เท่ากับ taskFirebase (RAM เหลือเฟือ ใช้อยู่แค่ 15%)
    Firebase.RTDB.setStreamCallback(&fbdoStream, streamCallback, streamTimeoutCallback, 16384);
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
    beat(T_SENSOR);

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
    float ph   = readPH(temp);   // ส่งอุณหภูมิเข้าไปชดเชยตาม Nernst
    updateSensorReadings(ph, temp);

    // นับเฉพาะ pH เท่านั้น — อุณหภูมิใช้แค่แสดงผลกับเก็บ log ไม่ได้เอาไปคำนวณ pH
    // ถ้านับ temp ด้วย พอ DS18B20 หลุดสายระบบจะหยุดฉุกเฉินทั้งที่ยังคุม pH ได้ปกติ
    if (isnan(ph)) {
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
    char msg[ALERT_MSG_LEN];
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
    beat(T_CONTROL);
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
          digitalWrite(PH_UP_RELAY,   PH_UP_OFF);
          digitalWrite(PH_DOWN_RELAY, PH_DOWN_OFF);
          digitalWrite(MIXER_RELAY,   MIXER_OFF);
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
    beat(T_DISPLAY);

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
        digitalWrite(MIXER_RELAY, gMixerWant ? MIXER_ON : MIXER_OFF);
        logInfo(gMixerWant ? "เปิดเครื่องกวน (สั่งมือ)" : "ปิดเครื่องกวน (สั่งมือ)");
        break;

      case CMD_START_CALIB:
        if (!state.calibMode) startCalibration();
        ackPath = "/control/startCalib";
        break;

      case CMD_CALIB_CAPTURE:
        // ให้ FSM ใน taskSensor เป็นคนเก็บค่าจริง ที่นี่แค่ตั้งธง
        if (state.calibMode) gCalibCaptureReq = true;
        ackPath = "/control/calibCapture";
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

  unsigned long lastStatus    = 0;
  unsigned long lastLowMemLog = 0;
  unsigned long lastHistory   = 0;
  unsigned long lastWiFiTry   = 0;
  float         lastSentPH    = -999.0;
  uint32_t      consecFails   = 0;

  // ── ตัวจับเวลาของระบบเฝ้าระวัง ──
  unsigned long wifiDownSince   = 0;   // 0 = ปกติดี
  unsigned long fbNotReadySince = 0;
  unsigned long lowMemSince     = 0;
  unsigned long streamDownSince = 0;
  bool          fbReinitDone    = false;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_task_wdt_reset();
    beat(T_FIREBASE);

    unsigned long now = millis();

    // ── ตรวจชีพจรของ task อื่น ──
    // WDT จับ task ที่ค้างได้ แต่จับ task ที่ตายหายไปเลยไม่ได้
    for (int i = 0; i < TASK_COUNT; i++) {
      if (i == T_FIREBASE || gTaskBeat[i] == 0) continue;
      if (isTimeElapsed(gTaskBeat[i], TASK_BEAT_TIMEOUT_MS)) {
        char msg[ALERT_MSG_LEN];
        snprintf(msg, sizeof(msg), "task %s ไม่ตอบสนองเกิน %lu วินาที",
                 kTaskName[i], TASK_BEAT_TIMEOUT_MS / 1000);
        safeReboot(msg);
      }
    }

    // ── รีบูตกันเหนียวตามกำหนด เฉพาะตอนที่ไม่ได้ทำงานอะไรค้างอยู่ ──
    if (DAILY_REBOOT_MS > 0 && millis() >= DAILY_REBOOT_MS
        && !isDosingInProgress() && !state.calibMode && !state.emergencyStop) {
      safeReboot("ครบกำหนดรีบูตประจำวัน (ไม่มีงานค้าง)");
    }

    // ── WiFi หลุด: พยายามต่อใหม่ ──
    if (WiFi.status() != WL_CONNECTED) {
      gWiFiConnected = false;
      gFirebaseReady = false;
      if (wifiDownSince == 0) wifiDownSince = now;

      // ต่อไม่ติดนานเกินไป = รีบูตดีกว่าปล่อยให้ระบบตาบอดทั้งคืน
      if (isTimeElapsed(wifiDownSince, WIFI_DEAD_REBOOT_MS)) {
        safeReboot("WiFi ต่อไม่ติดเกิน 15 นาที");
      }

      if (isTimeElapsed(lastWiFiTry, FB_WIFI_RETRY_MS)) {
        lastWiFiTry = now;
        if (wifiConnect()) syncNTP();   // ต้อง sync เวลาใหม่ทุกครั้งที่ต่อ WiFi ใหม่
      }
      continue;
    }
    gWiFiConnected = true;
    wifiDownSince  = 0;

    // ── Firebase ยังไม่พร้อม: รอ แล้วไล่ระดับการกู้คืน ──
    if (!Firebase.ready()) {
      gFirebaseReady = false;
      if (fbNotReadySince == 0) fbNotReadySince = now;

      // ขั้นที่ 1: init ใหม่ (token อาจพัง หรือ signUp ค้าง)
      if (!fbReinitDone && isTimeElapsed(fbNotReadySince, FB_REINIT_AFTER_MS)) {
        fbReinitDone = true;
        logWarning("Firebase ไม่พร้อมนานเกินไป — init ใหม่");
        sendAlert("Firebase ไม่พร้อมเกิน 10 นาที กำลัง init ใหม่", 1);
        fbdo.clear();
        fbdoStream.clear();
        gStreamStarted = false;
        firebaseInit();
      }

      // ขั้นที่ 2: init ใหม่แล้วยังไม่ขึ้น = ยอมแพ้ รีบูต
      if (isTimeElapsed(fbNotReadySince, FB_DEAD_REBOOT_MS)) {
        safeReboot("Firebase ไม่พร้อมเกิน 20 นาที");
      }

      vTaskDelay(pdMS_TO_TICKS(FB_NOT_READY_BACKOFF));
      continue;
    }

    if (!gFirebaseReady) {
      gFirebaseReady = true;
      logInfo("Firebase พร้อมใช้งาน");
    }
    fbNotReadySince = 0;
    fbReinitDone    = false;

    // ── stream ตายเงียบ: เปิดใหม่ ──
    // ไม่ดูจาก "ไม่มีข้อมูลเข้ามา" เพราะ /control นานๆ เปลี่ยนที
    // ดูจากสถานะการเชื่อมต่อจริงของ FirebaseData ที่ใช้ stream แทน
    if (gStreamStarted && !fbdoStream.httpConnected()) {
      if (streamDownSince == 0) streamDownSince = now;
      if (isTimeElapsed(streamDownSince, STREAM_DEAD_MS)) {
        streamDownSince = 0;
        logWarning("stream /control หลุดนานเกินไป — เปิดใหม่");
        sendAlert("stream /control หลุด กำลังเปิดใหม่", 1);
        Firebase.RTDB.endStream(&fbdoStream);
        fbdoStream.clear();
        gStreamStarted = false;
      }
    } else {
      streamDownSince = 0;
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
    uint32_t interval = state.calibMode      ? FB_STATUS_CALIB_MS
                      : isDosingInProgress() ? FB_STATUS_ACTIVE_MS
                                             : FB_STATUS_IDLE_MS;

    // หน่วยความจำต่อเนื่องเหลือน้อย = อย่าเพิ่งเปิด SSL รอบใหม่ ไม่งั้น handshake พัง
    // แล้วไลบรารีจะทิ้งทั้ง stream และ connection หลักไปด้วย
    size_t maxBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    if (maxBlock < FB_MIN_FREE_BLOCK) {
      if (lowMemSince == 0) lowMemSince = now;

      // ต่ำค้างนานแปลว่าหน่วยความจำรั่วหรือแตกจนกู้เองไม่ได้แล้ว
      if (isTimeElapsed(lowMemSince, LOWMEM_REBOOT_MS)) {
        safeReboot("หน่วยความจำต่อเนื่องต่ำค้างเกิน 5 นาที");
      }

      if (isTimeElapsed(lastLowMemLog, 30000)) {
        lastLowMemLog = now;
        Serial.printf("[FB] บล็อกต่อเนื่องเหลือ %u ไบต์ ข้ามการส่งรอบนี้\n", (unsigned)maxBlock);
      }
      continue;
    }
    lowMemSince = 0;

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

  // ── บันทึกว่ารอบก่อนดับเพราะอะไร ──
  esp_reset_reason_t rr = esp_reset_reason();
  copyUtf8(gResetReason, sizeof(gResetReason), resetReasonText(rr));

  // ตัวนับใน RTC memory อยู่รอด soft reset แต่หายเมื่อไฟดับ
  // ต้องเช็ค magic ก่อน ไม่งั้นจะได้ค่าขยะตอนเปิดไฟครั้งแรก
  if (gRtcMagic != RTC_MAGIC || rr == ESP_RST_POWERON) {
    gRtcMagic    = RTC_MAGIC;
    gRebootCount = 0;
    gLastRebootReason[0] = '\0';
  }

  Serial.printf("[BOOT] สาเหตุการรีเซ็ตรอบก่อน: %s\n", gResetReason);
  if (gLastRebootReason[0]) Serial.printf("[BOOT] รีบูตเองเพราะ: %s\n", gLastRebootReason);
  Serial.printf("[BOOT] รีบูตเองมาแล้ว %lu ครั้งตั้งแต่เสียบไฟ\n", (unsigned long)gRebootCount);

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
  digitalWrite(PH_UP_RELAY,   PH_UP_OFF);
  digitalWrite(PH_DOWN_RELAY, PH_DOWN_OFF);
  digitalWrite(MIXER_RELAY,   MIXER_OFF);
  pinMode(CALIB_BUTTON, INPUT_PULLUP);

  analogSetAttenuation(ADC_11db);   // ให้วัดได้ถึง ~3.3V

  // จำกัดเวลารอบัส I2C — ถ้าสาย LCD หลุดหรือบัสค้าง lcd.print() จะบล็อกทั้ง task
  // ตั้ง timeout ไว้ให้มันคืนค่าพลาดแทนที่จะรอไปเรื่อยๆ
  Wire.begin();
  Wire.setTimeOut(50);

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

  // แจ้งขึ้น Firebase ว่าเพิ่งบูต และเพราะอะไร — ระบบที่รัน 24 ชม.
  // ต้องตามรอยได้ว่าเคยรีบูตตอนไหนบ้างและด้วยสาเหตุอะไร
  {
    char msg[ALERT_MSG_LEN];
    if (gLastRebootReason[0])
      snprintf(msg, sizeof(msg), "บอร์ดเริ่มทำงาน (%s) — รอบก่อนรีบูตเพราะ: %s",
               gResetReason, gLastRebootReason);
    else
      snprintf(msg, sizeof(msg), "บอร์ดเริ่มทำงาน (%s)", gResetReason);
    sendAlert(msg, (strstr(gResetReason, "crash") || strstr(gResetReason, "watchdog")
                    || strstr(gResetReason, "ไฟตก")) ? 2 : 0);
  }

  showMessage("SYSTEM READY", cfg.isPhCalibrated ? "OK" : "Need calib!");

  esp_task_wdt_delete(NULL);   // ปล่อย loop task ออกจาก WDT ก่อนจะลบทิ้ง
}

// ============================================================================
//  LOOP — ไม่ใช้ ทุกอย่างอยู่ใน FreeRTOS task
// ============================================================================
void loop() {
  vTaskDelete(NULL);
}
