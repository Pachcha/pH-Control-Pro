# โครงสร้างข้อมูลบน Firebase Realtime Database

ESP32 เขียนที่ `/status`, `/lastReading`, `/history`, `/alerts`
Dashboard เขียนที่ `/control` แล้ว ESP32 อ่านผ่าน **stream** (ได้ทันทีไม่ต้อง poll)

```
root
├── status/          ← ESP32 เขียน  (ทุก 5 วิ ตอนทำงาน / 15 วิ ตอนอยู่เฉย)
├── lastReading/     ← ESP32 เขียน  (เมื่อค่า pH เปลี่ยนเกิน 0.02)
├── history/         ← ESP32 push   (ทุก 5 นาที ใช้วาดกราฟ)
├── alerts/          ← ESP32 push   (เมื่อมีเหตุการณ์สำคัญ)
└── control/         ← Dashboard เขียน / ESP32 อ่าน
```

---

## `/status`

| คีย์ | ชนิด | ความหมาย |
|---|---|---|
| `ph` | number | ค่า pH ปัจจุบัน (`-1` = อ่านไม่ได้) |
| `temp` | number | อุณหภูมิน้ำ °C (`-127` = อ่านไม่ได้) |
| `phValid` | bool | ค่า pH ใช้งานได้หรือไม่ |
| `dosing` | bool | กำลังจ่ายสารอยู่หรือไม่ |
| `lastAction` | string | `Idle` / `pH Up` / `pH Down` |
| `cooldownSec` | int | เหลือกี่วินาทีถึงจะจ่ายรอบถัดไปได้ |
| `phUp` `phDown` `mixer` | bool | สถานะรีเลย์แต่ละตัว |
| `calibrated` | bool | calibrate ผ่านแล้วหรือยัง |
| `calibMode` | bool | กำลังอยู่ในโหมด calibrate |
| `calibStep` | int | ขั้นที่กำลังทำ (1–3) |
| `phR2` | number | ความแม่นของเส้น calibration (%) — ต้อง > 90 ถึงจะผ่าน |
| `calibCount` | int | calibrate มาแล้วกี่ครั้ง |
| `targetPH` `phTolerance` | number | ค่าเป้าหมายที่ใช้อยู่จริง (สะท้อนกลับจาก EEPROM) |
| `dosingTimeSec` `cooldownConfSec` | int | ค่าตั้งที่ใช้อยู่จริง |
| `autoMode` | bool | โหมดอัตโนมัติเปิดอยู่หรือไม่ |
| `emergencyStop` | bool | อยู่ในโหมดหยุดฉุกเฉิน |
| `doseLimitHit` | bool | จ่ายครบลิมิตต่อชั่วโมงแล้ว |
| `dosesThisHour` `maxDosesPerHour` | int | จ่ายไปกี่ครั้ง / ลิมิต |
| `sensorErrorCount` | int | อ่านเซนเซอร์พลาดติดกันกี่ครั้ง |
| `freeHeap` | int | หน่วยความจำว่าง (bytes) |
| `uptimeMin` | int | เปิดเครื่องมากี่นาที |
| `wifiRSSI` | int | ความแรงสัญญาณ WiFi (dBm) |
| `fwVersion` | string | เวอร์ชันเฟิร์มแวร์ |
| `timestamp` | int | เวลาของเซิร์ฟเวอร์ (`.sv: "timestamp"`) |

## `/lastReading` และ `/history/<pushId>`

| คีย์ | ชนิด | ความหมาย |
|---|---|---|
| `ph` | number | ค่า pH |
| `temp` | number | อุณหภูมิ °C |
| `timestamp` | int | เวลาของเซิร์ฟเวอร์ |

## `/alerts/<pushId>`

| คีย์ | ชนิด | ความหมาย |
|---|---|---|
| `message` | string | ข้อความ (ภาษาไทย) |
| `priority` | int | `0` = ข้อมูล, `1` = เตือน, `2` = ผิดพลาด |
| `timestamp` | int | เวลาของเซิร์ฟเวอร์ |

## `/control`

### ค่าตั้ง — เขียนแล้วอยู่ถาวร ESP32 จะบันทึกลง EEPROM ให้

| คีย์ | ชนิด | ช่วงที่ยอมรับ |
|---|---|---|
| `targetPH` | number | 3.0 – 11.0 |
| `phTolerance` | number | 0.05 – 2.0 |
| `dosingTimeSec` | int | 1 – 60 |
| `cooldownSec` | int | 30 – 3600 |
| `autoMode` | bool | – |
| `emergencyStop` | bool | – |
| `mixer` | bool | สั่งเปิด/ปิดเครื่องกวนด้วยมือ |

### ปุ่มกดครั้งเดียว — เขียน `true` แล้ว ESP32 จะเขียน `false` กลับให้เองหลังทำงานเสร็จ

| คีย์ | ทำอะไร |
|---|---|
| `phUp` | สั่งจ่าย pH Up ทันที (ข้าม cooldown แต่ยังนับลิมิตต่อชั่วโมง) |
| `phDown` | สั่งจ่าย pH Down ทันที |
| `startCalib` | เริ่ม calibrate 3 จุด |
| `reboot` | รีบูตอุปกรณ์ |

> **ทำไมต้องเขียน `false` กลับ**
> ถ้าปล่อยให้ค้างเป็น `true` พออุปกรณ์รีบูตแล้ว stream ส่งค่าเดิมกลับมา ระบบจะสั่งจ่ายสารทันทีโดยไม่ตั้งใจ
> เฟิร์มแวร์จึงเคลียร์ปุ่มทั้งหมดเป็น `false` ทุกครั้งก่อนเปิด stream (ฟังก์ชัน `firebaseSeedControl()`)
