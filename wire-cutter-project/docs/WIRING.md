# การต่อวงจร - PCB Schematic Pin Mapping

## Pin Mapping (จาก EasyEDA Schematic → ESP32 DEVKITC V4)

| อุปกรณ์ / Net Label         | ESP32 Pin | PCB Connector      | หมายเหตุ                          |
|-----------------------------|-----------|---------------------|------------------------------------|
| **Stepper Motor (TB6600)**  |           |                     |                                    |
| DIR+                        | GPIO 32   | XY302V (Stepper1)   | ทิศทาง                             |
| PUL+                        | GPIO 33   | XY302V (Stepper2)   | Pulse                              |
| GND                         | GND       | XY302V              | Common GND                         |
| **Servo #1 กรรไกร (SRV1)**  |           |                     |                                    |
| SRV1                        | GPIO 35   | PZ254V (Servo1)     | High Torque - บีบกรรไกร            |
| **Servo #2 ท่อนำสาย (SRV2)**|           |                     |                                    |
| SRV2                        | GPIO 26   | PZ254V (Servo2)     | 5V - ทิศทางท่อนำสาย               |
| **IR Sensors**              |           |                     |                                    |
| SENSOR1                     | GPIO 39   | PZ254V (Sensor1)    | ตรวจสายเข้า + R1 pull-up 1KΩ      |
| SENSOR2                     | GPIO 34   | PZ254V (Sensor2)    | ตรวจสายออก + R2 pull-up 1KΩ       |
| **Buttons (มี RC debounce 100nF บน PCB)** |  |              |                                    |
| BTN1                        | GPIO 14   | SW1 (TC-6610)       | เพิ่มค่า + C2 100nF               |
| BTN2                        | GPIO 12   | SW2 (TC-6610)       | ยืนยัน + C3 100nF                 |
| BTN3                        | GPIO 13   | SW3 (TC-6610)       | ลดค่า + C4 100nF                  |
| **ST7735 TFT (OLEDLCD connector)** |     | PMZ 54-1X8PM-H85   |                                    |
| SCLK                        | GPIO 18   | Pin 1               |                                    |
| MOSI                        | GPIO 23   | Pin 2               |                                    |
| RST                         | GPIO 17   | Pin 3               |                                    |
| DC                          | GPIO 16   | Pin 4               |                                    |
| CS                          | GPIO 5    | Pin 5               |                                    |
| **Power**                   |           |                     |                                    |
| VCC                         | 3V3       | WJ500V (Power)      | + C5 100nF decoupling              |
| GND                         | GND       | WJ500V (Power)      |                                    |

## IoT System

### MQTT (ส่ง status)
- Broker: `broker.hivemq.com:1883` (หรือ broker ส่วนตัว)
- Topic publish: `wireCutter/status` — ส่ง JSON ทุก 2 วินาที
- Topic subscribe: `wireCutter/cmd` — รับคำสั่ง (optional)

### WebSocket (ระบบคิว)
- Port: `81` — เปิดหน้า `dashboard.html` แล้วใส่ IP ของ ESP32
- Commands: `add`, `remove`, `start`, `stop`, `get_queue`

## ลำดับการทำงานต่อ 1 เส้น

```
1. ป้อนสาย strip_length      ← Stepper
2. Guide → ทิศปลอก           ← Guide Servo
3. Cutter บีบบากฉนวนหัว      ← Cutter Servo (บีบเบา)
4. เปิดกรรไกร
5. ป้อนสาย (ความยาว + strip) ← Stepper
6. Guide → ทิศปลอก           ← Guide Servo
7. Cutter บีบบากฉนวนท้าย     ← Cutter Servo (บีบเบา)
8. เปิดกรรไกร
9. Guide → ทิศตัด            ← Guide Servo
10. Cutter บีบตัดขาด          ← Cutter Servo (บีบเต็มที่)
11. เปิดกรรไกร → วนซ้ำ
```

## ค่าที่ต้องปรับตามกลไกจริง

| ค่า                   | ค่าเริ่มต้น | คำอธิบาย                              |
|-----------------------|-------------|----------------------------------------|
| CUTTER_OPEN_ANGLE     | 10°         | กรรไกรเปิด                            |
| CUTTER_STRIP_ANGLE    | 120°        | บีบแค่บากฉนวน (ปรับให้ไม่ตัดทองแดง)   |
| CUTTER_CUT_ANGLE      | 170°        | บีบเต็มที่ตัดขาด                      |
| GUIDE_STRIP_ANGLE     | 40°         | ท่อนำสายชี้ไปส่วนปลายกรรไกร           |
| GUIDE_CUT_ANGLE       | 140°        | ท่อนำสายชี้ไปส่วนในกรรไกร             |
| MM_PER_REV            | 34.56       | ปรับตาม Mk8 Extruder gear             |
| IR_THRESHOLD          | 2000        | ปรับตาม IR Sensor                     |
