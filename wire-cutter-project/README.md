# Wire Cutter & Stripper - ESP32

เครื่องตัดและปลอกสายไฟอัตโนมัติ พร้อมระบบ IoT (MQTT + WebSocket Queue)

## โครงสร้างโปรเจค

```
wire-cutter-project/
├── src/
│   └── main.cpp          ← Firmware ESP32 (ตัวหลัก)
├── data/
│   └── index.html        ← Frontend Dashboard (WebSocket Queue)
├── docs/
│   └── WIRING.md         ← แผนผังวงจร + Pin Mapping
├── .vscode/
│   ├── settings.json
│   └── extensions.json
├── platformio.ini        ← PlatformIO Config
└── README.md             ← ไฟล์นี้
```

## วิธีใช้งาน (VSCode + PlatformIO)

### 1. ติดตั้ง
- ติดตั้ง [VSCode](https://code.visualstudio.com/)
- ติดตั้ง Extension: **PlatformIO IDE**
- เปิดโฟลเดอร์ `wire-cutter-project/` ใน VSCode

### 2. ตั้งค่า WiFi
แก้ไฟล์ `src/main.cpp` บรรทัด:
```cpp
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
```

### 3. Upload Firmware
```
PlatformIO: Upload  (หรือกด ➡️ ที่ toolbar ล่าง)
```

### 4. Upload Frontend (SPIFFS)
```
PlatformIO > Upload Filesystem Image
```
หรือ terminal:
```bash
pio run -t uploadfs
```

### 5. เปิด Dashboard
- เปิด Serial Monitor ดู IP ที่ ESP32 ได้รับ
- เปิดเบราว์เซอร์ไปที่ `http://<ESP32_IP>/`
- Dashboard จะ auto-connect WebSocket

## ระบบ IoT

### MQTT (ส่ง status)
- Broker: `broker.hivemq.com:1883`
- Topic: `wireCutter/status`
- ส่ง JSON ทุก 2 วินาที: state, phase, progress, wire_present, queue_size

### WebSocket (ระบบคิว)
- Port: `81`
- เพิ่มงาน: `{"cmd":"add", "label":"...", "length_cm":10, "quantity":5, "strip_mm":8}`
- ลบงาน: `{"cmd":"remove", "idx":0}`
- เริ่มทำคิว: `{"cmd":"start"}`
- หยุด: `{"cmd":"stop"}`

## การใช้งานจากปุ่มบนเครื่อง

หน้าจอ IDLE → กด OK:
- ถ้ามีงานในคิว → เริ่มทำคิวทันที
- ถ้าไม่มีคิว → เข้าโหมดตั้งค่าแมนนวล (ความยาว → จำนวน → ปลอก → ยืนยัน)

## Pin Mapping (PCB)

ดูรายละเอียดที่ `docs/WIRING.md`
