/*
 * =============================================================
 *  เครื่องตัดและปลอกสายไฟอัตโนมัติ + IoT
 *  Automatic Wire Cutter & Stripper with MQTT + WebSocket
 *  MCU: ESP32 DEVKITC V4
 * =============================================================
 * 
 * Pin mapping จาก PCB Schematic (EasyEDA)
 * 
 * หลักการทำงาน:
 *   กรรไกร 1 ตัว 2 ส่วน (ปลาย=ปลอก, ใน=ตัด)
 *   Guide Servo สลับทิศ: ปลอก ↔ ตัด
 *   Cutter Servo บีบกรรไกร 3 ระดับ: เปิด / บากฉนวน / ตัดขาด
 *
 * ลำดับต่อ 1 เส้น:
 *   1. ป้อนสาย strip_length → Guide ทิศปลอก → Cutter บาก → เปิด
 *   2. ป้อนสาย (ความยาว + strip) → Guide ทิศปลอก → Cutter บาก → เปิด
 *   3. Guide ทิศตัด → Cutter ตัดขาด → เปิด
 *
 * IoT:
 *   - MQTT: ส่ง status การทำงาน (state, progress, phase)
 *   - WebSocket: ระบบคิวการตัดสายไฟ (เพิ่ม/ลบ/ดูคิว)
 *
 * CLO 1: GPIO, Interrupt, PWM, Deep Sleep
 * CLO 2: Software debounce + hardware RC (100nF caps on PCB)
 * CLO 3: TB6600 driver with opto-coupler isolation
 * CLO 4: MQTT + WebSocket IoT
 * CLO 5: Complete embedded system with PCB
 */

#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <WebSocketsServer.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// ==================================================================
//  PIN DEFINITIONS - ตาม PCB Schematic (EasyEDA)
// ==================================================================

// Stepper Motor (TB6600) - ผ่าน XY302V connectors
#define STEPPER_DIR_PIN   32   // GPIO32 → DIR+
#define STEPPER_PUL_PIN   33   // GPIO33 → PUL+
// ENA ไม่ได้ต่อบน PCB → ต่อ ENA- กับ ENA+ ที่ TB6600 เพื่อ always enable
// หรือถ้าต้องการควบคุม ให้ต่อสายเพิ่ม

// Servo Motors - ผ่าน PZ254V-11-03P connectors
#define SERVO_CUTTER_PIN  25   // GPIO25 → SRV1 (Servo1 - กรรไกร High Torque)
// NOTE: GPIO35 is input-only on ESP32 and cannot output PWM. Changed to GPIO25.
//       Update this to match your actual PCB wiring.
#define SERVO_GUIDE_PIN   26   // GPIO26 → SRV2 (Servo2 - ท่อนำสาย 5V)

// IR Sensors - ผ่าน PZ254V-11-03P connectors + pull-up 1KΩ
#define SENSOR1_PIN       39   // VN (GPIO39) → SENSOR1 (ตรวจสายไฟเข้า)
#define SENSOR2_PIN       34   // GPIO34 → SENSOR2 (ตรวจสายไฟออก)

// Push Buttons - มี RC debounce 100nF บน PCB + Tact Switch
#define BTN_UP_PIN        14   // GPIO14 → BTN1/SW1 (เพิ่มค่า)
#define BTN_OK_PIN        12   // GPIO12 → BTN2/SW2 (ยืนยัน)
#define BTN_DOWN_PIN      13   // GPIO13 → BTN3/SW3 (ลดค่า)

// ST7735 TFT SPI - ผ่าน PMZ 54-1X8PM-H85 connector (OLEDLCD)
#define TFT_SCLK          18   // GPIO18 → SCLK
#define TFT_MOSI          23   // GPIO23 → MOSI
#define TFT_CS             5   // GPIO05 → CS
#define TFT_DC            16   // GPIO16 → DC
#define TFT_RST           17   // GPIO17 → RST

// ==================================================================
//  DISPLAY COLORS
// ==================================================================
#define C_BG              ST77XX_BLACK
#define C_TEXT            ST77XX_WHITE
#define C_TITLE           ST77XX_CYAN
#define C_VALUE           0xFD20
#define C_ACCENT          0x07E0
#define C_WARNING         0xFFE0
#define C_ERROR           ST77XX_RED
#define C_PROGRESS_BG     0x2104
#define C_PROGRESS        0x07FF
#define C_HEADER_BG       0x000F
#define C_DIM             0x7BEF

// ==================================================================
//  CONFIGURATION
// ==================================================================

// WiFi
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// MQTT Broker
const char* MQTT_BROKER   = "broker.hivemq.com";  // หรือ broker ส่วนตัว
const int   MQTT_PORT     = 1883;
const char* MQTT_CLIENT   = "wire_cutter_esp32";
const char* MQTT_TOPIC_STATUS = "wireCutter/status";
const char* MQTT_TOPIC_CMD    = "wireCutter/cmd";

// Stepper Motor
const int    STEPS_PER_REV       = 1600;
const float  MM_PER_REV          = 34.56;
const float  STEPS_PER_MM        = STEPS_PER_REV / MM_PER_REV;
const int    STEPPER_PULSE_DELAY = 200;

// Cutter Servo (High Torque)
const int CUTTER_OPEN_ANGLE    = 10;
const int CUTTER_STRIP_ANGLE   = 120;
const int CUTTER_CUT_ANGLE     = 170;
const int CUTTER_ACTION_DELAY  = 600;

// Guide Servo (5V)
const int GUIDE_STRIP_ANGLE    = 40;
const int GUIDE_CUT_ANGLE      = 140;
const int GUIDE_MOVE_DELAY     = 300;

// IR Sensor
const int IR_THRESHOLD = 2000;

// Deep Sleep
const unsigned long SLEEP_TIMEOUT_MS = 5UL * 60 * 1000;

// Debounce (software + hardware RC 100nF on PCB)
const unsigned long DEBOUNCE_MS = 30;  // ลดลงเพราะมี hardware debounce แล้ว

// ==================================================================
//  ENUMS & STRUCTS
// ==================================================================

enum SystemState {
  STATE_IDLE,            // รอคำสั่ง / ดูคิว
  STATE_SET_LENGTH,
  STATE_SET_QUANTITY,
  STATE_SET_STRIP_LEN,
  STATE_CONFIRM,
  STATE_WORKING,
  STATE_PAUSED,
  STATE_DONE
};

enum WorkPhase {
  PHASE_STRIP_HEAD,
  PHASE_FEED_WIRE,
  PHASE_STRIP_TAIL,
  PHASE_CUT
};

// Queue item สำหรับคิวตัดสาย
struct QueueItem {
  int lengthCm;
  int quantity;
  int stripMm;
  int done;         // จำนวนที่ทำเสร็จแล้ว
  bool active;      // กำลังทำอยู่
  char label[32];   // ชื่อ/label ของงาน
};

const int MAX_QUEUE = 10;

// ==================================================================
//  GLOBAL OBJECTS
// ==================================================================
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Servo cutterServo;
Servo guideServo;
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
WebSocketsServer webSocket(81);
WebServer httpServer(80);

// ==================================================================
//  STATE VARIABLES
// ==================================================================
volatile SystemState currentState = STATE_IDLE;
SystemState lastDrawnState = (SystemState)-1;
WorkPhase workPhase = PHASE_STRIP_HEAD;

// ตั้งค่าจากปุ่ม (manual mode)
int wireLengthCm  = 10;
int wireQuantity  = 1;
int stripLengthMm = 8;

// Queue
QueueItem queue[MAX_QUEUE];
int queueCount    = 0;
int currentQueueIdx = -1;  // index ที่กำลังทำ

// Working state
int wiresDone     = 0;
int lastDrawnValue = -1;
int lastDrawnDone  = -1;
WorkPhase lastDrawnPhase = (WorkPhase)-1;

// Buttons
volatile bool btnUpFlag   = false;
volatile bool btnOkFlag   = false;
volatile bool btnDownFlag = false;
volatile unsigned long lastBtnUpTime   = 0;
volatile unsigned long lastBtnOkTime   = 0;
volatile unsigned long lastBtnDownTime = 0;

// Timers
unsigned long lastActivityTime = 0;
unsigned long lastMqttPublish  = 0;

// ==================================================================
//  FORWARD DECLARATIONS
// ==================================================================
bool isWirePresent();
void startNextQueueItem();
void updateWorkingDisplay();
void updateDisplay();
void drawWorkingProgress();
void drawWorkPhase();

// ==================================================================
//  ISR (CLO1)
// ==================================================================

void IRAM_ATTR isrUp() {
  unsigned long now = millis();
  if (now - lastBtnUpTime > DEBOUNCE_MS) { btnUpFlag = true; lastBtnUpTime = now; }
}
void IRAM_ATTR isrOk() {
  unsigned long now = millis();
  if (now - lastBtnOkTime > DEBOUNCE_MS) { btnOkFlag = true; lastBtnOkTime = now; }
}
void IRAM_ATTR isrDown() {
  unsigned long now = millis();
  if (now - lastBtnDownTime > DEBOUNCE_MS) { btnDownFlag = true; lastBtnDownTime = now; }
}

// ==================================================================
//  STEPPER MOTOR (CLO3)
// ==================================================================

void stepperMove(float mm, bool forward) {
  int steps = (int)(abs(mm) * STEPS_PER_MM);
  digitalWrite(STEPPER_DIR_PIN, forward ? HIGH : LOW);
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEPPER_PUL_PIN, HIGH);
    delayMicroseconds(STEPPER_PULSE_DELAY);
    digitalWrite(STEPPER_PUL_PIN, LOW);
    delayMicroseconds(STEPPER_PULSE_DELAY);
  }
}

void feedForward(float mm) { stepperMove(mm, true); }

bool feedForwardChecked(float mm) {
  int totalSteps = (int)(mm * STEPS_PER_MM);
  int chunkSize = 100;
  int remaining = totalSteps;
  
  digitalWrite(STEPPER_DIR_PIN, HIGH);
  while (remaining > 0) {
    if (!isWirePresent()) return false;
    int steps = min(chunkSize, remaining);
    for (int i = 0; i < steps; i++) {
      digitalWrite(STEPPER_PUL_PIN, HIGH);
      delayMicroseconds(STEPPER_PULSE_DELAY);
      digitalWrite(STEPPER_PUL_PIN, LOW);
      delayMicroseconds(STEPPER_PULSE_DELAY);
    }
    remaining -= steps;
  }
  return true;
}

// ==================================================================
//  SERVO FUNCTIONS (CLO1 - PWM)
// ==================================================================

void guideToStrip() { guideServo.write(GUIDE_STRIP_ANGLE); delay(GUIDE_MOVE_DELAY); }
void guideToCut()   { guideServo.write(GUIDE_CUT_ANGLE);   delay(GUIDE_MOVE_DELAY); }

void cutterOpen()  { cutterServo.write(CUTTER_OPEN_ANGLE);  delay(300); }
void cutterStrip() { cutterServo.write(CUTTER_STRIP_ANGLE); delay(CUTTER_ACTION_DELAY); }
void cutterCut()   { cutterServo.write(CUTTER_CUT_ANGLE);   delay(CUTTER_ACTION_DELAY); }

// ==================================================================
//  SENSORS
// ==================================================================

bool isWirePresent() {
  // ใช้ SENSOR1 (สายเข้า) เป็นหลัก
  int val = analogRead(SENSOR1_PIN);
  return (val < IR_THRESHOLD);
}

bool isWireAtOutput() {
  int val = analogRead(SENSOR2_PIN);
  return (val < IR_THRESHOLD);
}

// ==================================================================
//  MQTT (CLO4) - ส่ง status การทำงาน
// ==================================================================

const char* getStateName() {
  switch (currentState) {
    case STATE_IDLE:          return "IDLE";
    case STATE_SET_LENGTH:    return "SET_LENGTH";
    case STATE_SET_QUANTITY:  return "SET_QUANTITY";
    case STATE_SET_STRIP_LEN: return "SET_STRIP";
    case STATE_CONFIRM:       return "CONFIRM";
    case STATE_WORKING:       return "WORKING";
    case STATE_PAUSED:        return "PAUSED";
    case STATE_DONE:          return "DONE";
    default:                  return "UNKNOWN";
  }
}

const char* getPhaseName() {
  switch (workPhase) {
    case PHASE_STRIP_HEAD: return "STRIP_HEAD";
    case PHASE_FEED_WIRE:  return "FEED_WIRE";
    case PHASE_STRIP_TAIL: return "STRIP_TAIL";
    case PHASE_CUT:        return "CUT";
    default:               return "UNKNOWN";
  }
}

void mqttPublishStatus() {
  if (!mqtt.connected()) return;
  
  StaticJsonDocument<512> doc;
  doc["state"] = getStateName();
  doc["phase"] = getPhaseName();
  doc["wire_present"] = isWirePresent();
  doc["queue_size"] = queueCount;
  doc["current_queue"] = currentQueueIdx;
  
  if (currentQueueIdx >= 0 && currentQueueIdx < queueCount) {
    JsonObject job = doc.createNestedObject("job");
    job["label"] = queue[currentQueueIdx].label;
    job["length_cm"] = queue[currentQueueIdx].lengthCm;
    job["quantity"] = queue[currentQueueIdx].quantity;
    job["strip_mm"] = queue[currentQueueIdx].stripMm;
    job["done"] = queue[currentQueueIdx].done;
  }
  
  doc["uptime"] = millis() / 1000;
  
  String json;
  serializeJson(doc, json);
  mqtt.publish(MQTT_TOPIC_STATUS, json.c_str());
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // รับคำสั่งจาก MQTT (optional)
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.printf("[MQTT] %s: %s\n", topic, msg.c_str());
}

void mqttReconnect() {
  if (mqtt.connected()) return;
  Serial.print("MQTT connecting...");
  if (mqtt.connect(MQTT_CLIENT)) {
    Serial.println("OK");
    mqtt.subscribe(MQTT_TOPIC_CMD);
  } else {
    Serial.printf("FAIL rc=%d\n", mqtt.state());
  }
}

// ==================================================================
//  WEBSOCKET (CLO4) - ระบบคิวตัดสายไฟ
// ==================================================================

void sendQueueToClient(uint8_t num) {
  // ส่งคิวทั้งหมดให้ client ที่เชื่อมต่อ
  StaticJsonDocument<1024> doc;
  doc["type"] = "queue_update";
  doc["state"] = getStateName();
  doc["current_idx"] = currentQueueIdx;
  
  JsonArray arr = doc.createNestedArray("queue");
  for (int i = 0; i < queueCount; i++) {
    JsonObject item = arr.createNestedObject();
    item["idx"] = i;
    item["label"] = queue[i].label;
    item["length_cm"] = queue[i].lengthCm;
    item["quantity"] = queue[i].quantity;
    item["strip_mm"] = queue[i].stripMm;
    item["done"] = queue[i].done;
    item["active"] = queue[i].active;
  }
  
  String json;
  serializeJson(doc, json);
  
  if (num == 255) {
    webSocket.broadcastTXT(json);
  } else {
    webSocket.sendTXT(num, json);
  }
}

void broadcastQueue() { sendQueueToClient(255); }

bool addToQueue(int lengthCm, int quantity, int stripMm, const char* label) {
  if (queueCount >= MAX_QUEUE) return false;
  
  QueueItem& item = queue[queueCount];
  item.lengthCm = constrain(lengthCm, 1, 999);
  item.quantity = constrain(quantity, 1, 999);
  item.stripMm = constrain(stripMm, 1, 30);
  item.done = 0;
  item.active = false;
  strncpy(item.label, label, 31);
  item.label[31] = '\0';
  
  queueCount++;
  Serial.printf("[Queue] Added: %s (%dcm x%d strip%dmm) [%d/%d]\n",
    label, lengthCm, quantity, stripMm, queueCount, MAX_QUEUE);
  return true;
}

void removeFromQueue(int idx) {
  if (idx < 0 || idx >= queueCount) return;
  if (queue[idx].active) return;  // ห้ามลบงานที่กำลังทำ
  
  for (int i = idx; i < queueCount - 1; i++) {
    queue[i] = queue[i + 1];
  }
  queueCount--;
  if (currentQueueIdx > idx) currentQueueIdx--;
  if (currentQueueIdx >= queueCount) currentQueueIdx = -1;
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[WS] Client #%u connected\n", num);
      sendQueueToClient(num);
      break;
      
    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client #%u disconnected\n", num);
      break;
      
    case WStype_TEXT: {
      StaticJsonDocument<256> doc;
      if (deserializeJson(doc, payload, length)) break;
      
      const char* cmd = doc["cmd"];
      if (!cmd) break;
      
      if (strcmp(cmd, "add") == 0) {
        int len = doc["length_cm"] | 10;
        int qty = doc["quantity"] | 1;
        int strip = doc["strip_mm"] | 8;
        const char* lbl = doc["label"] | "Job";
        addToQueue(len, qty, strip, lbl);
      }
      else if (strcmp(cmd, "remove") == 0) {
        int idx = doc["idx"] | -1;
        removeFromQueue(idx);
      }
      else if (strcmp(cmd, "start") == 0) {
        // เริ่มทำคิว
        if (currentState == STATE_IDLE && queueCount > 0) {
          startNextQueueItem();
        }
      }
      else if (strcmp(cmd, "stop") == 0) {
        currentState = STATE_IDLE;
        if (currentQueueIdx >= 0 && currentQueueIdx < queueCount) {
          queue[currentQueueIdx].active = false;
        }
        currentQueueIdx = -1;
      }
      else if (strcmp(cmd, "get_queue") == 0) {
        sendQueueToClient(num);
        break;  // ไม่ต้อง broadcast
      }
      
      broadcastQueue();
      lastActivityTime = millis();
      break;
    }
    default: break;
  }
}

void startNextQueueItem() {
  // หา queue item ถัดไปที่ยังไม่เสร็จ
  for (int i = 0; i < queueCount; i++) {
    if (queue[i].done < queue[i].quantity) {
      currentQueueIdx = i;
      queue[i].active = true;
      wireLengthCm = queue[i].lengthCm;
      wireQuantity = queue[i].quantity;
      stripLengthMm = queue[i].stripMm;
      wiresDone = queue[i].done;
      currentState = STATE_WORKING;
      Serial.printf("[Queue] Starting: %s\n", queue[i].label);
      return;
    }
  }
  // ไม่มีงานเหลือ
  currentState = STATE_DONE;
  currentQueueIdx = -1;
}

// ==================================================================
//  WORK SEQUENCE
// ==================================================================

void executeOneWire() {
  float stripMm = (float)stripLengthMm;
  float wireMm  = (float)wireLengthCm * 10.0;
  
  // Phase 1: บากฉนวนหัว
  workPhase = PHASE_STRIP_HEAD;
  updateWorkingDisplay();
  mqttPublishStatus();
  
  feedForward(stripMm);
  guideToStrip();
  cutterStrip();
  cutterOpen();
  
  // Phase 2: ป้อนสาย
  workPhase = PHASE_FEED_WIRE;
  updateWorkingDisplay();
  mqttPublishStatus();
  
  float totalFeedMm = wireMm + stripMm;
  if (!feedForwardChecked(totalFeedMm)) {
    currentState = STATE_PAUSED;
    cutterOpen();
    return;
  }
  
  // Phase 3: บากฉนวนท้าย
  workPhase = PHASE_STRIP_TAIL;
  updateWorkingDisplay();
  mqttPublishStatus();
  
  guideToStrip();
  cutterStrip();
  cutterOpen();
  
  // Phase 4: ตัด
  workPhase = PHASE_CUT;
  updateWorkingDisplay();
  mqttPublishStatus();
  
  guideToCut();
  cutterCut();
  cutterOpen();
  
  // เสร็จ 1 เส้น
  wiresDone++;
  if (currentQueueIdx >= 0 && currentQueueIdx < queueCount) {
    queue[currentQueueIdx].done = wiresDone;
  }
  lastActivityTime = millis();
  mqttPublishStatus();
  broadcastQueue();
}

void executeFullJob() {
  while (wiresDone < wireQuantity) {
    if (!isWirePresent()) {
      currentState = STATE_PAUSED;
      updateDisplay();
      mqttPublishStatus();
      broadcastQueue();
      return;
    }
    
    executeOneWire();
    if (currentState == STATE_PAUSED) {
      updateDisplay();
      return;
    }
    
    updateDisplay();
    
    // ให้ IoT ทำงาน
    webSocket.loop();
    mqtt.loop();
    
    delay(300);
  }
  
  // งานนี้เสร็จแล้ว
  if (currentQueueIdx >= 0 && currentQueueIdx < queueCount) {
    queue[currentQueueIdx].active = false;
  }
  
  // ลองทำ queue item ถัดไป
  startNextQueueItem();
  
  if (currentState != STATE_WORKING) {
    currentState = STATE_DONE;
  }
  
  updateDisplay();
  mqttPublishStatus();
  broadcastQueue();
}

// ==================================================================
//  TFT DISPLAY
// ==================================================================

void drawHeader(const char* title, uint16_t bg) {
  tft.fillRect(0, 0, 160, 22, bg);
  tft.setTextColor(C_TEXT); tft.setTextSize(1);
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((160 - w) / 2, 6);
  tft.print(title);
}

void drawProgressBar(int cur, int total, int y) {
  int bx = 10, bw = 140, bh = 14;
  tft.fillRoundRect(bx, y, bw, bh, 3, C_PROGRESS_BG);
  if (total > 0) {
    int fw = (int)((float)cur / total * (bw - 4));
    if (fw > 0) tft.fillRoundRect(bx + 2, y + 2, fw, bh - 4, 2, C_PROGRESS);
  }
  tft.setTextSize(1); tft.setTextColor(C_TEXT);
  char buf[16]; snprintf(buf, sizeof(buf), "%d / %d", cur, total);
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor(bx + (bw - w) / 2, y + 3);
  tft.print(buf);
}

void drawBigNum(int val, const char* unit, int y) {
  tft.fillRect(0, y, 160, 40, C_BG);
  char buf[16]; snprintf(buf, sizeof(buf), "%d", val);
  tft.setTextSize(3); tft.setTextColor(C_VALUE);
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  int tw = w + 6 * strlen(unit) + 4;
  tft.setCursor((160 - tw) / 2, y + 5);
  tft.print(buf);
  tft.setTextSize(1); tft.setTextColor(C_DIM);
  tft.setCursor((160 - tw) / 2 + w + 4, y + 18);
  tft.print(unit);
}

// --- Idle screen: show queue count ---
void drawIdleScreen() {
  tft.fillScreen(C_BG);
  drawHeader("WIRE CUTTER", C_HEADER_BG);
  
  tft.setTextSize(1); tft.setTextColor(C_DIM);
  tft.setCursor(10, 30);
  tft.print("Queue:");
  
  char buf[16]; snprintf(buf, sizeof(buf), "%d", queueCount);
  tft.setTextSize(3); tft.setTextColor(queueCount > 0 ? C_VALUE : C_DIM);
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((160 - w) / 2, 45);
  tft.print(buf);
  
  tft.setTextSize(1); tft.setTextColor(C_DIM);
  tft.setCursor(55, 78); tft.print("jobs");
  
  if (WiFi.status() == WL_CONNECTED) {
    tft.setTextColor(C_ACCENT);
    tft.setCursor(10, 95);
    tft.print(WiFi.localIP());
    tft.setCursor(10, 107);
    tft.print(":81 WebSocket");
  }
  
  tft.setTextColor(C_ACCENT);
  tft.setCursor(15, 118);
  if (queueCount > 0)
    tft.print("[OK] Start Queue");
  else
    tft.print("[OK] Manual Mode");
}

void drawSetLengthScreen() {
  tft.fillScreen(C_BG);
  drawHeader("WIRE LENGTH", C_HEADER_BG);
  tft.setTextSize(1); tft.setTextColor(C_DIM);
  tft.setCursor(15, 33); tft.print("[UP]+1  [DOWN]-1");
  tft.setTextColor(C_ACCENT);
  tft.setCursor(30, 115); tft.print("[OK] Next >");
  drawBigNum(wireLengthCm, "cm", 48);
  lastDrawnValue = wireLengthCm;
}

void drawSetQuantityScreen() {
  tft.fillScreen(C_BG);
  drawHeader("QUANTITY", C_HEADER_BG);
  tft.setTextSize(1); tft.setTextColor(C_DIM);
  char info[24]; snprintf(info, sizeof(info), "Length: %d cm", wireLengthCm);
  tft.setCursor(15, 26); tft.print(info);
  tft.setCursor(15, 36); tft.print("[UP]+1  [DOWN]-1");
  tft.setTextColor(C_ACCENT);
  tft.setCursor(30, 115); tft.print("[OK] Next >");
  drawBigNum(wireQuantity, "pcs", 50);
  lastDrawnValue = wireQuantity;
}

void drawSetStripScreen() {
  tft.fillScreen(C_BG);
  drawHeader("STRIP LENGTH", C_HEADER_BG);
  tft.setTextSize(1); tft.setTextColor(C_DIM);
  tft.setCursor(10, 26); tft.print("Strip each end");
  tft.setCursor(15, 36); tft.print("[UP]+1  [DOWN]-1");
  tft.setTextColor(C_ACCENT);
  tft.setCursor(30, 115); tft.print("[OK] Next >");
  drawBigNum(stripLengthMm, "mm", 50);
  lastDrawnValue = stripLengthMm;
}

void drawConfirmScreen() {
  tft.fillScreen(C_BG);
  drawHeader("CONFIRM", C_HEADER_BG);
  tft.setTextSize(1);
  int y = 30;
  tft.setTextColor(C_DIM); tft.setCursor(10, y); tft.print("Length:");
  tft.setTextColor(C_TEXT);
  char buf[24]; snprintf(buf, sizeof(buf), "%d cm", wireLengthCm);
  tft.setCursor(85, y); tft.print(buf);
  y += 14;
  tft.setTextColor(C_DIM); tft.setCursor(10, y); tft.print("Qty:");
  tft.setTextColor(C_TEXT);
  snprintf(buf, sizeof(buf), "%d pcs", wireQuantity);
  tft.setCursor(85, y); tft.print(buf);
  y += 14;
  tft.setTextColor(C_DIM); tft.setCursor(10, y); tft.print("Strip:");
  tft.setTextColor(C_TEXT);
  snprintf(buf, sizeof(buf), "%d mm", stripLengthMm);
  tft.setCursor(85, y); tft.print(buf);
  y += 20;
  tft.setTextColor(C_WARNING);
  tft.setCursor(10, y); tft.print("Insert wire & press OK");
  tft.setTextColor(C_ACCENT);
  tft.setCursor(22, 115); tft.print("[OK] Start!");
}

void drawWorkingScreen() {
  tft.fillScreen(C_BG);
  drawHeader("WORKING...", 0x0300);
  tft.setTextSize(1); tft.setTextColor(C_DIM);
  char buf[32];
  if (currentQueueIdx >= 0)
    snprintf(buf, sizeof(buf), "Q%d: %s", currentQueueIdx + 1, queue[currentQueueIdx].label);
  else
    snprintf(buf, sizeof(buf), "L:%dcm S:%dmm", wireLengthCm, stripLengthMm);
  tft.setCursor(10, 26); tft.print(buf);
  drawWorkingProgress();
  drawWorkPhase();
  drawProgressBar(wiresDone, wireQuantity, 96);
  lastDrawnDone = wiresDone;
  lastDrawnPhase = workPhase;
}

void drawWorkingProgress() {
  tft.fillRect(0, 38, 160, 30, C_BG);
  char buf[16]; snprintf(buf, sizeof(buf), "%d/%d", wiresDone, wireQuantity);
  tft.setTextSize(3); tft.setTextColor(C_ACCENT);
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((160 - w) / 2, 40);
  tft.print(buf);
}

void drawWorkPhase() {
  tft.fillRect(0, 72, 160, 18, C_BG);
  tft.setTextSize(1); tft.setTextColor(C_VALUE);
  const char* t;
  switch (workPhase) {
    case PHASE_STRIP_HEAD: t = ">> Strip Head";   break;
    case PHASE_FEED_WIRE:  t = ">> Feeding Wire"; break;
    case PHASE_STRIP_TAIL: t = ">> Strip Tail";   break;
    case PHASE_CUT:        t = ">> Cutting!";     break;
    default: t = ">> ..."; break;
  }
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(t, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((160 - w) / 2, 75);
  tft.print(t);
}

void updateWorkingDisplay() {
  if (wiresDone != lastDrawnDone) {
    drawWorkingProgress();
    drawProgressBar(wiresDone, wireQuantity, 96);
    lastDrawnDone = wiresDone;
  }
  if (workPhase != lastDrawnPhase) {
    drawWorkPhase();
    lastDrawnPhase = workPhase;
  }
}

void drawPausedScreen() {
  tft.fillScreen(C_BG);
  drawHeader("!! NO WIRE !!", C_ERROR);
  tft.setTextSize(2); tft.setTextColor(C_WARNING);
  tft.setCursor(5, 40); tft.print("Wire Empty!");
  tft.setTextSize(1); tft.setTextColor(C_TEXT);
  tft.setCursor(10, 68); tft.print("Reload wire spool");
  tft.setCursor(10, 80); tft.print("then press OK");
  tft.setTextColor(C_ACCENT);
  tft.setCursor(25, 115); tft.print("[OK] Resume");
}

void drawDoneScreen() {
  tft.fillScreen(C_BG);
  drawHeader("COMPLETE!", 0x0320);
  tft.setTextSize(1); tft.setTextColor(C_DIM);
  tft.setCursor(10, 30); tft.print("All jobs done!");
  tft.setTextSize(3); tft.setTextColor(C_ACCENT);
  char buf[8]; snprintf(buf, sizeof(buf), "OK");
  tft.setCursor(50, 50); tft.print(buf);
  tft.setTextSize(1); tft.setTextColor(C_ACCENT);
  tft.setCursor(25, 115); tft.print("[OK] Back to Idle");
}

void updateDisplay() {
  if (currentState != lastDrawnState) {
    lastDrawnState = currentState;
    lastDrawnValue = -1; lastDrawnDone = -1;
    lastDrawnPhase = (WorkPhase)-1;
    switch (currentState) {
      case STATE_IDLE:          drawIdleScreen();       break;
      case STATE_SET_LENGTH:    drawSetLengthScreen();  break;
      case STATE_SET_QUANTITY:  drawSetQuantityScreen(); break;
      case STATE_SET_STRIP_LEN: drawSetStripScreen();   break;
      case STATE_CONFIRM:       drawConfirmScreen();    break;
      case STATE_WORKING:       drawWorkingScreen();    break;
      case STATE_PAUSED:        drawPausedScreen();     break;
      case STATE_DONE:          drawDoneScreen();       break;
    }
    return;
  }
  switch (currentState) {
    case STATE_SET_LENGTH:
      if (wireLengthCm != lastDrawnValue) { drawBigNum(wireLengthCm, "cm", 48); lastDrawnValue = wireLengthCm; }
      break;
    case STATE_SET_QUANTITY:
      if (wireQuantity != lastDrawnValue) { drawBigNum(wireQuantity, "pcs", 50); lastDrawnValue = wireQuantity; }
      break;
    case STATE_SET_STRIP_LEN:
      if (stripLengthMm != lastDrawnValue) { drawBigNum(stripLengthMm, "mm", 50); lastDrawnValue = stripLengthMm; }
      break;
    case STATE_WORKING:
      updateWorkingDisplay();
      break;
    default: break;
  }
}

// ==================================================================
//  BUTTON HANDLING
// ==================================================================

void handleButtons() {
  if (btnUpFlag) {
    btnUpFlag = false; lastActivityTime = millis();
    switch (currentState) {
      case STATE_SET_LENGTH:     wireLengthCm  = min(wireLengthCm + 1, 999); break;
      case STATE_SET_QUANTITY:   wireQuantity   = min(wireQuantity + 1, 999); break;
      case STATE_SET_STRIP_LEN:  stripLengthMm = min(stripLengthMm + 1, 30); break;
      default: break;
    }
    updateDisplay();
  }
  
  if (btnDownFlag) {
    btnDownFlag = false; lastActivityTime = millis();
    switch (currentState) {
      case STATE_SET_LENGTH:     wireLengthCm  = max(wireLengthCm - 1, 1); break;
      case STATE_SET_QUANTITY:   wireQuantity   = max(wireQuantity - 1, 1); break;
      case STATE_SET_STRIP_LEN:  stripLengthMm = max(stripLengthMm - 1, 1); break;
      case STATE_CONFIRM:        currentState = STATE_SET_STRIP_LEN; break;
      default: break;
    }
    updateDisplay();
  }
  
  if (btnOkFlag) {
    btnOkFlag = false; lastActivityTime = millis();
    switch (currentState) {
      case STATE_IDLE:
        if (queueCount > 0) {
          startNextQueueItem();
        } else {
          currentState = STATE_SET_LENGTH;
        }
        break;
      case STATE_SET_LENGTH:    currentState = STATE_SET_QUANTITY; break;
      case STATE_SET_QUANTITY:  currentState = STATE_SET_STRIP_LEN; break;
      case STATE_SET_STRIP_LEN: currentState = STATE_CONFIRM; break;
      case STATE_CONFIRM:
        wiresDone = 0;
        currentQueueIdx = -1;
        currentState = STATE_WORKING;
        break;
      case STATE_PAUSED:
        if (isWirePresent()) currentState = STATE_WORKING;
        break;
      case STATE_DONE:
        currentState = STATE_IDLE;
        break;
      default: break;
    }
    updateDisplay();
    mqttPublishStatus();
    broadcastQueue();
  }
}

// ==================================================================
//  DEEP SLEEP (CLO1)
// ==================================================================

void enterDeepSleep() {
  tft.fillScreen(C_BG);
  tft.setTextSize(1); tft.setTextColor(C_DIM);
  tft.setCursor(20, 50); tft.print("Sleeping...");
  tft.setTextColor(C_ACCENT);
  tft.setCursor(10, 70); tft.print("Press OK to wake");
  delay(1500);
  
  cutterServo.detach();
  guideServo.detach();
  mqtt.disconnect();
  webSocket.close();
  WiFi.disconnect(true);
  
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BTN_OK_PIN, 0);
  esp_deep_sleep_start();
}

// ==================================================================
//  SETUP
// ==================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Wire Cutter & Stripper + IoT ===");
  
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0)
    Serial.println("Woke up from Deep Sleep");
  
  // GPIO
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_PUL_PIN, OUTPUT);
  pinMode(SENSOR1_PIN, INPUT);
  pinMode(SENSOR2_PIN, INPUT);
  pinMode(BTN_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_OK_PIN, INPUT_PULLUP);
  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
  
  // Interrupts
  attachInterrupt(digitalPinToInterrupt(BTN_UP_PIN), isrUp, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_OK_PIN), isrOk, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_DOWN_PIN), isrDown, FALLING);
  
  // Servos
  cutterServo.attach(SERVO_CUTTER_PIN);
  cutterServo.write(CUTTER_OPEN_ANGLE);
  guideServo.attach(SERVO_GUIDE_PIN);
  guideServo.write(GUIDE_STRIP_ANGLE);
  
  // TFT
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(C_BG);
  tft.setTextColor(C_TITLE); tft.setTextSize(1);
  tft.setCursor(15, 30); tft.print("WIRE CUTTER");
  tft.setCursor(15, 45); tft.print("& STRIPPER");
  tft.setTextColor(C_DIM);
  tft.setCursor(15, 70); tft.print("Connecting...");
  
  // WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500); Serial.print("."); tft.print(".");
    timeout++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi OK - IP: %s\n", WiFi.localIP().toString().c_str());
    tft.fillRect(0, 65, 160, 40, C_BG);
    tft.setTextColor(C_ACCENT);
    tft.setCursor(15, 70); tft.print("WiFi OK!");
    tft.setCursor(15, 85); tft.print(WiFi.localIP());
  } else {
    Serial.println("\nWiFi FAIL");
    tft.setTextColor(C_WARNING);
    tft.setCursor(15, 85); tft.print("Offline mode");
  }
  delay(1000);
  
  // MQTT
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqttReconnect();
  
  // SPIFFS - เก็บ frontend (data/index.html)
  if (SPIFFS.begin(true)) {
    Serial.println("SPIFFS mounted");
  } else {
    Serial.println("SPIFFS mount failed!");
  }
  
  // HTTP Server - serve dashboard จาก SPIFFS
  httpServer.on("/", HTTP_GET, []() {
    File file = SPIFFS.open("/index.html", "r");
    if (file) {
      httpServer.streamFile(file, "text/html");
      file.close();
    } else {
      httpServer.send(200, "text/html",
        "<html><body style='background:#0a0e17;color:#e2e8f0;font-family:monospace;padding:40px;'>"
        "<h2 style='color:#f59e0b;'>Wire Cutter Dashboard</h2>"
        "<p>SPIFFS not uploaded. Run: <code>pio run -t uploadfs</code></p>"
        "<p>WebSocket port: 81</p></body></html>");
    }
  });
  httpServer.begin();
  Serial.println("HTTP Server started on port 80");
  
  // WebSocket
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  // Init
  currentState = STATE_IDLE;
  lastDrawnState = (SystemState)-1;
  lastActivityTime = millis();
  updateDisplay();
  
  Serial.println("System Ready!");
  Serial.printf("WebSocket: ws://%s:81\n", WiFi.localIP().toString().c_str());
}

// ==================================================================
//  LOOP
// ==================================================================

void loop() {
  // IoT
  webSocket.loop();
  httpServer.handleClient();
  if (!mqtt.connected()) mqttReconnect();
  mqtt.loop();
  
  // Buttons
  handleButtons();
  
  // Work
  if (currentState == STATE_WORKING) {
    executeFullJob();
  }
  
  // MQTT publish ทุก 2 วินาที
  if (millis() - lastMqttPublish > 2000) {
    mqttPublishStatus();
    lastMqttPublish = millis();
  }
  
  // Deep Sleep
  if (currentState == STATE_IDLE &&
      (millis() - lastActivityTime > SLEEP_TIMEOUT_MS)) {
    enterDeepSleep();
  }
  
  delay(10);
}
