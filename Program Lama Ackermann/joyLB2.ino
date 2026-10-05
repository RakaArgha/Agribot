# joyLB2.ino

// joyLB2.ino
#include <SPI.h>
#include <due_can.h>

// --- 1. 左後輪 (LR) 硬體引腳定義 (左板配置) ---
static const uint8_t PIN_CS         = 27; // 左板 CS
static const uint8_t PIN_MOTOR_EN   = 24; // 左板 EN
static const uint8_t PIN_MOTOR_PWM1 = 4;  // 左板 PWM1
static const uint8_t PIN_MOTOR_PWM2 = 5;  // 左板 PWM2

// --- 2. 全域變數 ---
float lr_targetAngle = 180.0; 
float lr_currentAngle = 0.0;
uint8_t motor_switch = 0; 
const uint32_t CAN_ID = 0x123;

// --- 3. 控制參數 (強力版更新) ---
float Kp = 180.0;     // 提高比例增益 (原本 80)
float Ki = 0.8;       // 積分增益：解決靜摩擦力卡死問題
float integral = 0;   // 積分累加器
float deadzone = 0.2; // 死區範圍
int minPWM = 1000;    // 克服 15 度推不動的關鍵 (建議 800-1200)

// --- 4. AS5047P SPI 讀取邏輯 ---
static SPISettings AS5047P_SPI(1000000, MSBFIRST, SPI_MODE1);

static inline uint8_t evenParity16(uint16_t v) {
  v ^= v >> 8; v ^= v >> 4; v ^= v >> 2; v ^= v >> 1;
  return v & 1;
}

uint16_t readEncoderRaw() {
  uint16_t cmd = 0x3FFF | 0x4000; 
  cmd |= (uint16_t(evenParity16(cmd)) << 15);
  SPI.beginTransaction(AS5047P_SPI);
  digitalWrite(PIN_CS, LOW);
  SPI.transfer((cmd >> 8) & 0xFF); SPI.transfer(cmd & 0xFF);
  digitalWrite(PIN_CS, HIGH);
  delayMicroseconds(1);
  digitalWrite(PIN_CS, LOW);
  uint8_t hi = SPI.transfer(0x00); uint8_t lo = SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();
  return (uint16_t(hi) << 8) | lo;
}

float getAngle() {
  uint16_t val = readEncoderRaw();
  if (val & 0x4000) { readEncoderRaw(); } 
  return ((val & 0x3FFF) * 360.0f) / 16384.0f;
}

// --- 5. 強力馬達驅動邏輯 (LR 專用) ---
void driveLR(float output) {
  float absOut = abs(output);

  // 1. 停止邏輯：誤差極小時才真正斷開動力 (Coast)
  if (absOut < 20) {
    digitalWrite(PIN_MOTOR_EN, HIGH); 
    analogWrite(PIN_MOTOR_PWM1, 4095);
    digitalWrite(PIN_MOTOR_PWM2, HIGH);
    return;
  }

  digitalWrite(PIN_MOTOR_EN, LOW); // 使能啟動

  // 2. 核心改進：保底 PWM + 比例輸出
  int pwmVal = minPWM + constrain((int)absOut, 0, 4095 - minPWM);

  if (output < 0) { 
    analogWrite(PIN_MOTOR_PWM1, 4095 - pwmVal);
    digitalWrite(PIN_MOTOR_PWM2, HIGH);
  } else {
    digitalWrite(PIN_MOTOR_PWM1, HIGH);
    analogWrite(PIN_MOTOR_PWM2, 4095 - pwmVal);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_CS, OUTPUT); digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_MOTOR_EN, OUTPUT); digitalWrite(PIN_MOTOR_EN, HIGH); 
  pinMode(PIN_MOTOR_PWM1, OUTPUT); pinMode(PIN_MOTOR_PWM2, OUTPUT);
  
  analogWriteResolution(12); // Due 專屬 12-bit
  SPI.begin();
  if (Can0.begin(CAN_BPS_250K)) {
    Can0.watchFor(CAN_ID);
    Serial.println("LR Powerful System Ready - Mirror + Inverted Mode");
  }
}

void loop() {
  // --- A. 接收 CAN 數據 ---
  CAN_FRAME frame;
  if (Can0.available()) {
    Can0.read(frame);
    if (frame.id == CAN_ID && frame.length >= 8) {
      // 1. 取得左前輪 (LF) 的目標角作為參考 (Byte 0, 1)
      uint16_t lf_raw = (frame.data.bytes[0] << 8) | frame.data.bytes[1];
      float lf_target = lf_raw / 10.0;

      // 2. 計算左後輪目標角 (鏡像轉換)
      lr_targetAngle = 360.0 - lf_target;

      // 3. 取得電源狀態 (Byte 7)
      motor_switch = frame.data.bytes[7];
    }
  }

  // --- B. 執行控制邏輯 ---
  if (motor_switch == 0x01) {
    lr_currentAngle = getAngle();
    
    // 4. 【維持你的核心修改】反轉旋轉方向：使用 Current - Target
    float error = lr_currentAngle - lr_targetAngle; 
    
    if (abs(error) > deadzone) {
      // 積分累加：解決 15 度推不動的困境
      integral += error;
      integral = constrain(integral, -1500, 1500); // 防止過度衝出 (Windup)
      
      // 計算強力輸出：P項 + I項
      driveLR(error * Kp + integral * Ki);
    } else {
      integral = 0; // 到位後清空積分
      driveLR(0);
    }
  } 
  else {
    // 關閉狀態
    integral = 0;
    digitalWrite(PIN_MOTOR_EN, HIGH);
    analogWrite(PIN_MOTOR_PWM1, 4095);
    digitalWrite(PIN_MOTOR_PWM2, HIGH);
  }

  // --- C. Debug 監控 ---
  static unsigned long lastT = 0;
  if (millis() - lastT > 100) {
    Serial.print(motor_switch ? "[ON] " : "[OFF] ");
    Serial.print("LR Target: "); Serial.print(lr_targetAngle, 1);
    Serial.print(" | Actual: "); Serial.print(lr_currentAngle, 1);
    Serial.print(" | Error: "); Serial.println(lr_currentAngle - lr_targetAngle, 1);
    lastT = millis();
  }
}
