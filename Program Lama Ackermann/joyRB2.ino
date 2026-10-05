// joyRB2.ino
#include <SPI.h>
#include <due_can.h>

// --- 1. 右後輪 (RR) 硬體引腳定義 (右板配置) ---
static const uint8_t PIN_CS         = 27; 
static const uint8_t PIN_MOTOR_EN   = 24; 
static const uint8_t PIN_MOTOR_PWM1 = 4;  
static const uint8_t PIN_MOTOR_PWM2 = 5;  

// --- 2. 全域變數 ---
float rr_targetAngle = 180.0; 
float rr_currentAngle = 0.0;
uint8_t motor_switch = 0; 
const uint32_t CAN_ID = 0x123;

// --- 3. 控制參數 (強化版更新) ---
float Kp = 180.0;     // 提高比例增益 (原本 80)
float Ki = 0.8;       // 積分增益：解決靜摩擦力卡死問題
float integral = 0;   // 積分累加器
float deadzone = 0.2; // 死區範圍
int minPWM = 1000;    // 關鍵：克服 15 度推不動的保底 PWM (建議範圍 800-1200)

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
  if (val & 0x4000) { readEncoderRaw(); } // 清除 Error Flag
  return ((val & 0x3FFF) * 360.0f) / 16384.0f;
}

// --- 5. 強力馬達驅動邏輯 (RR 專用) ---
void driveRR(float output) {
  float absOut = abs(output);

  // 1. 停止邏輯：誤差極小時才真正斷開動力 (Coast)
  if (absOut < 20) {
    digitalWrite(PIN_MOTOR_EN, HIGH); 
    analogWrite(PIN_MOTOR_PWM1, 4095); // 讓兩端沒壓差
    digitalWrite(PIN_MOTOR_PWM2, HIGH);
    return;
  }

  digitalWrite(PIN_MOTOR_EN, LOW); // 使能啟動

  // 2. 核心改進：只要馬達要動，最少就給 minPWM 的力道
  // pwmVal = minPWM + (比例計算後剩下的動力)
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
  
  analogWriteResolution(12); // Due 專屬 12-bit 解析度
  SPI.begin();
  if (Can0.begin(CAN_BPS_250K)) {
    Can0.watchFor(CAN_ID);
    Serial.println("RR Powerful System Ready - Ackermann Mirror Active");
  }
}

void loop() {
  // --- A. 接收 CAN 數據 ---
  CAN_FRAME frame;
  if (Can0.available()) {
    Can0.read(frame);
    if (frame.id == CAN_ID && frame.length >= 8) {
      // 1. 取得右前輪 (RF) 的目標角度作為鏡像參考
      uint16_t rf_raw = (frame.data.bytes[2] << 8) | frame.data.bytes[3];
      float rf_target = rf_raw / 10.0;

      // 2. 計算右後輪目標角 (Ackermann 鏡像邏輯: 360 - RF_target)
      rr_targetAngle = 360.0 - rf_target;

      // 3. 取得電源狀態 (Byte 7)
      motor_switch = frame.data.bytes[7];
    }
  }

  // --- B. 執行控制邏輯 ---
  if (motor_switch == 0x01) {
    rr_currentAngle = getAngle();
    float error = rr_targetAngle - rr_currentAngle; 
    
    if (abs(error) > deadzone) {
      // 積分累加：如果誤差一直存在 (如卡住)，力量會隨時間累積變強
      integral += error;
      // 限制積分上限，防止反應過激 (Windup)
      integral = constrain(integral, -1500, 1500); 
      
      // 計算強力輸出：P項 + I項
      driveRR(error * Kp + integral * Ki);
    } else {
      integral = 0; // 到位後清空積分
      driveRR(0);
    }
  } 
  else {
    // 關閉狀態：清空積分並斷電
    integral = 0;
    digitalWrite(PIN_MOTOR_EN, HIGH);
    analogWrite(PIN_MOTOR_PWM1, 4095);
    digitalWrite(PIN_MOTOR_PWM2, HIGH);
  }

  // --- C. Debug 監控 ---
  static unsigned long lastT = 0;
  if (millis() - lastT > 100) {
    Serial.print(motor_switch ? "[ON] " : "[OFF] ");
    Serial.print("Target: "); Serial.print(rr_targetAngle, 1);
    Serial.print(" | Actual: "); Serial.print(rr_currentAngle, 1);
    Serial.print(" | Error: "); Serial.println(rr_targetAngle - rr_currentAngle, 1);
    lastT = millis();
  }
}
