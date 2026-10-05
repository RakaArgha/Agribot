// joyLF2.ino
#include <SPI.h>
#include <due_can.h>

// --- 1. 硬體引腳定義 ---
static const uint8_t PIN_CS         = 25; // AS5047P CS (左板25、右板27)
static const uint8_t PIN_MOTOR_EN   = 26; // MTD-450 EN (左板26、右板24)
static const uint8_t PIN_MOTOR_PWM1 = 6;  // MTD-450 PWM1 (左板 6、右板 4)
static const uint8_t PIN_MOTOR_PWM2 = 7;  // MTD-450 PWM2 (左板 7、右板 5)

// --- 2. 全域變數 ---
float lf_targetAngle = 180.0; 
float lf_currentAngle = 0.0;
uint8_t motor_switch = 0; 
const uint32_t CAN_ID = 0x123;

// --- 3. 控制參數 (強化版) ---
float Kp = 180.0;    // 比例增益
float Ki = 0.8;      // 積分增益：當誤差持續存在時，會隨時間加強推力
float integral = 0;  // 積分累加器
float deadzone = 0.2; // 容許誤差範圍 (度)
int minPWM = 1000;   // 關鍵補償：只要馬達需要動，最少給多少 PWM (建議值 800-1200)

// --- 4. AS5047P SPI 讀取邏輯 ---
static SPISettings AS5047P_SPI(1000000, MSBFIRST, SPI_MODE1);//(1MHz)

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
  if (val & 0x4000) readEncoderRaw(); 
  return ((val & 0x3FFF) * 360.0f) / 16384.0f;
}

// --- 5. 強力馬達驅動邏輯 (修正 15 度推不動問題) ---
void driveLF(float output) {
  float absOut = abs(output);

  // 1. 到位或停止判斷：當輸出非常微小時，才真正放開馬達
  if (absOut < 20) {
    digitalWrite(PIN_MOTOR_EN, HIGH); 
    analogWrite(PIN_MOTOR_PWM1, 4095);
    digitalWrite(PIN_MOTOR_PWM2, HIGH);
    return;
  }

  digitalWrite(PIN_MOTOR_EN, LOW); // 啟動使能

  // 2. 加入保底推力：pwmVal = minPWM + (剩餘空間的比例輸出)
  // 這樣保證只要馬達一動，最少就有 minPWM 的力道
  int pwmVal = minPWM + constrain((int)absOut, 0, 4095 - minPWM);

  if (output < 0) { // 方向 1
    analogWrite(PIN_MOTOR_PWM1, 4095 - pwmVal);
    digitalWrite(PIN_MOTOR_PWM2, HIGH);
  } else { // 方向 2
    digitalWrite(PIN_MOTOR_PWM1, HIGH);
    analogWrite(PIN_MOTOR_PWM2, 4095 - pwmVal);
  }
}

void setup() {
  Serial.begin(115200);//跟電腦的BR
  pinMode(PIN_CS, OUTPUT); digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_MOTOR_EN, OUTPUT); digitalWrite(PIN_MOTOR_EN, HIGH);
  pinMode(PIN_MOTOR_PWM1, OUTPUT); pinMode(PIN_MOTOR_PWM2, OUTPUT);
  
  analogWriteResolution(12); // Due 12-bit PWM (0-4095)
  SPI.begin();
  if (Can0.begin(CAN_BPS_250K)) {
    Can0.watchFor(CAN_ID);
    Serial.println("Powerful System Ready - MinPWM Active");
  }
}

void loop() {
  // --- A. 接收 CAN 數據 ---
  CAN_FRAME frame;
  if (Can0.available()) {
    Can0.read(frame);
    if (frame.id == CAN_ID && frame.length >= 8) {
      uint16_t lf_raw = (frame.data.bytes[0] << 8) | frame.data.bytes[1];
      lf_targetAngle = lf_raw / 10.0;
      motor_switch = frame.data.bytes[7];
    }
  }

  // --- B. 執行控制邏輯 ---
  if (motor_switch == 0x01) {
    lf_currentAngle = getAngle();
    float error = lf_targetAngle - lf_currentAngle;
    
    // 判斷是否超出容許死區
    if (abs(error) > deadzone) {
      // 積分累加：如果馬達卡住不動，integral 會隨時間變大，推力就變強
      integral += error;
      // 限制積分上限，防止衝過頭 (Integral Windup)
      integral = constrain(integral, -1500, 1500); 
      
      // 計算總輸出：P項 + I項
      driveLF(error * Kp + integral * Ki);
    } else {
      // 到位：清空積分並停止
      integral = 0;
      driveLF(0); 
    }
  } 
  else {
    // 總電源關閉：強制 Coast
    integral = 0;
    digitalWrite(PIN_MOTOR_EN, HIGH);
    analogWrite(PIN_MOTOR_PWM1, 4095);
    digitalWrite(PIN_MOTOR_PWM2, HIGH);
  }

  // --- C. Debug 監控 ---
  static unsigned long lastT = 0;
  if (millis() - lastT > 100) {
    Serial.print(motor_switch ? "[ON] " : "[OFF] ");
    Serial.print("Target: "); Serial.print(lf_targetAngle, 1);
    Serial.print(" | Actual: "); Serial.print(lf_currentAngle, 1);
    Serial.print(" | Error: "); Serial.println(lf_targetAngle - lf_currentAngle, 1);
    lastT = millis();
  }
}
