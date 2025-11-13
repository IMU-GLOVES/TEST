/*
 * 專案：3x MPU-6050 IMU 讀取器 - V13 (官方 Adafruit 修正版)
 * 函式庫: (不再需要 Adafruit_TCA9548A.h)
 * * * * V13 接線 (同 V12) * * *
 * ESP32 P21(SDA), P22(SCL) -> TCA9548A 的 SDA, SCL (位址 0x70)
 * IMU 1 (AD0=GND) -> TCA9548A 的 Channel 0 (SD0, SC0)
 * IMU 2 (AD0=GND) -> TCA9548A 的 Channel 1 (SD1, SC1)
 * IMU 3 (AD0=GND) -> TCA9548A 的 Channel 2 (SD2, SC2)
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// *********** V13 修正 ***********
// 我們不再需要 <Adafruit_TCA9548A.h> 函式庫
// #include <Adafruit_TCA9548A.h> // <--- 刪除

#define TCA_ADDR 0x70 // TCA9548A 多工器的 I2C 位址
// ********************************

// --- 建立 3 個 MPU 物件 ---
Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;
Adafruit_MPU6050 mpu3;

float pitch1 = 0, roll1 = 0;
float pitch2 = 0, roll2 = 0;
float pitch3 = 0, roll3 = 0;
unsigned long timer;
float dt; 

// --- 輔助函式 (保持不變) ---
void setupMPU(Adafruit_MPU6050 &mpu, const char* name) {
  Serial.printf("正在初始化 %s ... ", name);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); 
  Serial.println("完成.");
}

void updateAngles(Adafruit_MPU6050 &mpu, float &pitch, float &roll) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); 
  float acc_pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / M_PI;
  float acc_roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / M_PI;
  float gyro_pitch = pitch + g.gyro.y * dt;
  float gyro_roll = roll + g.gyro.x * dt;
  pitch = 0.98 * gyro_pitch + 0.02 * acc_pitch;
  roll = 0.98 * gyro_roll + 0.02 * acc_roll;
}

// *********** V13 修正 ***********
// 這是 Adafruit 官方推薦的輔助函式
// 它手動告訴多工器 (在 0x70 位址) 要切換到哪個通道
void tca_select(uint8_t channel) {
  if (channel > 7) return;
  
  Wire.beginTransmission(TCA_ADDR);  // 開始與多工器 (0x70) 通訊
  Wire.write(1 << channel);          // 發送一個位元組來選擇通道 (例如, 通道 0 是 00000001)
  Wire.endTransmission();            // 結束通訊
}
// ********************************

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); } 

  // 初始化 I2C #1 (P21, P22)
  Wire.begin(21, 22);
  Wire.setClock(400000); 

  Serial.println("Adafruit MPU6050 3顆 IMU 測試 (V13 - 官方修正版)");

  // --- 檢查多工器 "總機" 是否在線上 ---
  Wire.beginTransmission(TCA_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("錯誤: 找不到 TCA9548A 多工器 (位址 0x70)");
    while (1);
  }
  Serial.println("TCA9548A 多工器找到！");
  delay(100);

  // --- 啟動 IMU 1 (在通道 0) ---
  tca_select(0); // 切換到通道 0
  if (!mpu1.begin(0x68, &Wire)) { 
    Serial.println("錯誤: 找不到 IMU 1 (位址 0x68 @ TCA Channel 0)");
    while (1) { delay(10); }
  }
  setupMPU(mpu1, "IMU 1");
  delay(100); 

  // --- 啟動 IMU 2 (在通道 1) ---
  tca_select(1); // 切換到通道 1
  if (!mpu2.begin(0x68, &Wire)) { 
    Serial.println("錯誤: 找不到 IMU 2 (位址 0x68 @ TCA Channel 1)");
    while (1) { delay(10); }
  }
  setupMPU(mpu2, "IMU 2");
  delay(100); 

  // --- 啟動 IMU 3 (在通道 2) ---
  tca_select(2); // 切換到通道 2
  if (!mpu3.begin(0x68, &Wire)) { 
    Serial.println("錯誤: 找不到 IMU 3 (位址 0x68 @ TCA Channel 2)");
    while (1) { delay(10); }
  }
  setupMPU(mpu3, "IMU 3");
  delay(100);

  Serial.println("所有 IMU 準備就緒！");
  timer = millis(); 
}

void loop() {
  dt = (float)(millis() - timer) / 1000.0;
  timer = millis();

  tca_select(0); // 切換到通道 0
  updateAngles(mpu1, pitch1, roll1);

  tca_select(1); // 切換到通道 1
  updateAngles(mpu2, pitch2, roll2);

  tca_select(2); // 切換到通道 2
  updateAngles(mpu3, pitch3, roll3);

  Serial.print(roll1); Serial.print(",");
  Serial.print(pitch1); Serial.print(",");
  Serial.print(roll2); Serial.print(",");
  Serial.print(pitch2); Serial.print(",");
  Serial.print(roll3); Serial.print(",");
  Serial.println(pitch3);

  delay(10); 
}