/*
 * 專案：3x MPU-6050 IMU 讀取器 (使用雙 I2C 匯流排) - V6 錯字修正版
 * 函式庫: Adafruit_MPU6050, Adafruit_Unified_Sensor, Adafruit_BusIO
 * * 接線: (保持不變)
 * I2C 匯流排 #1 (Wire @ GPIO 21, 22)
 * - IMU 1: SCL=22, SDA=21, AD0=GND (位址 0x68)
 * - IMU 2: SCL=22, SDA=21, AD0=VCC (位址 0x69)
 *
 * I2C 匯流排 #2 (Wire1 @ GPIO 18, 19)
 * - IMU 3: SCL=19, SDA=18, AD0=GND (位址 0x68)
 */

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- 宣告 3 個 MPU 物件 ---
Adafruit_MPU6050 mpu1;
Adafruit_MPU6050 mpu2;
Adafruit_MPU6050 mpu3;

// (ESP32 函式庫已定義 'Wire1', 我們不需重複定義)

// --- 互補濾波器 變數 ---
float pitch1 = 0, roll1 = 0;
float pitch2 = 0, roll2 = 0;
float pitch3 = 0, roll3 = 0;
unsigned long timer;
float dt; // 時間差

// 一個輔助函式，用來初始化 IMU
void setupMPU(Adafruit_MPU6050 &mpu, const char* name) {
  Serial.printf("正在初始化 %s ... ", name);
  // (設定範圍)
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  
  // *********** 錯誤修正 V6 ***********
  // 'MPU650_BAND_21_HZ' -> 'MPU6050_BAND_21_HZ'
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); 
  // ********************************
  
  Serial.println("完成.");
}

// 一個輔助函式，用來計算角度
void updateAngles(Adafruit_MPU6050 &mpu, float &pitch, float &roll) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); // 讀取原始數據

  // --- 互補濾波器數學 ---
  float acc_pitch = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / M_PI;
  float acc_roll = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / M_PI;

  float gyro_pitch = pitch + g.gyro.y * dt;
  float gyro_roll = roll + g.gyro.x * dt;

  pitch = 0.98 * gyro_pitch + 0.02 * acc_pitch;
  roll = 0.98 * gyro_roll + 0.02 * acc_roll;
}


void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); } 

  // --- 初始化 I2C #1 (SDA=21, SCL=22) ---
  Wire.begin(21, 22);
  Wire.setClock(400000);

  // --- 初始化 I2C #2 (SDA=18, SCL=19) ---
  Wire1.begin(18, 19);
  Wire1.setClock(400000);

  Serial.println("Adafruit MPU6050 3顆 IMU 測試 (V6)");

  // --- 啟動 IMU 1 ---
  if (!mpu1.begin(0x68, &Wire)) {
    Serial.println("錯誤: 找不到 IMU 1 (位址 0x68 @ Wire)");
    while (1) { delay(10); }
  }
  setupMPU(mpu1, "IMU 1");

  // --- 啟動 IMU 2 ---
  if (!mpu2.begin(0x69, &Wire)) {
    Serial.println("錯誤: 找不到 IMU 2 (位址 0x69 @ Wire)");
    while (1) { delay(10); }
  }
  setupMPU(mpu2, "IMU 2");

  // --- 啟動 IMU 3 ---
  if (!mpu3.begin(0x68, &Wire1)) { 
    Serial.println("錯誤: 找不到 IMU 3 (位址 0x68 @ Wire1)");
    while (1) { delay(10); }
  }
  setupMPU(mpu3, "IMU 3");

  Serial.println("所有 IMU 準備就緒！");
  timer = millis(); // 初始化計時器
}

void loop() {
  // --- 計算時間差 (dt) ---
  dt = (float)(millis() - timer) / 1000.0;
  timer = millis();

  // --- 更新 3 顆 IMU 的角度 ---
  updateAngles(mpu1, pitch1, roll1);
  updateAngles(mpu2, pitch2, roll2);
  updateAngles(mpu3, pitch3, roll3);

  /*
   * 傳送資料到電腦
   * 格式: Roll_1,Pitch_1,Roll_2,Pitch_2,Roll_3,Pitch_3
   */

  // IMU 1
  Serial.print(roll1);
  Serial.print(",");
  Serial.print(pitch1);
  Serial.print(",");

  // IMU 2
  Serial.print(roll2);
  Serial.print(",");
  Serial.print(pitch2);
  Serial.print(",");

  // IMU 3
  Serial.print(roll3);
  Serial.print(",");
  Serial.println(pitch3); // 結尾換行

  delay(10); // 控制更新率
}