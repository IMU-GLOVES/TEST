#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define TCA_ADDR 0x70  // 多工器位址

// 3 顆 IMU
Adafruit_MPU6050 imu1, imu2, imu3;

// ------------------------
// 多工器選擇通道
// ------------------------
void tca_select(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// ------------------------
// 初始化一顆 IMU
// ------------------------
bool initIMU(Adafruit_MPU6050 &imu, int channel, const char *name) {
  tca_select(channel);
  delay(5);

  if (!imu.begin(0x68, &Wire)) {
    Serial.printf("❌ %s 初始化失敗（通道 %d）\n", name, channel);
    return false;
  }

  imu.setAccelerometerRange(MPU6050_RANGE_8_G);
  imu.setGyroRange(MPU6050_RANGE_500_DEG);
  imu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.printf("✔ %s 初始化完成（通道 %d）\n", name, channel);
  return true;
}

// ------------------------
// 讀取一顆 IMU 的原始加速度與角速度
// ------------------------
void readIMU(Adafruit_MPU6050 &imu, int channel, float data[6]) {
  sensors_event_t a, g, temp;

  tca_select(channel);
  imu.getEvent(&a, &g, &temp);

  // 加速度（轉成 g）
  data[0] = a.acceleration.x / 9.80665f;
  data[1] = a.acceleration.y / 9.80665f;
  data[2] = a.acceleration.z / 9.80665f;

  // 陀螺儀（rad/s）
  data[3] = g.gyro.x;
  data[4] = g.gyro.y;
  data[5] = g.gyro.z;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);   // ESP32 SDA=21, SCL=22
  Wire.setClock(400000);

  Serial.println("=== MPU6050 + TCA9548A 原始 IMU 資料版本 ===");

  // 檢查多工器
  Wire.beginTransmission(TCA_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("❌ 找不到 TCA9548A（0x70）");
    while (1);
  }
  Serial.println("✔ TCA9548A 多工器已偵測到！");

  // 初始化三顆 IMU
  initIMU(imu1, 0, "IMU1");
  initIMU(imu2, 1, "IMU2");
  initIMU(imu3, 2, "IMU3");
}

void loop() {
  float imu1_data[6], imu2_data[6], imu3_data[6];

  // 讀取三顆 IMU
  readIMU(imu1, 0, imu1_data);
  readIMU(imu2, 1, imu2_data);
  readIMU(imu3, 2, imu3_data);

  // 輸出 18 個原始數值給 Python
  Serial.printf(
    "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,   %.4f,%.4f,%.4f,%.4f,%.4f,%.4f,   %.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
    imu1_data[0], imu1_data[1], imu1_data[2], imu1_data[3], imu1_data[4], imu1_data[5],
    imu2_data[0], imu2_data[1], imu2_data[2], imu2_data[3], imu2_data[4], imu2_data[5],
    imu3_data[0], imu3_data[1], imu3_data[2], imu3_data[3], imu3_data[4], imu3_data[5]
  );

  delay(10);  // 100 Hz
}
