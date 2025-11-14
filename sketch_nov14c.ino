#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>

#define TCA_ADDR 0x70  // 多工器位址

// 3 顆 IMU 與 3 個濾波器
Adafruit_MPU6050 imu1, imu2, imu3;
Madgwick filter1, filter2, filter3;


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
// 讀取一顆 IMU 並套用 Madgwick Filter
// ------------------------
void readIMU(Adafruit_MPU6050 &imu, Madgwick &filter, int channel, float q[4]) {
  sensors_event_t a, g, temp;

  tca_select(channel);
  imu.getEvent(&a, &g, &temp);

  // 加速度（轉成 g）
  float ax = a.acceleration.x / 9.80665f;
  float ay = a.acceleration.y / 9.80665f;
  float az = a.acceleration.z / 9.80665f;

  // 陀螺儀（rad/s）
  float gx = g.gyro.x;
  float gy = g.gyro.y;
  float gz = g.gyro.z;

  // Madgwick 更新
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  // ⭐ 使用 public 成員取得 quaternion（你的版本支援）
  q[0] = filter.q0;  // w
  q[1] = filter.q1;  // x
  q[2] = filter.q2;  // y
  q[3] = filter.q3;  // z
}



void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);   // ESP32 SDA=21, SCL=22
  Wire.setClock(400000);

  Serial.println("=== MPU6050 + TCA9548A + Madgwick 四元數版本 ===");

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

  // 設定 Madgwick 頻率
  filter1.begin(100);
  filter2.begin(100);
  filter3.begin(100);
}



void loop() {
  float q1[4], q2[4], q3[4];

  // 讀取三顆 IMU
  readIMU(imu1, filter1, 0, q1);
  readIMU(imu2, filter2, 1, q2);
  readIMU(imu3, filter3, 2, q3);

  // 輸出四元數（給 Python / Unity）
  Serial.printf(
    "%.4f,%.4f,%.4f,%.4f,   %.4f,%.4f,%.4f,%.4f,   %.4f,%.4f,%.4f,%.4f\n",
    q1[0], q1[1], q1[2], q1[3],
    q2[0], q2[1], q2[2], q2[3],
    q3[0], q3[1], q3[2], q3[3]
  );

  delay(10);  // 100 Hz
}
