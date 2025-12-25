#include <Wire.h>

// ==========================================
// [設定] 除錯模式
// true  = 開啟文字模式 (人看得懂，速度慢，Python 不能用)
// false = 開啟極速模式 (亂碼，速度快，給 Python 用)
// ==========================================
const bool DEBUG_MODE = false; // <--- 想檢查數據時改成 true，要接 Python 時改成 false
// ==========================================
// [類別定義] FastIMU
// 負責處理單顆 MPU6050 的所有底層操作
// 包含自動切換多功器、讀取數據、優化傳輸
// ==========================================
class FastIMU {
  private:
    uint8_t _muxAddr;   // 這顆 IMU 接在哪個多功器 (0x70 或 0x71)
    uint8_t _channel;   // 這顆 IMU 接在該多功器的哪個通道 (0-7)
    uint8_t _imuAddr = 0x68; // MPU6050 預設位址

    // --- 核心邏輯：切換多功器通道 ---
    // 這裡解決了 "雙多功器" 的衝突問題
    void selectMux() {
      // 1. 如果未來有兩顆多功器，先關閉 "另一顆" 以防衝突
      // 如果我是 0x70，就關掉 0x71；如果我是 0x71，就關掉 0x70
      uint8_t otherMux = (_muxAddr == 0x70) ? 0x71 : 0x70;
      
      Wire.beginTransmission(otherMux);
      Wire.write(0); // 寫入 0 代表關閉該多功器所有通道
      Wire.endTransmission();

      // 2. 開啟 "這一顆" 多功器的指定通道
      // 寫入 (1 << _channel) 會自動開啟指定通道並關閉同顆的其他通道
      Wire.beginTransmission(_muxAddr);
      Wire.write(1 << _channel);
      Wire.endTransmission();
    }

  public:
    // 儲存 6 個軸向數據 (AccX, Y, Z, GyroX, Y, Z)
    // 使用 int16_t (2 bytes) 而非 float (4 bytes)，節省一半頻寬
    int16_t data[6]; 

    // 建構子：設定這顆 IMU 的身分
    FastIMU(uint8_t muxAddr, uint8_t channel) {
      _muxAddr = muxAddr;
      _channel = channel;
    }

    // 初始化感測器
    void begin() {
      selectMux(); // 切換過去
      Wire.beginTransmission(_imuAddr);
      Wire.write(0x6B); // 電源管理暫存器
      Wire.write(0);    // 寫入 0 喚醒 MPU6050
      Wire.endTransmission();
    }

    // 讀取數據 (極速版)
    void update() {
      selectMux(); // 確保現在是連通這顆 IMU

      // 設定讀取起始點：0x3B (加速度 X 高位)
      Wire.beginTransmission(_imuAddr);
      Wire.write(0x3B);
      Wire.endTransmission(false); // Restart 訊號，保持連線

      // 一次索取 14 個字節 (Acc:6 + Temp:2 + Gyro:6)
      Wire.requestFrom(_imuAddr, (uint8_t)14);

      if (Wire.available() >= 14) {
        // 使用位元運算組合 High Byte 和 Low Byte
        data[0] = Wire.read() << 8 | Wire.read(); // Acc X
        data[1] = Wire.read() << 8 | Wire.read(); // Acc Y
        data[2] = Wire.read() << 8 | Wire.read(); // Acc Z
        
        Wire.read(); Wire.read(); // 跳過溫度 (Temp)，我們不需要
        
        data[3] = Wire.read() << 8 | Wire.read(); // Gyro X
        data[4] = Wire.read() << 8 | Wire.read(); // Gyro Y
        data[5] = Wire.read() << 8 | Wire.read(); // Gyro Z
      }
    }
};

// ==========================================
// [使用者修改區] 硬體配置設定
// 這裡決定你有幾顆 IMU，接在哪裡
// ==========================================

// 建立 IMU 物件清單
FastIMU imuList[] = {
  // --- 目前配置 (1 個多功器，3 顆 IMU) ---
  FastIMU(0x70, 0), // 第 1 顆：多功器 0x70, 通道 0
  FastIMU(0x70, 1), // 第 2 顆：多功器 0x70, 通道 1
  FastIMU(0x70, 2), // 第 3 顆：多功器 0x70, 通道 2

  // --- 未來擴充範例 (直接解除註解即可) ---
  // FastIMU(0x70, 3), // 第 4 顆
  // FastIMU(0x70, 4), // ...
  // FastIMU(0x70, 7), // 第 8 顆 (0x70 滿了)
  
  // --- 第二個多功器 (0x71) ---
  // 硬體接法：第二個 TCA9548A 的 A0 接 3.3V
  // FastIMU(0x71, 0), // 第 9 顆：多功器 0x71, 通道 0
  // FastIMU(0x71, 1)  // 第 10 顆
};

// 自動計算有幾顆 IMU (不用手動改數字)
const int NUM_IMUS = sizeof(imuList) / sizeof(imuList[0]);

// ==========================================
// 主程式 Setup
// ==========================================
void setup() {
  // 1. 初始化序列埠 (速度越快越好，Python 端要對應)
  Serial.begin(115200);

  // 2. 初始化 I2C
  // 根據你提供的 NodeMCU/ESP32 腳位
  Wire.begin(21, 22); 
  
  // [重要] 提升 I2C 速度到 400kHz (預設是 100kHz)
  // 這對讀取 10 顆 IMU 至關重要
  Wire.setClock(400000); 

  delay(100); // 等待電壓穩定

  // 3. 逐一喚醒所有 IMU
  for (int i = 0; i < NUM_IMUS; i++) {
    imuList[i].begin();
    delay(10); // 給一點點時間讓感測器啟動
  }
}

// ==========================================
// 主程式 Loop
// ==========================================
void loop() {
  // 如果是 Debug 模式，就印文字
  if (DEBUG_MODE) {
    for (int i = 0; i < NUM_IMUS; i++) {
      imuList[i].update();
      
      Serial.print("IMU"); Serial.print(i); 
      // 顯示加速度 (Acc)
      Serial.print(" A: ");
      Serial.print(imuList[i].data[0]); Serial.print(", ");
      Serial.print(imuList[i].data[1]); Serial.print(", ");
      Serial.print(imuList[i].data[2]);
      
      // 顯示陀螺儀 (Gyro)
      Serial.print(" | G: ");
      Serial.print(imuList[i].data[3]); Serial.print(", ");
      Serial.print(imuList[i].data[4]); Serial.print(", ");
      Serial.println(imuList[i].data[5]); // 換行
    }
    Serial.println("----------------"); // 分隔線
    delay(200); // 方便閱讀

  } else {
    // 1. 發送封包標頭 (Header) - 2 Bytes
    // 用來讓 Python 識別這是新的一幀數據
    Serial.write(0xAA);
    Serial.write(0xBB);

    // 2. 讀取並傳送所有 IMU 數據
    for (int i = 0; i < NUM_IMUS; i++) {
      imuList[i].update(); // 讀取這一顆的數據
    
      // 直接發送二進位數據 (6 個 int16 = 12 Bytes)
      // (uint8_t*) 是將 int16 指標轉型為 byte 指標，以便逐字節發送
      Serial.write((uint8_t*)imuList[i].data, 12);
    }
    // 3. 控制採樣率
    // delay(5) 約等於 200Hz (含運算時間)
    // 如果 Python 端處理不及，可以把這個改大一點 (例如 10 或 20)
    delay(5);
  } 
}