#include <Wire.h>

// ==========================================
//   使用者設定區 (User Configuration)
// ==========================================

// [模式切換]
// true  = 人類可讀模式 (開啟序列埠監控視窗查看數據, 檢查硬體用)
// false = Python 極速模式 (發送二進位封包, 視覺化用)
const bool DEBUG_MODE = false; 

// [系統參數]
// 設定目前連接的 IMU 總數量 (目前 3顆指節 + 1顆手掌 = 4)
const int TOTAL_IMUS = 4;

// 多功器位址定義
#define MUX_ADDR_A 0x70 // 第一顆多功器 (A0, A1, A2 懸空)
#define MUX_ADDR_B 0x71 // 第二顆多功器 (未來擴充用: A0 接 VCC)

// ==========================================
//   基礎類別定義 (Base Class)
// ==========================================
// 這是一個「模板」，定義了所有 IMU 都該有的基本功能
class GenericIMU {
  public:
    uint8_t muxAddress; // 這顆 IMU 接在哪一顆多功器上 (0x70 或 0x71)
    uint8_t muxChannel; // 這顆 IMU 接在該多功器的第幾號通道 (0~7)
    uint8_t dataBytes;  // 這個 IMU 會吐出幾個 bytes (6050是12, 9250是18)
    int16_t* rawData;   // 用來儲存讀到的數據 (指標陣列)

    // 建構子 (Constructor): 建立物件時會執行這裡
    GenericIMU(uint8_t muxAddr, uint8_t channel, uint8_t bytes) {
      muxAddress = muxAddr;
      muxChannel = channel;
      dataBytes = bytes;
      // 根據數據量動態分配記憶體空間 (bytes / 2 因為 int16 佔 2 bytes)
      rawData = new int16_t[bytes / 2]; 
    }

    // 虛擬函式 (Virtual Functions): 子類別必須「覆寫 (override)」這些功能
    virtual void init() = 0;   // 初始化設定
    virtual void update() = 0; // 讀取數據

    // [共用功能] 切換多功器通道
    // 這是控制 "火車鐵軌" 的核心函式
    void selectMux() {
      // 1. 指定跟哪一顆多功器講話
      Wire.beginTransmission(muxAddress);
      // 2. 指定開啟哪一個通道 (使用位元位移操作)
      Wire.write(1 << muxChannel);
      Wire.endTransmission();
    }
};

// ==========================================
//   MPU-6050 類別 (6軸手指專用)
// ==========================================
class MPU6050_Node : public GenericIMU {
  public:
    // 建構子: 呼叫爸爸 (GenericIMU) 的建構子，固定數據長度為 12 bytes
    MPU6050_Node(uint8_t muxAddr, uint8_t channel) : GenericIMU(muxAddr, channel, 12) {} 

    // 初始化: 喚醒 MPU6050
    void init() override {
      selectMux();      // 切換軌道
      
      Wire.beginTransmission(0x68);
      Wire.write(0x6B); // 暫存器: PWR_MGMT_1 (電源管理)
      Wire.write(0);    // 寫入 0 = 解除睡眠模式 (喚醒)
      Wire.endTransmission();
    }

    // 更新: 讀取 Acc 和 Gyro
    void update() override {
      selectMux();      // 切換軌道

      // 1. 設定讀取指標
      Wire.beginTransmission(0x68);
      Wire.write(0x3B); // 從 ACCEL_XOUT_H (加速度X高位) 開始讀
      Wire.endTransmission(false); // false 代表不釋放總線，準備接著讀
      
      // 2. 一口氣讀取 14 bytes (Acc x6 + Temp x2 + Gyro x6)
      Wire.requestFrom(0x68, 14);
      
      if (Wire.available() == 14) {
        // MPU6050 是 Big Endian (高位在前)，所以先讀高位左移 8 bit 再加低位
        rawData[0] = (Wire.read() << 8) | Wire.read(); // Acc X
        rawData[1] = (Wire.read() << 8) | Wire.read(); // Acc Y
        rawData[2] = (Wire.read() << 8) | Wire.read(); // Acc Z
        Wire.read(); Wire.read(); // 讀掉溫度數據 (我們不需要，丟棄)
        rawData[3] = (Wire.read() << 8) | Wire.read(); // Gyro X
        rawData[4] = (Wire.read() << 8) | Wire.read(); // Gyro Y
        rawData[5] = (Wire.read() << 8) | Wire.read(); // Gyro Z
      }
    }
};

// ==========================================
//   MPU-9250 類別 (9軸手掌專用)
// ==========================================
class MPU9250_Node : public GenericIMU {
  public:
    // AK8963 磁力計的 I2C 位址 (這是藏在 MPU9250 裡面的另一顆晶片)
    const uint8_t MAG_ADDR = 0x0C;

    // 建構子: 數據長度為 18 bytes (6軸 + 3軸磁力)
    MPU9250_Node(uint8_t muxAddr, uint8_t channel) : GenericIMU(muxAddr, channel, 18) {} 

    // 初始化: 步驟比較繁瑣，要設定 "Bypass Mode"
    void init() override {
      selectMux();
      
      // 1. 喚醒 MPU9250 本體
      Wire.beginTransmission(0x68);
      Wire.write(0x6B);
      Wire.write(0);
      Wire.endTransmission();
      delay(10);

      // 2. 開啟 Bypass Mode 
      // 這讓 Arduino 可以直接透過 I2C 跟內部的 AK8963 講話
      Wire.beginTransmission(0x68);
      Wire.write(0x37); // INT_PIN_CFG
      Wire.write(0x02); // Bit 1 = BYPASS_EN
      Wire.endTransmission();
      delay(10);

      // 3. 設定 AK8963 磁力計
      // 步驟 A: 重置磁力計 (Power Down)
      writeMagRegister(0x0B, 0x01); 
      delay(10);
      
      // 步驟 B: 設定為 16-bit 輸出, 100Hz 連續測量模式
      // CNTL1 register (0x0A): "0001" (16-bit) + "0110" (Mode 2) = 0x16
      writeMagRegister(0x0A, 0x16); 
      delay(10);
    }

    // 更新: 讀取 Acc, Gyro 和 Mag
    void update() override {
      selectMux();

      // --- Part A: 讀取 6軸 (跟 MPU6050 一樣) ---
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(0x68, 14);

      if (Wire.available() == 14) {
        rawData[0] = (Wire.read() << 8) | Wire.read(); // Acc X
        rawData[1] = (Wire.read() << 8) | Wire.read(); // Acc Y
        rawData[2] = (Wire.read() << 8) | Wire.read(); // Acc Z
        Wire.read(); Wire.read(); // 忽略溫度
        rawData[3] = (Wire.read() << 8) | Wire.read(); // Gyro X
        rawData[4] = (Wire.read() << 8) | Wire.read(); // Gyro Y
        rawData[5] = (Wire.read() << 8) | Wire.read(); // Gyro Z
      }

      // --- Part B: 讀取 3軸 (磁力計) ---
      // AK8963 需要讀取 ST2 (0x09) 暫存器來確認數據讀取完畢
      Wire.beginTransmission(MAG_ADDR);
      Wire.write(0x03); // 從 HXL (磁力X低位) 開始讀
      Wire.endTransmission(false);
      
      Wire.requestFrom(MAG_ADDR, 7); // 讀 6 byte 數據 + 1 byte ST2
      if (Wire.available() == 7) {
        // 注意：磁力計是 Little Endian (低位在前)，這跟 MPU6050 相反！
        uint8_t xl = Wire.read(); uint8_t xh = Wire.read();
        uint8_t yl = Wire.read(); uint8_t yh = Wire.read();
        uint8_t zl = Wire.read(); uint8_t zh = Wire.read();
        uint8_t st2 = Wire.read(); // 這是狀態位，必須讀出來才能結束傳輸

        // 組合數據 (先將高位左移，再跟低位做 OR)
        rawData[6] = (int16_t)(xh << 8 | xl); // Mag X
        rawData[7] = (int16_t)(yh << 8 | yl); // Mag Y
        rawData[8] = (int16_t)(zh << 8 | zl); // Mag Z
      }
    }

  private:
    // 輔助函式: 寫入磁力計暫存器
    void writeMagRegister(uint8_t reg, uint8_t data) {
      Wire.beginTransmission(MAG_ADDR);
      Wire.write(reg);
      Wire.write(data);
      Wire.endTransmission();
    }
};

// ==========================================
//   全域變數與物件管理
// ==========================================

// 建立一個指標陣列來管理所有的 IMU
GenericIMU* imus[TOTAL_IMUS]; 

// ==========================================
//   Arduino Setup (開機執行一次)
// ==========================================
void setup() {
  // 1. 初始化通訊
  Serial.begin(115200); // 鮑率設定
  Wire.begin();         // I2C 初始化
  Wire.setClock(400000); // 開啟 400kHz I2C 極速模式 (很重要，不然讀不完)

  // 2. 建立 IMU 物件 (對應你的硬體接線)
  // 語法: new MPUxxxx_Node(多功器位址, 通道號碼)
  
  // --- [目前硬體配置] ---
  // 多功器 A (0x70)
  imus[0] = new MPU6050_Node(MUX_ADDR_A, 0); // 指節 1 (接 SD0)
  imus[1] = new MPU6050_Node(MUX_ADDR_A, 1); // 指節 2 (接 SD1)
  imus[2] = new MPU6050_Node(MUX_ADDR_A, 2); // 指節 3 (接 SD2)
  
  // 手掌 (MPU9250) 接在 SD7 (特意留到最後)
  imus[3] = new MPU9250_Node(MUX_ADDR_A, 7); 


  /* --- [未來擴充預留位置] (等你買了第二顆多功器再來解除註解) ---
  // 設定 TOTAL_IMUS 改成 15
  
  // 多功器 A (0x70) 接滿 8 顆 6050
  imus[0] = new MPU6050_Node(MUX_ADDR_A, 0);
  ...
  imus[7] = new MPU6050_Node(MUX_ADDR_A, 7);

  // 多功器 B (0x71) 接剩下的 6050
  imus[8] = new MPU6050_Node(MUX_ADDR_B, 0);
  ...
  imus[13] = new MPU6050_Node(MUX_ADDR_B, 5);

  // 手掌 9250 移到多功器 B 的 SD7
  imus[14] = new MPU9250_Node(MUX_ADDR_B, 7);
  */

  // 3. 逐一初始化所有感測器
  if (DEBUG_MODE) Serial.println(">>> System Initializing...");
  
  for (int i = 0; i < TOTAL_IMUS; i++) {
    imus[i]->init();
    if (DEBUG_MODE) {
      Serial.print("IMU "); Serial.print(i); 
      Serial.print(" (Ch"); Serial.print(imus[i]->muxChannel);
      Serial.println(") initialized.");
    }
    delay(50); // 稍微休息，避免電流衝擊
  }
  
  if (DEBUG_MODE) {
    Serial.println(">>> Initialization Complete. Starting Loop...");
    Serial.println("--------------------------------");
    delay(1000);
  }
}

// ==========================================
//   Arduino Loop (無限迴圈)
// ==========================================
void loop() {
  // 1. 讀取所有感測器數據
  for (int i = 0; i < TOTAL_IMUS; i++) {
    imus[i]->update();
  }

  // 2. 輸出數據 (根據模式選擇)
  
  // [模式 A] Debug 文字模式 (給人看)
  if (DEBUG_MODE) {
    for (int i = 0; i < TOTAL_IMUS; i++) {
      Serial.print("IMU"); Serial.print(i);
      
      // 印出加速度與陀螺儀 (Acc, Gyro) - 這是所有 IMU 都有的
      Serial.print(" A:[");
      Serial.print(imus[i]->rawData[0]); Serial.print(",");
      Serial.print(imus[i]->rawData[1]); Serial.print(",");
      Serial.print(imus[i]->rawData[2]); Serial.print("] G:[");
      Serial.print(imus[i]->rawData[3]); Serial.print(",");
      Serial.print(imus[i]->rawData[4]); Serial.print(",");
      Serial.print(imus[i]->rawData[5]); Serial.print("]");
      
      // 如果是 9250 (dataBytes == 18)，多印出磁力計 (Mag)
      if (imus[i]->dataBytes == 18) {
        Serial.print(" M:[");
        Serial.print(imus[i]->rawData[6]); Serial.print(",");
        Serial.print(imus[i]->rawData[7]); Serial.print(",");
        Serial.print(imus[i]->rawData[8]); Serial.print("]");
      }
      Serial.println(); // 該 IMU 數據結束換行
    }
    Serial.println("--- End of Frame ---"); // 一幀結束
    delay(200); // 放慢速度方便閱讀
  }
  
  // [模式 B] Python 極速模式 (給電腦看)
  else {
    // A. 發送封包標頭 (Header) 用來同步
    Serial.write(0xAA);
    Serial.write(0xBB);

    // B. 發送所有數據 (Payload)
    for (int i = 0; i < TOTAL_IMUS; i++) {
      // 將 int16 陣列轉成 byte 串流發送
      // 6050 會送 12 bytes, 9250 會送 18 bytes
      Serial.write((uint8_t*)imus[i]->rawData, imus[i]->dataBytes);
    }
    
    // C. 控制採樣率 (Loop 速度)
    // delay(5) 約等於 200Hz 採樣率 (考慮到讀取時間，實際約 100Hz)
    delay(5);
  }
}