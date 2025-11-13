/*
 * I2C Scanner (掃描器)
 * * 這個程式會掃描預設 I2C 匯流排 (Wire) 上的所有位址 (0-127)
 * 並在序列埠監控視窗中回報找到的裝置位址。
 * * 接線:
 * - SCL -> GPIO 22 (P22)
 * - SDA -> GPIO 21 (P21)
 */
 
#include <Wire.h>

void setup() {
  Wire.begin(21, 22); // (SDA, SCL)
  Serial.begin(115200);
  while (!Serial); // 等待 Serial 連線
  Serial.println("\nI2C Scanner is running...");
  Serial.println("-------------------------");
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("Scanning...");

  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) { // 0 代表成功找到裝置
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX); // 以 16 進位 (HEX) 印出位址
      Serial.println("  <--- FOUND!");
      nDevices++;
    } else if (error == 4) { // 4 代表其他錯誤
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }    
  }
  
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("Done.\n");
  }
  
  delay(5000); // 每 5 秒鐘重新掃描一次
}