import serial
import struct
import time
import serial.tools.list_ports

class SerialManager:
    def __init__(self, port=None, baudrate=115200, imu_setup=None):
        """
        初始化通訊管理器
        :param port: 指定 COM Port (例如 'COM3')，若為 None 則自動搜尋
        :param baudrate: 鮑率 (必須與 Arduino 設定一致，建議 115200 或 400000)
        :param imu_setup: [重要] IMU 配置列表。
                          格式為 list，裡面的數字代表該 IMU 的軸數。
                          例如: [6, 6, 6, 9] 代表前三顆是 6050，最後一顆是 9250。
        """
        self.port = port
        self.baudrate = baudrate
        self.ser = None

        # --- 1. 設定 IMU 配置 (若未指定，給一個預設值避免報錯) ---
        if imu_setup is None:
            print("⚠️ 警告: 未指定 imu_setup，使用預設值 [6, 6, 6, 9]")
            self.imu_setup = [6, 6, 6]
        else:
            self.imu_setup = imu_setup

        # --- 2. 自動計算封包結構 ---
        # 為什麼要這樣算？因為 Arduino 傳來的是一長串二進位數據
        # 我們必須知道總共有幾個 Byte，才能正確從 Serial 讀取
        
        # 總 Bytes 數 = 所有 IMU 軸數的總和 * 2 (因為每個數據是 int16，佔 2 bytes)
        # 例如: (6+6+6+9) * 2 = 54 bytes
        self.total_bytes = sum([axes * 2 for axes in self.imu_setup])
        
        # 總數值個數 (Total Shorts) = 所有軸數的總和
        # 例如: 6+6+6+9 = 27 個數值
        self.total_shorts = sum(self.imu_setup)

        # 顯示目前的配置狀態給使用者看
        print(f"--- SerialManager 設定 ---")
        print(f"IMU 數量: {len(self.imu_setup)} 顆")
        print(f"IMU 配置: {self.imu_setup}")
        print(f"預期封包: {self.total_bytes} bytes (包含 {self.total_shorts} 個數值)")
        print(f"--------------------------")

    def find_esp32_port(self):
        """自動列出並尋找可能的 ESP32/Arduino Port"""
        ports = list(serial.tools.list_ports.comports())
        print(f"掃描到 {len(ports)} 個序列埠:")
        for p in ports:
            print(f" - {p.device}: {p.description}")
            # 常見的 USB 轉 TTL 晶片名稱
            if "Silicon" in p.description or "USB" in p.description or "CH340" in p.description:
                return p.device
        return None

    def connect(self):
        """建立連線"""
        try:
            # 如果沒有指定 port，嘗試自動尋找
            if self.port is None:
                found_port = self.find_esp32_port()
                if found_port:
                    self.port = found_port
                    print(f"自動偵測到 Port: {self.port}")
                else:
                    raise Exception("找不到可用的 COM Port，請手動指定。")

            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2) # 重要：等待 Arduino 重啟 (Arduino 連線瞬間會 Reset)
            print(f"✔ 成功連線至 {self.port} @ {self.baudrate}")
            
            # 清空緩衝區，確保不會讀到連線前的垃圾數據
            self.ser.reset_input_buffer()
            return True
        except Exception as e:
            print(f"❌ 連線失敗: {e}")
            return False

    def read_data(self):
        """
        讀取並解析一幀完整的數據
        :return: 整理好的二維列表 (List of Lists)
                 例如: [[ax,ay,az,gx,gy,gz], ..., [ax,ay,az,gx,gy,gz,mx,my,mz]]
        """
        if not self.ser or not self.ser.is_open:
            return None

        try:
            # --- 步驟 A: 同步封包標頭 (Header) ---
            # 這是為了確保我們讀到的真的是一幀數據的開頭，而不是中間
            while True:
                if self.ser.in_waiting > 0:
                    byte1 = self.ser.read(1)
                    if byte1 == b'\xAA':
                        byte2 = self.ser.read(1)
                        if byte2 == b'\xBB':
                            # 找到了 0xAA, 0xBB，代表後面緊接著就是數據了
                            break 
                else:
                    # 沒有數據時回傳 None，避免卡死程式
                    return None 

            # --- 步驟 B: 讀取有效負載 (Payload) ---
            # 根據我們算出來的 total_bytes 一次讀取
            raw_bytes = self.ser.read(self.total_bytes)
            
            if len(raw_bytes) != self.total_bytes:
                print(f"⚠️ 數據長度不符 (預期 {self.total_bytes}, 實際 {len(raw_bytes)})，丟棄此幀")
                self.ser.reset_input_buffer() # 清空緩衝區重新同步
                return None

            # --- 步驟 C: 解包 (Unpack) ---
            # '<' 代表 Little Endian (Arduino 標準)
            # 'h' 代表 short (int16, 2 bytes)
            # 這裡會產生一個扁平的一維列表，例如: (-1200, 500, 16000, ...)
            fmt = '<' + 'h' * self.total_shorts
            unpacked_data = list(struct.unpack(fmt, raw_bytes))

            # --- 步驟 D: 數據重組 (Reshape) ---
            # 將扁平的列表依照 imu_setup 切割成一顆一顆 IMU 的數據
            organized_data = []
            current_idx = 0
            
            for axes_count in self.imu_setup:
                # 算出這顆 IMU 佔用了幾個數值 (例如 6 或 9)
                end_idx = current_idx + axes_count
                
                # 切割出來
                imu_readings = unpacked_data[current_idx : end_idx]
                organized_data.append(imu_readings)
                
                # 更新指標，準備切下一顆
                current_idx = end_idx

            return organized_data

        except Exception as e:
            print(f"讀取錯誤: {e}")
            self.ser.close()
            return None

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("連線已關閉")

# =========================================
#   使用者設定與測試區 (User Configuration)
# =========================================
if __name__ == "__main__":
    
    # ------------- [未來擴充教學] -------------
    # 這裡教你如何快速設定 IMU 數量，不需要手動打一堆數字
    
    # 【情境 1：現狀 (Current)】
    # 3 顆 6軸 (手指) + 1 顆 9軸 (手掌)
    # Python 寫法：[6] * 3 會產生 [6, 6, 6]
    #current_config = [6] * 3 + [9] 
    current_config = [6] * 14 + [9] 
     
    # 【情境 2：未來 (Future)】
    # 假設你有 14 顆指節 (6軸) + 1 顆手掌 (9軸)
    # 你只需要把下面的 14 改掉就好，超級方便！
    future_config = [6] * 14 + [9]

    # ----------------------------------------
    
    # 選擇你要使用的配置 (目前先用 current_config)
    my_setup = current_config 

    manager = SerialManager(imu_setup=my_setup)
    
    if manager.connect():
        print("開始接收數據... (請確保 Arduino DEBUG_MODE = false)")
        print("按 Ctrl+C 停止測試")
        try:
            while True:
                data = manager.read_data()
                if data:
                    # 簡單印出最後一顆 (9軸) 的數據來驗證
                    # data[-1] 代表列表的最後一個元素
                    print(f"指節 IMU (6軸): {data[-2]}")
                    print(f"手掌 IMU (9軸): {data[-1]}")
                    
                    # 如果你想看 IMU 數量對不對
                    # print(f"收到 {len(data)} 顆 IMU 的數據")
                    
        except KeyboardInterrupt:
            print("\n停止測試")
        finally:
            manager.close()