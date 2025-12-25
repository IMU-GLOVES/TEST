import serial
import struct
import time
import serial.tools.list_ports

IMU_NUM = 3 #imu數量
class SerialManager:
    def __init__(self, port='COM4', baudrate=115200, num_imus=IMU_NUM):
        """
        初始化通訊管理器
        :param port: 指定 COM Port (例如 'COM3' 或 '/dev/ttyUSB0')，若為 None 則嘗試自動搜尋
        :param baudrate: 鮑率，必須跟 Arduino 設定的一樣 (建議 115200 或 400000)
        :param num_imus: IMU 的數量 (目前是 3，之後改成 10 只要改這裡)
        """
        self.port = port
        self.baudrate = baudrate
        self.num_imus = num_imus
        self.packet_size = num_imus * 12  # 每個 IMU 12 bytes (6個 int16)
        self.ser = None

    def find_esp32_port(self):
        """自動列出並尋找可能的 ESP32/Arduino Port"""
        ports = list(serial.tools.list_ports.comports())
        print(f"掃描到 {len(ports)} 個序列埠:")
        for p in ports:
            print(f" - {p.device}: {p.description}")
            # 簡單的自動判斷邏輯：通常描述裡會有 'Silicon Labs' (CP210x) 或 'USB-SERIAL'
            if "Silicon" in p.description or "USB" in p.description:
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
            time.sleep(2) # 等待 Arduino 重啟與穩定
            print(f"✔ 成功連線至 {self.port} @ {self.baudrate}")
            # 清空緩衝區，避免讀到舊的垃圾數據
            self.ser.reset_input_buffer()
            return True
        except Exception as e:
            print(f"❌ 連線失敗: {e}")
            return False

    def read_data(self):
        """
        讀取一幀完整的數據
        :return: 包含所有 IMU 數據的列表 (List of Lists)，失敗則回傳 None
                 格式: [[ax, ay, az, gx, gy, gz], [ax, ay, az, gx, gy, gz], ...]
        """
        if not self.ser or not self.ser.is_open:
            return None

        try:
            # 1. 同步：尋找封包標頭 0xAA 0xBB
            # 我們讀取一個 byte，如果是 AA，再讀下一個看是不是 BB
            while True:
                if self.ser.in_waiting > 0:
                    byte1 = self.ser.read(1)
                    if byte1 == b'\xAA':
                        byte2 = self.ser.read(1)
                        if byte2 == b'\xBB':
                            # 找到了！跳出迴圈開始讀數據
                            break
                else:
                    # 沒有數據時稍微休息，避免 CPU 100%
                    return None 

            # 2. 讀取 Payload (主體數據)
            # 我們需要讀取 num_imus * 12 個 bytes
            raw_data = self.ser.read(self.packet_size)

            if len(raw_data) != self.packet_size:
                print("⚠️ 數據長度不符，丟棄此幀")
                return None

            # 3. 解包 (Unpack)
            # '<' 代表 Little Endian (Arduino 標準)
            # 'h' 代表 short (int16, 2 bytes)
            # 我們需要解析 (6 * num_imus) 個 short
            fmt = '<' + 'h' * (6 * self.num_imus)
            unpacked_data = struct.unpack(fmt, raw_data)

            # 4. 整理數據格式
            # 把扁平的 list 整理成 [[IMU0], [IMU1], [IMU2]...] 的格式
            organized_data = []
            for i in range(self.num_imus):
                start_idx = i * 6
                # 取出這一顆 IMU 的 6 個數據
                imu_readings = list(unpacked_data[start_idx : start_idx + 6])
                organized_data.append(imu_readings)

            return organized_data

        except Exception as e:
            print(f"讀取錯誤: {e}")
            return None

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("連線已關閉")

# =========================================
# 獨立測試區 (直接執行這個檔案可以用來測試連線)
# =========================================
if __name__ == "__main__":
    # 使用範例：指定你的 COM port，或是填 None 讓它自己找
    # windows 可能是 'COM3', mac 可能是 '/dev/tty.usbserial-xxx'
    manager = SerialManager(port=None, baudrate=115200, num_imus=IMU_NUM) 
    
    if manager.connect():
        print("開始接收原始數據 (按 Ctrl+C 停止)...")
        try:
            while True:
                data = manager.read_data()
                if data:
                    #print(f"IMU0: {data[0]}")
                    #print(f"IMU1: {data[1]}")
                    print(f"IMU2: {data[2]}")
                    print("-" * 30) # 分隔線
                    # 你會看到類似 [-1200, 16300, 500, 20, -15, 3]
        except KeyboardInterrupt:
            print("\n停止測試")
        finally:
            manager.close()