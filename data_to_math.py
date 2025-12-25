import math
import time

IMU_NUM = 3 #imu數量
class HandProcessor:
    def __init__(self, num_imus=IMU_NUM):
        self.num_imus = num_imus
        
        # --- 參數設定 ---
        self.rad_to_deg = 180.0 / math.pi
        
        # 互補濾波係數 (0.95 相信陀螺儀, 0.05 相信加速度計)
        # 如果覺得反應太慢，可以把 0.95 改小 (例如 0.90)
        self.alpha = 0.95 
        
        # --- 狀態變數 ---
        # 儲存每一顆 IMU 的 [Pitch, Roll, Yaw]
        # 初始值都是 [0.0, 0.0, 0.0]
        self.current_angles = [[0.0, 0.0, 0.0] for _ in range(num_imus)]
        
        # 儲存 Offset (歸零偏差)
        self.angle_offsets = [[0.0, 0.0, 0.0] for _ in range(num_imus)]
        
        # 上一次更新的時間 (用來算 dt)
        self.last_time = time.time()

    def reset_calibration(self):
        """
        【重置功能】
        當使用者按下 'R' 鍵時呼叫此函式。
        將當下的計算角度設為 "新的零點"。
        """
        print(">>> 執行歸零校正 (Tare) <<<")
        # 把當下的 [P, R, Y] 存為偏差值
        for i in range(self.num_imus):
            for j in range(3): # 0:Pitch, 1:Roll, 2:Yaw
                self.angle_offsets[i][j] = self.current_angles[i][j]

    def process(self, raw_data_list):
        """
        主運算函式
        :param raw_data_list: 從 SerialManager 收到的 [[ax,ay,az,gx,gy,gz], ...]
        :return: 修正後的角度列表 (List of floats) [angle0, angle1, angle2]
        """
        if raw_data_list is None:
            return None

        # 1. 計算動態 dt (距離上次運算過了多久)
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        
        # 避免 dt 過大 (例如剛啟動或卡頓時)，限制最大 0.1 秒
        if dt > 0.1: dt = 0.01

        # 最終回傳的資料結構：一個包含多個 [P, R, Y] 的列表
        final_output = []

        for i in range(self.num_imus):
            raw = raw_data_list[i]
            
            # --- 1. 取出原始數據 ---
            # 根據 MPU6050 定義
            acc_x = raw[0]
            acc_y = raw[1]
            acc_z = raw[2]
            gyro_x = raw[3]
            gyro_y = raw[4]
            gyro_z = raw[5]

            # --- 2. 物理單位轉換 ---
            # Gyro 轉成 deg/s (給互補濾波用)
            gyr_x_dps = gyro_x / 131.0
            gyr_y_dps = gyro_y / 131.0
            gyr_z_dps = gyro_z / 131.0

            # --- 3. 計算加速度角度 (Acc Angles) ---
            # Pitch (繞 X 軸): 用 Y, Z
            acc_pitch = math.atan2(acc_y, acc_z) * self.rad_to_deg
            
            # Roll (繞 Y 軸): 用 X, Z
            # 注意: 根據公式，這裡通常要放負號，或取決於安裝方向
            acc_roll = math.atan2(-acc_x, acc_z) * self.rad_to_deg

            # --- 4. 互補濾波 (Pitch & Roll) ---
            # 這些有重力參考，所以用濾波
            prev_p = self.current_angles[i][0]
            prev_r = self.current_angles[i][1]
            
            new_pitch = self.alpha * (prev_p + gyr_x_dps * dt) + (1 - self.alpha) * acc_pitch
            new_roll  = self.alpha * (prev_r + gyr_y_dps * dt) + (1 - self.alpha) * acc_roll

            # --- 5. 純積分 (Yaw) ---
            # Yaw 沒有重力參考，只能純粹累加 Gyro Z
            prev_y = self.current_angles[i][2]
            new_yaw = prev_y + (gyr_z_dps * dt)

            # 更新狀態
            self.current_angles[i] = [new_pitch, new_roll, new_yaw]

            # --- 6. 扣除偏差 (Output) ---
            display_pitch = new_pitch - self.angle_offsets[i][0]
            display_roll  = new_roll  - self.angle_offsets[i][1]
            display_yaw   = new_yaw   - self.angle_offsets[i][2]

            final_output.append([display_pitch, display_roll, display_yaw])

        return final_output

# =========================================
# 獨立測試區
# =========================================
if __name__ == "__main__":
    # 這裡我們要模擬 "SerialManager" 和 "HandProcessor" 一起工作
    # 為了測試，我們需要 import SerialManager
    from serial_manager import SerialManager

    # 1. 初始化
    manager = SerialManager(num_imus=IMU_NUM) # 如果你知道 port 可以加 port='COMx'
    processor = HandProcessor(num_imus=IMU_NUM)
    
    if manager.connect():
        print("開始計算角度... (請彎曲手指測試)")
        print("按 Ctrl+C 停止")
        
        try:
            # 模擬主迴圈
            while True:
                # 1. 讀取
                raw_data = manager.read_data()
                
                if raw_data:
                    # 2. 運算
                    angles = processor.process(raw_data)
                    
                    # 3. 顯示
                    # angles[0] 是第一顆 IMU 的 [Pitch, Roll, Yaw]
                    # 我們分別把它們印出來
                    
                    # 為了版面整潔，我們只印第一顆 IMU (指尖) 的三個角度來觀察
                    p0, r0, y0 = angles[0]
                    p1, r1, y1 = angles[1]
                    p2, r2, y2 = angles[2]

                    print(f"IMU0 (指節): P={p0:6.2f} | R={r0:6.2f} | Y={y0:6.2f}")
                    # 如果你想看全部，可以把下面這行解除註解
                    # print(f"IMU1: P={p1:6.2f} R={r1:6.2f} Y={y1:6.2f} | IMU2: P={p2:6.2f} R={r2:6.2f} Y={y2:6.2f}")
                    print("-" * 30) # 分隔線，讓視覺不要太花
                
        except KeyboardInterrupt:
            print("停止")
        finally:
            manager.close()