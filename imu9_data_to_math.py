import math
import time

class DataProcessor:
    def __init__(self):
        """
        初始化運算處理器
        這裡不需要預先知道有幾顆 IMU，也不用知道誰是 6 軸誰是 9 軸。
        程式會在運算時動態判斷。
        """
        self.rad_to_deg = 180.0 / math.pi
        self.deg_to_rad = math.pi / 180.0
        
        # 互補濾波係數 (0.95 = 95% 相信陀螺儀, 5% 修正重力)
        self.alpha = 0.95 
        
        # 儲存每一顆 IMU 的目前角度狀態 [Pitch, Roll, Yaw]
        # 初始為空列表，第一次收到數據時會自動建立
        self.current_angles = []
        
        # 儲存校正用的偏差值 (Offset)
        self.angle_offsets = []
        
        # 時間計算用
        self.last_time = time.time()
        self.is_initialized = False # 標記是否已經初始化陣列大小

    def reset_calibration(self):
        """
        【歸零功能】(按 R 鍵觸發)
        將當下的角度設為 0 度。
        """
        if not self.current_angles: return
        
        print(">>> 執行歸零 (Tare) <<<")
        # 重新複製目前的角度到 offset
        self.angle_offsets = [list(angles) for angles in self.current_angles]

    def process(self, raw_data_list):
        """
        主運算核心
        :param raw_data_list: 從 SerialManager 收到的二維列表
                              例如: [[ax,ay,az,gx,gy,gz], ..., [ax,ay,az,gx,gy,gz,mx,my,mz]]
        :return: 計算後的角度列表 [[p,r,y], [p,r,y]...]
        """
        if raw_data_list is None or len(raw_data_list) == 0:
            return None

        # 1. 計算時間差 dt (用來積分角速度)
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        if dt > 0.1: dt = 0.01 # 避免剛啟動時 dt 過大導致數值暴衝

        # 2. 自動初始化狀態陣列 (只在第一次執行)
        # 這樣你就不用手動設定 "我有幾顆 IMU"
        if not self.is_initialized:
            num_imus = len(raw_data_list)
            self.current_angles = [[0.0, 0.0, 0.0] for _ in range(num_imus)]
            self.angle_offsets = [[0.0, 0.0, 0.0] for _ in range(num_imus)]
            self.is_initialized = True
            print(f"系統自動偵測到 {num_imus} 顆 IMU，運算層初始化完成。")

        final_output = []

        # 3. 逐一處理每一顆 IMU
        for i, raw in enumerate(raw_data_list):
            
            # 判斷這顆是 6 軸還是 9 軸
            # 6 軸長度=6 (Acc+Gyro)
            # 9 軸長度=9 (Acc+Gyro+Mag)
            is_9axis = (len(raw) == 9)

            # --- A. 解析共用數據 (Acc & Gyro) ---
            acc_x, acc_y, acc_z = raw[0], raw[1], raw[2]
            gyro_x, gyro_y, gyro_z = raw[3], raw[4], raw[5]

            # 單位轉換: Gyro 轉 deg/s
            gx_dps = gyro_x / 131.0
            gy_dps = gyro_y / 131.0
            gz_dps = gyro_z / 131.0

            # --- B. 計算 Pitch & Roll (由重力加速度決定) ---
            # 這是絕對角度，不會飄移
            # Pitch (繞 X 轉): atan2(Y, Z)
            acc_roll = math.atan2(acc_y, acc_z) * self.rad_to_deg
            # Roll (繞 Y 轉): atan2(-X, Z)
            acc_pitch = math.atan2(-acc_x, acc_z) * self.rad_to_deg

            # --- C. 互補濾波運算 (融合 Gyro 與 Acc) ---
            # 取出上一次的角度
            prev_p, prev_r, prev_y = self.current_angles[i]

            # 公式: 新角度 = 0.95 * (舊角度 + 陀螺儀轉動) + 0.05 * (加速度計角度)
            new_pitch = self.alpha * (prev_p + gy_dps * dt) + (1 - self.alpha) * acc_pitch
            new_roll  = self.alpha * (prev_r + gx_dps * dt) + (1 - self.alpha) * acc_roll

            # --- D. 計算 Yaw (最關鍵的左右轉) ---
            
            new_yaw = 0.0

            if is_9axis:
                # 【9 軸模式】：使用磁力計修正 Yaw (絕對方位)
                mag_x, mag_y, mag_z = raw[6], raw[7], raw[8]
                
                # 1. 傾斜補償 (Tilt Compensation)
                # 因為磁力計跟著手傾斜，必須先把磁向量「轉」回水平面，才能算出正確的北方
                # 使用剛算出來的 Pitch/Roll (轉成弧度)
                phi = new_roll * self.deg_to_rad
                theta = new_pitch * self.deg_to_rad
                
                # 旋轉矩陣公式 (將磁場投影到水平面)
                X_h = mag_x * math.cos(theta) + mag_y * math.sin(theta) * math.sin(phi) + mag_z * math.sin(theta) * math.cos(phi)
                Y_h = mag_y * math.cos(phi) - mag_z * math.sin(phi)
                
                # 2. 計算電子羅盤方位角
                mag_yaw = math.atan2(Y_h, X_h) * self.rad_to_deg
                
                # 3. 融合 (這裡用簡單的濾波讓數值更平滑，避免磁場跳動)
                # 0.02 代表強烈信任磁力計的長期趨勢，但短期保留 Gyro 的滑順
                # 注意：這裡還可以加入 Gyro 積分來做更高級的融合，但目前這樣就很準了
                yaw_diff = mag_yaw - prev_y
                
                # 處理 360 度迴轉問題 (例如從 359 跳到 1 度)
                if yaw_diff > 180: yaw_diff -= 360
                if yaw_diff < -180: yaw_diff += 360
                
                new_yaw = prev_y + yaw_diff * 0.1 # 0.1 是平滑係數

            else:
                # 【6 軸模式】：原本只能靠 Gyro 積分，會飄移
                # new_yaw = prev_y + (gz_dps * dt)  <-- 舊的寫法
                
                # --- 新增：防飄移魔法 (Yaw Decay) ---
                # 概念：手指很少會一直張開著，所以我們讓 Yaw 慢慢地「自動歸零」
                # 這樣可以抵銷掉積分產生的累積誤差
                
                # 1. 先做標準積分
                integrated_yaw = prev_y + (gz_dps * dt)
                
                # 2. 判斷手指是否正在劇烈運動
                # 如果 Gyro Z 數值很小 (代表手指靜止或緩慢移動)，我們就啟動「自動歸零」
                if abs(gz_dps) < 5.0: # 門檻值: 5度/秒
                    # 每次運算都把角度縮小一點點 (乘以 0.99)
                    # 效果：手指會像是被一條橡皮筋慢慢拉回中間
                    new_yaw = integrated_yaw * 0.99 
                else:
                    # 如果正在快速張開/閉合，就相信 Gyro，不做衰減
                    new_yaw = integrated_yaw

            # --- E. 存檔與輸出 ---
            # 更新狀態
            self.current_angles[i] = [new_pitch, new_roll, new_yaw]

            # 扣除歸零偏差 (Tare)
            final_p = new_pitch - self.angle_offsets[i][0]
            final_r = new_roll  - self.angle_offsets[i][1]
            final_y = new_yaw   - self.angle_offsets[i][2]

            final_output.append([final_p, final_r, final_y])

        return final_output

# =========================================
#   獨立測試區 (顯示所有 IMU 數據)
# =========================================
if __name__ == "__main__":
    from imu9_serial_manager import SerialManager
    import sys # 用來做終端機控制

    # 1. 硬體配置設定
    # 目前: 3顆指節(6軸) + 1顆手掌(9軸)
    # 未來: 改成 [6]*14 + [9] 即可，下面的顯示程式碼完全不用動！
    my_setup = [6] * 14 + [9]
    
    manager = SerialManager(imu_setup=my_setup)
    processor = DataProcessor()

    if manager.connect():
        print("開始測試... (按 Ctrl+C 停止)")
        try:
            while True:
                raw_data = manager.read_data()
                if raw_data:
                    angles = processor.process(raw_data)
                    
                    # --- 動態顯示邏輯 ---
                    
                    # 建立要輸出的字串緩衝區
                    output = []
                    output.append(f"=== 系統監控: 共 {len(angles)} 顆 IMU ===")
                    
                    # 使用 enumerate 自動取得索引 (i) 和數據 (ang)
                    for i, ang in enumerate(angles):
                        p, r, y = ang[0], ang[1], ang[2]
                        
                        # 簡單判斷：如果是最後一顆，標記為手掌，其他是手指
                        is_last = (i == len(angles) - 1)
                        name = "手掌(9軸)" if is_last else f"指節 {i:02d}  "
                        
                        # 格式化字串: 保持對齊
                        info = f"IMU{i:02d} [{name}] : Pitch={p:6.1f} | Roll={r:6.1f} | Yaw={y:6.1f}"
                        output.append(info)
                    
                    output.append("========================================")
                    
                    # --- 顯示技巧 ---
                    # 這行指令會把游標移回畫面左上角，覆蓋舊內容 (類似儀表板效果)
                    # 如果在某些 Windows cmd 沒效果，會變成一直往下印，也不影響閱讀
                    sys.stdout.write("\033[H\033[J") 
                    print("\n".join(output))
                    
        except KeyboardInterrupt:
            print("\n停止測試")
            manager.close()