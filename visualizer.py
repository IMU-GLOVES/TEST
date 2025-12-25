from vpython import canvas, box, vector, color, rate, distant_light, cylinder, arrow, radians, cross
from imu9_serial_manager import SerialManager 
from imu9_data_to_math import DataProcessor
import time
import sys

# ==========================================
#   1. 初始化連線
# ==========================================
my_setup = [6] * 3 + [9]
manager = SerialManager(imu_setup=my_setup)
processor = DataProcessor()

if not manager.connect():
    print("❌ 連線失敗")
else:
    print("✔ 連線成功 (平放模式)")

print("請將手掌【平放於桌面】並指向前方，然後按 'R' 鍵歸零。")

# ==========================================
#   2. 場景設定
# ==========================================
scene = canvas(
    title='Flat IMU -> Upright Visualizer',
    width=800, height=600,
    background=color.black,
    center=vector(0, 0, 0)
)
distant_light(direction=vector(1, 1, 1), color=color.white)

# 視覺上：我們依然畫一個「直立」的手掌
initial_up = vector(0, 1, 0)           # 手指朝上
initial_face = vector(-1, 0, 1).norm() # 掌心朝左前

palm = cylinder(
    pos=vector(0, 0, 0),
    axis=initial_face * 0.5, 
    radius=4,
    color=color.gray(0.9)
)
palm.up = initial_up 

# 輔助箭頭
arrow_len = 6
arrow_x = arrow(pos=palm.pos, axis=vector(arrow_len,0,0), color=color.red, shaftwidth=0.2) # 側邊
arrow_y = arrow(pos=palm.pos, axis=vector(0,arrow_len,0), color=color.green, shaftwidth=0.2) # 手指方向
arrow_z = arrow(pos=palm.pos, axis=vector(0,0,arrow_len), color=color.blue, shaftwidth=0.2) # 掌心法線

# ==========================================
#   3. 歸零
# ==========================================
def on_key(evt):
    if evt.key == 'r' or evt.key == 'R':
        processor.reset_calibration()
        print("\n>>> 已歸零 (Tare) - 請確保歸零時手是平放的！ <<<")

scene.bind('keydown', on_key)

# ==========================================
#   4. 主迴圈
# ==========================================
while True:
    rate(60)
    # --- [新增] 防延遲機制 (Anti-Lag) ---
    # 檢查緩衝區：如果堆積了超過 2 包數據量的資料，就瘋狂讀取並丟棄
    # 這樣保證我們等一下讀到的 raw_data 是最新鮮的
    
    # 只有當連線正常時才執行清空
    if manager.ser and manager.ser.is_open:
        waiting_bytes = manager.ser.in_waiting
        packet_size = manager.total_bytes
        
        # 如果堆積超過 2 包 (代表開始延遲了)
        if waiting_bytes > packet_size * 2:
            # 計算要丟掉幾包，保留最後一包就好
            num_to_skip = waiting_bytes // packet_size
            for _ in range(num_to_skip):
                manager.read_data() # 讀取但不處理 (丟掉舊資料)

    raw_data = manager.read_data()
    
    if raw_data:
        angles = processor.process(raw_data)
        
        # 取得 IMU 數據 (變數是基於平放物理定義的)
        # p=Pitch(繞Y), r=Roll(繞X), y=Yaw(繞Z)
        p, r, y = angles[-1]

        msg = f"Pitch(自轉Y):{p:5.1f} | Roll(點頭X):{r:5.1f} | Yaw(揮手Z):{y:5.1f}"
        sys.stdout.write(f"\r{msg}")
        sys.stdout.flush()

        # --- 1. 重置模型姿態 ---
        palm.axis = initial_face * 0.5
        palm.up = initial_up

        # --- 2. 計算模型當下的軸向 (Local Axes) ---
        # 這是為了讓旋轉跟隨模型當下的狀態
        model_Y = palm.up.norm()   # 綠色 (手指軸)
        model_Z = palm.axis.norm() # 藍色 (掌心軸)
        model_X = cross(model_Z, model_Y).norm() # 紅色 (側面軸)

        # --- 3. 映射邏輯 (Mapping) ---
        # 這裡決定了 "IMU 的數據" 驅動 "模型的哪個軸"
        
        # A. IMU Yaw (繞 Z 軸轉) -> 對應手掌左右揮
        # 在直立模型上，左右揮通常是繞著掌心軸 (Z) 或者是絕對垂直軸 (World Y)
        # 讓我們試試看繞著模型的掌心軸 (Z)
        palm.rotate(angle=radians(-y), axis=model_Z)
        
        # B. IMU Pitch (繞 Y 軸轉) -> 對應手掌自轉
        # 因為你平放時 Y 軸朝指尖，所以這代表自轉
        # 對應模型的 Y 軸
        palm.rotate(angle=radians(p), axis=model_Y)

        # C. IMU Roll (繞 X 軸轉) -> 對應手腕壓手
        # 因為你平放時 X 軸朝側邊，所以這代表壓手
        # 對應模型的 X 軸
        palm.rotate(angle=radians(r), axis=model_X)

        # --- 4. 更新箭頭 ---
        arrow_x.axis = model_X * arrow_len
        arrow_y.axis = model_Y * arrow_len
        arrow_z.axis = model_Z * arrow_len