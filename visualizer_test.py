from vpython import canvas, box, vector, color, rate, distant_light, cylinder, arrow, radians, cross, sphere
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
#   2. 場景與模型設定
# ==========================================
scene = canvas(
    title='IMU Hand Visualizer with Finger',
    width=800, height=600,
    background=color.black,
    center=vector(0, 5, 0) # 視角稍微往上移一點，方便看手指
)
distant_light(direction=vector(1, 1, 1), color=color.white)

# --- A. 定義初始方向向量 (Rest Pose) ---
# 這是模型「還沒轉動前」的標準姿態
initial_up = vector(0, 1, 0)            # 手指朝上 (Y軸)
initial_face = vector(-1, 0, 1).norm()  # 掌心朝左前 (45度)
# 計算側邊向量 (用來定位食指要在左邊還是右邊)
# Cross product: Face x Up = Right Side
initial_side = cross(initial_face, initial_up).norm() 

# --- B. 建立模型元件 ---

# 1. 手掌 (Palm)
palm_radius = 4
palm = cylinder(
    pos=vector(0, 0, 0),
    axis=initial_face * 0.5, # 厚度方向
    radius=palm_radius,
    color=color.gray(0.9)
)

# 2. 關節 (Joint) - 小圓球
# 演算法：從掌心出發，往上走一段(y)，往旁邊走一段(side)
# 這樣關節就會黏在手掌的右上角位置
joint_radius = 0.8
joint_offset_up = 3.5    # 往上多遠 (接近手掌半徑)
joint_offset_side = 1.5  # 往旁邊多遠 (食指位置)

# 計算關節的「初始絕對位置」
joint_initial_pos = vector(0,0,0) + (initial_up * joint_offset_up) + (initial_side * joint_offset_side)

joint = sphere(
    pos=joint_initial_pos,
    radius=joint_radius,
    color=vector(0.95, 0.92, 0.85) # 米白色 (Beige)
)

# 3. 食指指根 (Finger Base) - 圓柱體
finger_len = 6
finger_radius = 0.9

# 食指的初始設定
# pos: 圓柱的底部，設定在關節的位置
# axis: 圓柱的長度方向，設定為初始向上
finger = cylinder(
    pos=joint_initial_pos,
    axis=initial_up * finger_len,
    radius=finger_radius,
    color=palm.color # 跟手掌一樣顏色
)

# 保存手指的初始向量，方便迴圈中重置
finger_initial_axis = initial_up * finger_len


# 輔助箭頭
arrow_len = 6
arrow_x = arrow(pos=palm.pos, axis=vector(arrow_len,0,0), color=color.red, shaftwidth=0.2) 
arrow_y = arrow(pos=palm.pos, axis=vector(0,arrow_len,0), color=color.green, shaftwidth=0.2)
arrow_z = arrow(pos=palm.pos, axis=vector(0,0,arrow_len), color=color.blue, shaftwidth=0.2)

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
    
    # --- 防延遲機制 ---
    if manager.ser and manager.ser.is_open:
        waiting_bytes = manager.ser.in_waiting
        packet_size = manager.total_bytes
        if waiting_bytes > packet_size * 2:
            num_to_skip = waiting_bytes // packet_size
            for _ in range(num_to_skip):
                manager.read_data() 

    raw_data = manager.read_data()
    
    if raw_data:
        angles = processor.process(raw_data)
        p, r, y = angles[-1] # Pitch, Roll, Yaw

        msg = f"Pitch:{p:5.1f} | Roll:{r:5.1f} | Yaw:{y:5.1f}"
        sys.stdout.write(f"\r{msg}")
        sys.stdout.flush()

        # ==========================================
        #   關鍵邏輯：重置 -> 旋轉
        # ==========================================

        # --- 1. 重置所有元件到「初始狀態」 ---
        palm.pos = vector(0,0,0)
        palm.axis = initial_face * 0.5
        palm.up = initial_up
        
        joint.pos = joint_initial_pos
        
        finger.pos = joint_initial_pos
        finger.axis = finger_initial_axis

        # --- 2. 計算旋轉軸 (跟隨 Palm 目前的狀態) ---
        # 由於剛剛重置了，目前的軸就是初始軸
        current_up = palm.up.norm()     # Y軸方向
        current_face = palm.axis.norm() # Z軸方向
        current_side = cross(current_face, current_up).norm() # X軸方向

        # --- 3. 施加旋轉 (Mapping) ---
        
        # 定義一個 helper function 來一次旋轉三個物件
        # 參數 origin=vector(0,0,0) 非常重要！
        # 這代表大家都是繞著「手掌中心」轉，這樣相對位置才不會散掉
        def rotate_whole_hand(angle_rad, rot_axis):
            origin_point = vector(0,0,0) # 旋轉中心：手掌中心
            palm.rotate(angle=angle_rad, axis=rot_axis, origin=origin_point)
            joint.rotate(angle=angle_rad, axis=rot_axis, origin=origin_point)
            finger.rotate(angle=angle_rad, axis=rot_axis, origin=origin_point)

        # A. Yaw (揮手) -> 繞著手掌面法線 (Z)
        rotate_whole_hand(radians(-y), current_face)
        
        # 旋轉後軸向改變了，需要重新抓取最新的軸向給下一次旋轉用
        # (雖然對於小角度順序影響不大，但嚴謹來說要這樣做)
        # 但 VPython 的 rotate 會自動更新 palm.up 和 palm.axis，所以我們可以直接用 palm 的屬性
        
        # B. Pitch (自轉) -> 繞著手指方向 (Y)
        rotate_whole_hand(radians(p), palm.up.norm())

        # C. Roll (壓手) -> 繞著側面 (X)
        # 側面軸需要算一下 cross product
        current_side_axis = cross(palm.axis.norm(), palm.up.norm()).norm()
        rotate_whole_hand(radians(r), current_side_axis)

        # ==========================================
        #   未來擴充：如果要動手指，寫在這裡
        # ==========================================
        # 這裡的旋轉就要以 joint.pos 為 origin 了
        # finger.rotate(angle=..., axis=..., origin=joint.pos)
        
        # --- 4. 更新箭頭 ---
        arrow_x.axis = current_side_axis * arrow_len
        arrow_y.axis = palm.up.norm() * arrow_len
        arrow_z.axis = palm.axis.norm() * arrow_len