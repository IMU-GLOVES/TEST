from vpython import canvas, vector, color, rate, distant_light, cylinder, sphere, arrow, radians, cross
from imu9_serial_manager import SerialManager 
from imu9_data_to_math import DataProcessor
import sys

# ==========================================
#   Class: 手部組件 (包含 圓頂關節 + 指骨圓柱)
# ==========================================
class HandSegment:
    def __init__(self, name, parent, length, radius, gap,
                  imu_index, initial_axis, initial_up,
                    is_palm=False, is_thumb=False,pos_offset=vector(0,0,0),
                 allow_pitch=True, allow_roll=True, allow_yaw=True):
        """
        :param name: 組件名稱
        :param parent: 父物件 (HandSegment)，如果是掌心則填 None
        :param length: 骨頭長度
        :param radius: 骨頭粗細
        :param gap: 與父物件之間的空隙距離
        :param imu_index: 對應 data 陣列中的第幾個索引 (0, 1, 2, 3...)
        :param initial_axis: 初始指向向量
        :param initial_up: 初始上方向量
        :param is_palm: 是否為掌心 (決定是否隱藏關節球)
        :param is_palm: 是否為掌心 (把拇指的移動邏輯單獨拉出來)
        """
        self.name = name
        self.parent = parent
        self.length = length
        self.gap = gap
        self.imu_index = imu_index
        self.pos_offset = pos_offset  # <--- [新增] 把偏移量存起來
        # [新增] 把開關存起來
        self.allow_pitch = allow_pitch # 是否允許自轉/外展 (Y軸)
        self.allow_roll = allow_roll   # 是否允許彎曲/點頭 (X軸)
        self.allow_yaw = allow_yaw     # 是否允許左右揮/扭動 (Z軸)
        
        # 初始狀態設定 (用於歸零與重置)
        self.initial_axis = initial_axis.norm()
        self.initial_up = initial_up.norm()
        self.is_palm = is_palm
        self.is_thumb = is_thumb

        # --- 建立視覺物件 ---
        # 1. 關節球 (Joint Sphere) - 作為旋轉軸心的視覺裝飾
        #    它的位置就是這個物件的 pos (原點)
        sphere_radius = radius # 讓關節跟骨頭一樣粗，或者稍微大一點 (radius * 1.1)
        if is_palm:
            sphere_radius = 0 # 掌心通常隱藏關節球
            
        self.visual_joint = sphere(
            pos=vector(0,0,0), 
            radius=sphere_radius,
            color=color.orange
        )

        # 2. 指骨 (Bone Cylinder)
        #    它的起點也在 pos (原點)，延伸出去
        self.visual_bone = cylinder(
            pos=vector(0,0,0),
            axis=self.initial_axis * self.length,
            radius=radius,
            color=color.gray(0.9)
        )
        # 設定上方軸，確保旋轉基準正確
        self.visual_bone.up = self.initial_up

    def update(self, all_imu_data):
        """
        每一幀呼叫此函式來更新位置與角度
        """
        # --- 1. 取得數據 ---
        # 根據 imu_index 從總資料中抓取屬於我的那一組
        if self.imu_index < len(all_imu_data):
            my_data = all_imu_data[self.imu_index]
        else:
            my_data = (0, 0, 0) # 如果還沒接那麼多顆，預設 0
            
        p, r, y = my_data

        # --- 2. 計算位置 (Forward Kinematics) ---
        # 這是父子連動的核心：我的位置 = 爸爸的位置 + 爸爸的長度 + 空隙
        
        if self.parent is None:
            # A. 如果我是掌心 (Root)，位置固定在畫面中心
            current_start_pos = vector(0, 0, 0)
        else:
            # --- [新增/修改] 智慧型跟隨演算法 ---
            
            # 1. 取得父親的三軸 (因為偏移是要相對於父親的身體)
            parent_bone = self.parent.visual_bone
            parent_Z = parent_bone.axis.norm()         # 父親的前方 (軸向)
            parent_Y = parent_bone.up.norm()           # 父親的上方
            parent_X = cross(parent_Z, parent_Y).norm()# 父親的側方 (右邊)

            # 2. 算出父親的尾巴在哪裡
            parent_tip = parent_bone.pos + parent_bone.axis
            
            # 3. 計算空隙向量 (沿著父親前方延伸)
            gap_vec = parent_Z * self.gap

            # 4. [關鍵] 計算側邊偏移向量 (讓偏移量跟著父親的軸向轉)
            # offset.x 代表往父親的右邊移，offset.y 往上，offset.z 往前
            offset_vec = (parent_X * self.pos_offset.x) + \
                         (parent_Y * self.pos_offset.y) + \
                         (parent_Z * self.pos_offset.z)

            # 5. 總結算出我的起點
            current_start_pos = parent_tip + gap_vec + offset_vec

        # --- 3. 更新視覺物件的位置 ---
        # 讓關節球與骨頭起點都移動到計算出的位置
        self.visual_joint.pos = current_start_pos
        self.visual_bone.pos = current_start_pos

        # --- 4. 執行旋轉 (Rotation) ---
        # 這裡的邏輯是：先重置回初始狀態，再依照 IMU 數據旋轉
        # 這樣做是因為 IMU 給的是絕對角度 (相對於校正時的狀態)
        
        # A. 重置骨頭方向
        self.visual_bone.axis = self.initial_axis * self.length
        self.visual_bone.up = self.initial_up
        
        # B. 建立局部座標系 (Local Coordinate System)
        # 這是為了讓旋轉軸正確 (例如 Pitch 是繞著手指的側面轉)
        model_Y = self.visual_bone.up.norm()     # 上方
        model_Z = self.visual_bone.axis.norm()   # 前方 (骨頭指向)
        model_X = cross(model_Z, model_Y).norm() # 側方 (右方)

        # C. 分流對接邏輯 (根據部件類型決定旋轉軸)
        if self.is_palm:
            # 掌心的邏輯
            if self.allow_yaw:   self.visual_bone.rotate(angle=radians(-y), axis=model_Z)
            if self.allow_pitch: self.visual_bone.rotate(angle=radians(p),  axis=model_Y) # 繞上軸擺動
            if self.allow_roll:  self.visual_bone.rotate(angle=radians(r),  axis=model_X)
        else:
            if self.is_thumb:
                #把拇指的移動修正
                if self.allow_yaw:   self.visual_bone.rotate(angle=radians(y), axis=model_X) 
                if self.allow_pitch: self.visual_bone.rotate(angle=radians(p),  axis=model_Z) # 繞生長軸擺動
                if self.allow_roll:  self.visual_bone.rotate(angle=radians(r),  axis=model_Y)
            else:
                # 指節的邏輯 (對接方式與掌心不同)
                # 我們把 IMU 的 Pitch (p) 改為驅動指節的 model_Z (因為指節的 Z 朝上)
                if self.allow_yaw:   self.visual_bone.rotate(angle=radians(-y), axis=model_Y) 
                if self.allow_pitch: self.visual_bone.rotate(angle=radians(p),  axis=model_Z) # 繞生長軸擺動
                if self.allow_roll:  self.visual_bone.rotate(angle=radians(r),  axis=model_X)


# ==========================================
#   主程式區塊
# ==========================================

# 1. 硬體設定
# 根據你的描述：[Ch0(指根), Ch1(指中), Ch2(指尖), Ch7(掌心)]
my_setup = [6] * 8 + [9] 
manager = SerialManager(imu_setup=my_setup)
processor = DataProcessor()

if not manager.connect():
    print("❌ 連線失敗")
else:
    print("✔ 連線成功 (請平放手掌)")

print("請將手掌【平放於桌面】並指向前方，然後按 'R' 鍵歸零。")

# 2. 場景設定
scene = canvas(
    title='Full Index Finger Visualizer',
    width=1000, height=800,
    background=color.black,
    # center=vector(-4, 5, 0.5) # 把鏡頭中心稍微往右移，因為手指會往右長
)
scene.camera.pos = vector(-8, 5, 3)
scene.forward = vector(3, -1.5, -1)
distant_light(direction=vector(1, 1, 1), color=color.white)

# 3. 定義初始向量
# 掌心：稍微傾斜 (你的原始設定)
palm_init_axis = vector(-1, 0, 0).norm()
palm_init_up = vector(0, 1, 0)

# 手指：預設跟掌心同方向，或者你可以設為 vector(1,0,0) 讓它直直向右
# finger_init_axis = palm_init_axis 
# finger_init_up = palm_init_up
# --- 修改後 (強制指尖朝上) ---
# vector(0, 1, 0) 代表世界座標的「正上方」
finger_init_axis = vector(0, 1, 0) 

# 同時建議修改 finger_init_up (指甲面朝向)
# 如果手指朝上(Y)，那指甲面通常朝向螢幕(Z)或朝向自己
finger_init_up = vector(1, 0, 0)

# ==========================================
#   4. 建立手部物件 (The Hand Construction)
# ==========================================

# [索引 8] 掌心 (Palm) - 連接 Mux2 Ch7
# 1. 掌心 (Palm)
# 它是老大，parent=None
palm = HandSegment(
    name="Palm", 
    parent=None, 
    length=0.5, radius=4, gap=0, imu_index=18, # 假設最後一顆是掌心
    initial_axis=palm_init_axis, 
    initial_up=palm_init_up, 
    is_palm=True,
    allow_roll=False,
    allow_pitch=False,
    allow_yaw=False    # 禁止揮手 (Z軸鎖定)
)

# -----------------------------------------------------------
# [拇指] (Index / Index1)
# -----------------------------------------------------------
# [索引 1] 拇指指根 - 連接 Mux1 Ch1
index1_base = HandSegment(
    name="Index1_Base", 
    parent=palm,          # 變數名稱
    length=2, radius=0.6, 
    gap=0.5, 
    imu_index=1, 
    is_thumb=True,
    # 確保這裡是寫 finger_init_axis，而不是 palm_init_axis
    initial_axis=finger_init_axis,
    # 這裡也要確保是用新的 finger_init_up
    initial_up=finger_init_up,
    # Offset: 食指在掌心右側，所以 X 設為 2 (數值請依畫面調整)
    pos_offset=vector(-4, 0, 0),
    allow_pitch=False    # 禁止自轉 (手指不會像螺絲起子一樣轉)
)

# [索引 0] 拇指指尖 - 連接 Mux1 Ch0
index1_top = HandSegment(
    name="Index1_Top", 
    parent=index1_base,   # 接在 index1_base 後面
    length=2, radius=0.5, 
    gap=0.3, 
    imu_index=0, 
    is_thumb=True,
    initial_axis=finger_init_axis, initial_up=finger_init_up,
    pos_offset=vector(0, 0, 0), # 接龍，不需要偏移
    allow_yaw=False,    # 鎖
    allow_pitch=False  # 鎖 (指中關節是樞紐關節，不能左右張開)
)

# -----------------------------------------------------------
# [食指] (Index / Index2)
# -----------------------------------------------------------
# [索引 4] 食指指根 - 連接 Mux1 Ch4
index2_base = HandSegment(
    name="Index2_Base", 
    parent=palm,          # 變數名稱
    length=2, radius=0.6, 
    gap=0.5, 
    imu_index=4,
    # 確保這裡是寫 finger_init_axis，而不是 palm_init_axis
    initial_axis=finger_init_axis, 
    
    # 這裡也要確保是用新的 finger_init_up
    initial_up=finger_init_up,
    # Offset: 食指在掌心右側，所以 X 設為 2 (數值請依畫面調整)
    pos_offset=vector(-1.5, 3.7, 0),
    allow_pitch=False    # 禁止自轉 (手指不會像螺絲起子一樣轉)
)

# [索引 3] 食指指中 - 連接 Mux1 Ch3
index2_mid = HandSegment(
    name="Index2_Mid", 
    parent=index2_base,   # 接在 index2_base 後面
    length=2, radius=0.5, 
    gap=0.3, 
    imu_index=3,
    initial_axis=finger_init_axis, initial_up=finger_init_up,
    pos_offset=vector(0, 0, 0), # 接龍，不需要偏移
    allow_yaw=False,    # 鎖
    allow_pitch=False  # 鎖 (指中關節是樞紐關節，不能左右張開)
)

# [索引 2] 食指指尖 - 連接 Mux1 Ch2
index2_top = HandSegment(
    name="Index2_Top", 
    parent=index2_mid,   # 接在 index2_mid 後面
    length=2, radius=0.5, 
    gap=0.3, 
    imu_index=2,
    initial_axis=finger_init_axis, initial_up=finger_init_up,
    pos_offset=vector(0, 0, 0), # 接龍，不需要偏移
    allow_yaw=False,    # 鎖
    allow_pitch=False  # 鎖
)

# -----------------------------------------------------------
# [中指] (Index / Index3)
# -----------------------------------------------------------
# [索引 7] 中指指根 - 連接 Mux1 Ch7
index3_base = HandSegment(
    name="Index3_Base", 
    parent=palm,          # 變數名稱
    length=2, radius=0.6, 
    gap=0.5, 
    imu_index=7,
    # 確保這裡是寫 finger_init_axis，而不是 palm_init_axis
    initial_axis=finger_init_axis, 
    
    # 這裡也要確保是用新的 finger_init_up
    initial_up=finger_init_up,
    # Offset: 食指在掌心右側，所以 X 設為 2 (數值請依畫面調整)
    pos_offset=vector(0, 4, 0),
    allow_pitch=False    # 禁止自轉 (手指不會像螺絲起子一樣轉)
)

# [索引 6] 中指指中 - 連接 Mux1 Ch6
index3_mid = HandSegment(
    name="Index3_Mid", 
    parent=index3_base,   # 接在 index3_base 後面
    length=2, radius=0.5, 
    gap=0.3, 
    imu_index=6,
    initial_axis=finger_init_axis, initial_up=finger_init_up,
    pos_offset=vector(0, 0, 0), # 接龍，不需要偏移
    allow_yaw=False,    # 鎖
    allow_pitch=False  # 鎖 (指中關節是樞紐關節，不能左右張開)
)

# [索引 5] 中指指尖 - 連接 Mux1 Ch5
index3_top = HandSegment(
    name="Index3_Top", 
    parent=index3_mid,   # 接在 index3_mid 後面
    length=2, radius=0.5, 
    gap=0.3, 
    imu_index=5,
    initial_axis=finger_init_axis, initial_up=finger_init_up,
    pos_offset=vector(0, 0, 0), # 接龍，不需要偏移
    allow_yaw=False,    # 鎖
    allow_pitch=False  # 鎖
)

# -----------------------------------------------------------
# [中指] (Index / Index3)
# -----------------------------------------------------------
# [索引 10] 無名指指根 - 連接 Mux2 Ch2
index4_base = HandSegment(
    name="Index4_Base", 
    parent=palm,          # 變數名稱
    length=2, radius=0.6, 
    gap=0.5, 
    imu_index=10,
    # 確保這裡是寫 finger_init_axis，而不是 palm_init_axis
    initial_axis=finger_init_axis, 
    
    # 這裡也要確保是用新的 finger_init_up
    initial_up=finger_init_up,
    # Offset: 食指在掌心右側，所以 X 設為 2 (數值請依畫面調整)
    pos_offset=vector(1.5, 3.7, 0),
    allow_pitch=False    # 禁止自轉 (手指不會像螺絲起子一樣轉)
)

# [索引 9] 無名指指中 - 連接 Mux2 Ch1
index4_mid = HandSegment(
    name="Index4_Mid", 
    parent=index4_base,   # 接在 index3_base 後面
    length=2, radius=0.5, 
    gap=0.3, 
    imu_index=9,
    initial_axis=finger_init_axis, initial_up=finger_init_up,
    pos_offset=vector(0, 0, 0), # 接龍，不需要偏移
    allow_yaw=False,    # 鎖
    allow_pitch=False  # 鎖 (指中關節是樞紐關節，不能左右張開)
)

# [索引 8] 無名指指尖 - 連接 Mux2 Ch0
index4_top = HandSegment(
    name="Index4_Top", 
    parent=index4_mid,   # 接在 index3_mid 後面
    length=2, radius=0.5, 
    gap=0.3, 
    imu_index=8,
    initial_axis=finger_init_axis, initial_up=finger_init_up,
    pos_offset=vector(0, 0, 0), # 接龍，不需要偏移
    allow_yaw=False,    # 鎖
    allow_pitch=False  # 鎖
)

# 放入清單，順序其實不影響邏輯，因為 update 裡面是看 parent 計算
# 但為了保險起見，我們還是按層級順序放
hand_parts = [palm, index1_base, index1_top,
               index2_base, index2_mid, index2_top,
                index3_base, index3_mid, index3_top,
                index4_base, index4_mid, index4_top]

# ==========================================
#   5. 主迴圈
# ==========================================

def on_key(evt):
    if evt.key == 'r' or evt.key == 'R':
        processor.reset_calibration()
        print("\n>>> 已歸零 (Tare) <<<")

scene.bind('keydown', on_key)

while True:
    rate(60)
    
    # --- 防延遲機制 ---
    if manager.ser and manager.ser.is_open:
        waiting = manager.ser.in_waiting
        pkt_size = manager.total_bytes
        if waiting > pkt_size * 2:
            skip_count = waiting // pkt_size
            for _ in range(skip_count):
                manager.read_data()

    # --- 讀取數據 ---
    raw_data = manager.read_data()
    
    if raw_data:
        # 計算角度
        angles = processor.process(raw_data)
        
        # 顯示掌心數據除錯
        p, r, y = angles[3] # 掌心
        sys.stdout.write(f"\rPalm: P{p:5.1f}|R{r:5.1f}|Y{y:5.1f}  Tip: P{angles[2][0]:5.1f}")
        sys.stdout.flush()

        # --- 更新所有部位 ---
        # 這裡會自動處理連動：Palm 先動 -> Base 抓 Palm 位置 -> Mid 抓 Base 位置...
        for part in hand_parts:
            part.update(angles)