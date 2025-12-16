"""增加靜止時畫面穩定性、按 r 鍵校正中立姿勢、以及由 roll 角度重建純彎曲四元數的功能。
# imu_vpython_finger_roll_yaw_width_long_vertical.py 
from vpython import canvas, box, vector, color, rate
import serial
import math
import numpy as np
from ahrs.filters import Madgwick

# ==== 串口設定 ====
SERIAL_PORT = "COM3"
BAUD_RATE   = 115200

# ==== VPython 場景設定 ====
scene = canvas(title="IMU Finger Viewer (3x MPU6050, constrained quats)",
               width=1000, height=600, center=vector(0,0,0),
               forward=vector(1,-0.8,1))  # 側視角

# ==== 三節手指長度 ====
BASE_LEN = 1.0
MID_LEN  = 0.8
TIP_LEN  = 0.6

# 建立三段 box，長邊沿 y 軸（垂直）
imu1_box = box(pos=vector(0,0,0), length=0.2, height=BASE_LEN, width=1.25, color=color.red)
imu2_box = box(pos=vector(0,BASE_LEN,0), length=0.18, height=MID_LEN, width=1.22, color=color.green)
imu3_box = box(pos=vector(0,BASE_LEN+MID_LEN,0), length=0.15, height=TIP_LEN, width=1.2, color=color.blue)

for b in (imu1_box, imu2_box, imu3_box):
    b.axis = vector(0,1,0)  # 長邊沿 y 軸
    b.up   = vector(0,0,1)  # up 指向 z 軸

# ==== 四元數工具函式 ====
def quat_normalize(q):
    q = np.array(q)
    n = np.linalg.norm(q)
    if n == 0:
        return np.array([1.0,0.0,0.0,0.0])
    return q / n

def quat_mul(a,b):
    aw,ax,ay,az = a
    bw,bx,by,bz = b
    w = aw*bw - ax*bx - ay*by - az*bz
    x = aw*bx + ax*bw + ay*bz - az*by
    y = aw*by - ax*bz + ay*bw + az*bx
    z = aw*bz + ax*by - ay*bx + az*bw
    return np.array([w,x,y,z])

def quat_to_matrix(q):
    w,x,y,z = quat_normalize(q)
    m00 = 1 - 2*(y*y + z*z)
    m01 = 2*(x*y - w*z)
    m02 = 2*(x*z + w*y)
    m10 = 2*(x*y + w*z)
    m11 = 1 - 2*(x*x + z*z)
    m12 = 2*(y*z - w*x)
    m20 = 2*(x*z - w*y)
    m21 = 2*(y*z + w*x)
    m22 = 1 - 2*(x*x + y*y)
    return [[m00,m01,m02],[m10,m11,m12],[m20,m21,m22]]

def quat_to_euler(q):
    w,x,y,z = quat_normalize(q)
    siny_cosp = 2*(w*z + x*y)
    cosy_cosp = 1 - 2*(y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    sinp = 2*(w*y - z*x)
    pitch = math.asin(max(-1,min(1,sinp)))
    sinr_cosp = 2*(w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    return yaw,pitch,roll

def euler_to_quat(yaw,pitch,roll):
    cy = math.cos(yaw*0.5); sy = math.sin(yaw*0.5)
    cp = math.cos(pitch*0.5); sp = math.sin(pitch*0.5)
    cr = math.cos(roll*0.5); sr = math.sin(roll*0.5)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return np.array([w,x,y,z])

# ==== 1209 修正: 由加速度估算姿態，用於靜止時校正 ====
def quat_from_acc(acc):

    #使用加速度向量推估 pitch / roll，
    #yaw 無法由重力得到，這裡先設為 0，稍後用原本 yaw 替換回去。

    ax, ay, az = acc
    norm = math.sqrt(ax*ax + ay*ay + az*az)
    if norm == 0:
        return np.array([1.0,0.0,0.0,0.0])
    ax /= norm
    ay /= norm
    az /= norm

    pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
    roll  = math.atan2(ay, az)
    yaw   = 0.0
    return euler_to_quat(yaw, pitch, roll)

# ==== 1209 修正: 校正用 - 四元數共軛 (inverse for unit quats) ====
def quat_conj(q):
    w,x,y,z = quat_normalize(q)
    return np.array([w, -x, -y, -z])

# ==== DoF 限制 ====
def constrain_quat(q, mode, yaw_ref=0.0):
    yaw,pitch,roll = quat_to_euler(q)
    if mode == 'roll_only':
        yaw = yaw_ref
        pitch = 0.0
    elif mode == 'yaw_roll':
        pitch = 0.0
    return euler_to_quat(yaw,pitch,roll)

# ==== 套用到 box ====
def apply_quat(box_obj,q_world):
    R = quat_to_matrix(q_world)
    # 長邊沿 y 軸
    box_obj.axis = vector(R[0][1], R[1][1], R[2][1])
    box_obj.up   = vector(R[0][2], R[1][2], R[2][2])

# ==== Serial ====
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# ==== smoothing ====
SMOOTH_ALPHA = 0.6
prev_q1 = prev_q2 = prev_q3 = np.array([1.0,0.0,0.0,0.0])

def smooth_quat(prev,new):
    prev = np.array(prev)
    new  = np.array(new)
    q = SMOOTH_ALPHA*new + (1-SMOOTH_ALPHA)*prev
    return quat_normalize(q)

# ==== 1209 修正: 顯示用校正 quaternion（初始為 identity）====
calib_q1 = np.array([1.0,0.0,0.0,0.0])
calib_q2 = np.array([1.0,0.0,0.0,0.0])
calib_q3 = np.array([1.0,0.0,0.0,0.0])

# ==== Madgwick 初始化 ====
madgwick1 = Madgwick()
madgwick2 = Madgwick()
madgwick3 = Madgwick()

# ==== 1209 add====
def correct_if_still(q, acc_norm, gyr_norm, acc_vec):
            # 加速度約 1g 且 角速度很小 → 視為靜止
            if 0.95 < acc_norm < 1.05 and gyr_norm < 0.02:
                q_acc = quat_from_acc(acc_vec)
                # 保留原本 yaw，只修正 pitch/roll
                yaw_orig, _, _ = quat_to_euler(q)
                yaw_acc, pitch_acc, roll_acc = quat_to_euler(q_acc)
                q_target = euler_to_quat(yaw_orig, pitch_acc, roll_acc)
                blend = 0.01  # 修正比例（越小越慢）
                return quat_normalize((1-blend)*q + blend*q_target)
            else:
                return q

pressed_key = None

def on_key(evt):
    global pressed_key
    pressed_key = evt.key

scene.bind('keydown', on_key)

# === 角度工具: 取出 roll(繞 X 軸) 角度，回傳 rad 與 deg ===
def quat_to_roll_deg(q):
    w, x, y, z = quat_normalize(q)
    # standard yaw-pitch-roll
    sinr_cosp = 2*(w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)  # rad
    return roll, roll * 180.0 / math.pi

# ====end add====

# ==== 主迴圈 ====
try:
    while True:
        rate(100)
        line = ser.readline().decode('utf-8','ignore').strip()
        if not line:
            continue
        parts = line.replace(" ","").split(',')
        if len(parts) < 18:
            continue
        try:
            vals = [float(p) for p in parts[:18]]
        except:
            continue

        imu1_raw = vals[0:6]
        imu2_raw = vals[6:12]
        imu3_raw = vals[12:18]

        acc1, gyr1 = np.array(imu1_raw[0:3]), np.array(imu1_raw[3:6])
        acc2, gyr2 = np.array(imu2_raw[0:3]), np.array(imu2_raw[3:6])
        acc3, gyr3 = np.array(imu3_raw[0:3]), np.array(imu3_raw[3:6])

        # ---- 更新 quaternion ----
        q1 = madgwick1.updateIMU(prev_q1, gyr1, acc1)
        q2 = madgwick2.updateIMU(prev_q2, gyr2, acc2)
        q3 = madgwick3.updateIMU(prev_q3, gyr3, acc3)

        # ---- smoothing ----
        q1 = smooth_quat(prev_q1, q1)
        q2 = smooth_quat(prev_q2, q2)
        q3 = smooth_quat(prev_q3, q3)

        # ==== 1209 修正: 靜止時自動校正，減少長時間漂移 ====
        acc1_norm = np.linalg.norm(acc1)
        gyr1_norm = np.linalg.norm(gyr1)
        acc2_norm = np.linalg.norm(acc2)
        gyr2_norm = np.linalg.norm(gyr2)
        acc3_norm = np.linalg.norm(acc3)
        gyr3_norm = np.linalg.norm(gyr3)

        q1 = correct_if_still(q1, acc1_norm, gyr1_norm, acc1)
        q2 = correct_if_still(q2, acc2_norm, gyr2_norm, acc2)
        q3 = correct_if_still(q3, acc3_norm, gyr3_norm, acc3)

        # 原本程式：更新 prev（保留）
        prev_q1, prev_q2, prev_q3 = q1, q2, q3

        # ==== 1209 修正: 按 'r' 將目前姿態設為「基準姿態」 ====
        # 在手指伸直、你覺得是 0 度的姿勢按下 r/R，就會以這一刻為新的 neutral。
        if pressed_key:
            if pressed_key in ('r', 'R'):
                calib_q1 = quat_conj(q1)
                calib_q2 = quat_conj(q2)
                calib_q3 = quat_conj(q3)
                print("[1209] Calibration updated: current pose set as neutral")
            pressed_key = None  # 重置避免重複觸發

        # ==== 1209 修正: 套用校正 quaternion，得到顯示用姿態 ====
        q1_disp = quat_mul(calib_q1, q1)
        q2_disp = quat_mul(calib_q2, q2)
        q3_disp = quat_mul(calib_q3, q3)

        # debug: 看三節目前的 roll 角度（已包含 neutral calibration）
        r1_rad, r1_deg = quat_to_roll_deg(q1_disp)
        r2_rad, r2_deg = quat_to_roll_deg(q2_disp)
        r3_rad, r3_deg = quat_to_roll_deg(q3_disp)
        print(f"roll1={r1_deg:.1f}, roll2={r2_deg:.1f}, roll3={r3_deg:.1f}")



        # ---- DoF 約束（套在顯示用 quaternion 上）----
        q1c = constrain_quat(q1_disp,'roll_only')
        q2c = constrain_quat(q2_disp,'roll_only', yaw_ref=0)
        q3c = constrain_quat(q3_disp,'roll_only', yaw_ref=0)

        # ---- 合成世界 quaternion ----
        q1q2   = quat_mul(q1c, q2c)
        q1q2q3 = quat_mul(q1q2, q3c)

        # ---- 套用到 VPython ----
        apply_quat(imu1_box, q1c)
        apply_quat(imu2_box, q1q2)
        apply_quat(imu3_box, q1q2q3)

        # ---- 更新位置連接三節 (垂直) ----
        imu2_box.pos = imu1_box.pos + imu1_box.axis * imu1_box.height
        imu3_box.pos = imu2_box.pos + imu2_box.axis * imu2_box.height

except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    try:
        ser.close()
    except:
        pass

"""

# 增加畫面手指彎取角度偵測 可依真實角度與修正使畫面更符合實際彎曲角度 1209

# imu_vpython_finger_roll_yaw_width_long_vertical.py 
from vpython import canvas, box, vector, color, rate
import serial
import math
import numpy as np
from ahrs.filters import Madgwick

# ==== 串口設定 ====
SERIAL_PORT = "COM3"  #請根據實際情況修改
BAUD_RATE   = 115200

# ==== VPython 場景設定 ====
scene = canvas(title="IMU Finger Viewer (3x MPU6050, constrained quats)",
               width=1000, height=600, center=vector(0,0,0),
               forward=vector(1,-0.8,1))  # 側視角

# ==== 三節手指長度 ====
BASE_LEN = 1.0
MID_LEN  = 0.8
TIP_LEN  = 0.6

# 建立三段 box，長邊沿 y 軸（垂直）
imu1_box = box(pos=vector(0,0,0), length=0.2, height=BASE_LEN, width=1.25, color=color.red)
imu2_box = box(pos=vector(0,BASE_LEN,0), length=0.18, height=MID_LEN, width=1.22, color=color.green)
imu3_box = box(pos=vector(0,BASE_LEN+MID_LEN,0), length=0.15, height=TIP_LEN, width=1.2, color=color.blue)

for b in (imu1_box, imu2_box, imu3_box):
    b.axis = vector(0,1,0)  # 長邊沿 y 軸
    b.up   = vector(0,0,1)  # up 指向 z 軸

# ==== 四元數工具函式 ====
def quat_normalize(q):
    q = np.array(q)
    n = np.linalg.norm(q)
    if n == 0:
        return np.array([1.0,0.0,0.0,0.0])
    return q / n

def quat_mul(a,b):
    aw,ax,ay,az = a
    bw,bx,by,bz = b
    w = aw*bw - ax*bx - ay*by - az*bz
    x = aw*bx + ax*bw + ay*bz - az*by
    y = aw*by - ax*bz + ay*bw + az*bx
    z = aw*bz + ax*by - ay*bx + az*bw
    return np.array([w,x,y,z])

def quat_to_matrix(q):
    w,x,y,z = quat_normalize(q)
    m00 = 1 - 2*(y*y + z*z)
    m01 = 2*(x*y - w*z)
    m02 = 2*(x*z + w*y)
    m10 = 2*(x*y + w*z)
    m11 = 1 - 2*(x*x + z*z)
    m12 = 2*(y*z - w*x)
    m20 = 2*(x*z - w*y)
    m21 = 2*(y*z + w*x)
    m22 = 1 - 2*(x*x + y*y)
    return [[m00,m01,m02],[m10,m11,m12],[m20,m21,m22]]

def quat_to_euler(q):
    w,x,y,z = quat_normalize(q)
    siny_cosp = 2*(w*z + x*y)
    cosy_cosp = 1 - 2*(y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    sinp = 2*(w*y - z*x)
    pitch = math.asin(max(-1,min(1,sinp)))
    sinr_cosp = 2*(w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    return yaw,pitch,roll

def euler_to_quat(yaw,pitch,roll):
    cy = math.cos(yaw*0.5); sy = math.sin(yaw*0.5)
    cp = math.cos(pitch*0.5); sp = math.sin(pitch*0.5)
    cr = math.cos(roll*0.5); sr = math.sin(roll*0.5)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return np.array([w,x,y,z])

# ==== 1209 修正: 由加速度估算姿態，用於靜止時校正 ====
def quat_from_acc(acc):
    # 使用加速度向量推估 pitch / roll，yaw 設為 0
    ax, ay, az = acc
    norm = math.sqrt(ax*ax + ay*ay + az*az)
    if norm == 0:
        return np.array([1.0,0.0,0.0,0.0])
    ax /= norm
    ay /= norm
    az /= norm

    pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
    roll  = math.atan2(ay, az)
    yaw   = 0.0
    return euler_to_quat(yaw, pitch, roll)

# ==== 1209 修正: 校正用 - 四元數共軛 (inverse for unit quats) ====
def quat_conj(q):
    w,x,y,z = quat_normalize(q)
    return np.array([w, -x, -y, -z])

# ==== DoF 限制（目前保留，後面改用 roll-only quaternion）====
def constrain_quat(q, mode, yaw_ref=0.0):
    yaw,pitch,roll = quat_to_euler(q)
    if mode == 'roll_only':
        yaw = yaw_ref
        pitch = 0.0
    elif mode == 'yaw_roll':
        pitch = 0.0
    return euler_to_quat(yaw,pitch,roll)

# ==== 套用到 box ====
def apply_quat(box_obj,q_world):
    R = quat_to_matrix(q_world)
    # 長邊沿 y 軸
    box_obj.axis = vector(R[0][1], R[1][1], R[2][1])
    box_obj.up   = vector(R[0][2], R[1][2], R[2][2])

# ==== Serial ====
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# ==== smoothing ====
SMOOTH_ALPHA = 0.6
prev_q1 = prev_q2 = prev_q3 = np.array([1.0,0.0,0.0,0.0])

def smooth_quat(prev,new):
    prev = np.array(prev)
    new  = np.array(new)
    q = SMOOTH_ALPHA*new + (1-SMOOTH_ALPHA)*prev
    return quat_normalize(q)

# ==== 1209 修正: 顯示用校正 quaternion（初始為 identity）====
calib_q1 = np.array([1.0,0.0,0.0,0.0])
calib_q2 = np.array([1.0,0.0,0.0,0.0])
calib_q3 = np.array([1.0,0.0,0.0,0.0])

# ==== Madgwick 初始化 ====
madgwick1 = Madgwick()
madgwick2 = Madgwick()
madgwick3 = Madgwick()

# ==== 1209 add: 靜止自動修正 ====
def correct_if_still(q, acc_norm, gyr_norm, acc_vec):
    # 加速度約 1g 且 角速度很小 → 視為靜止
    if 0.95 < acc_norm < 1.05 and gyr_norm < 0.02:
        q_acc = quat_from_acc(acc_vec)
        # 保留原本 yaw，只修正 pitch/roll
        yaw_orig, _, _ = quat_to_euler(q)
        yaw_acc, pitch_acc, roll_acc = quat_to_euler(q_acc)
        q_target = euler_to_quat(yaw_orig, pitch_acc, roll_acc)
        blend = 0.01  # 修正比例（越小越慢）
        return quat_normalize((1-blend)*q + blend*q_target)
    else:
        return q

pressed_key = None

def on_key(evt):
    global pressed_key
    pressed_key = evt.key

scene.bind('keydown', on_key)

# === 角度工具: 取出 roll(繞 X 軸) 角度，回傳 rad 與 deg ===
def quat_to_roll_deg(q):
    w, x, y, z = quat_normalize(q)
    sinr_cosp = 2*(w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)  # rad
    return roll, roll * 180.0 / math.pi

# ==== 1210 add: 由 roll 角度建立「只繞 X 軸」的 quaternion ====
def quat_from_roll(roll_rad):
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    # yaw = pitch = 0
    return np.array([cr, sr, 0.0, 0.0])

# ==== 1210 add: 每一節的角度校正參數（之後你可以手動調整）====
flex_gain1 = 4   # base 節 angle scale
flex_gain2 = 4   # middle 節 angle scale
flex_gain3 = 4   # tip 節 angle scale

flex_offset1 = 0.0 # rad，若 0 度還是有一點彎，可微調
flex_offset2 = 0.0
flex_offset3 = 0.0

# ==== 主迴圈 ====
try:
    while True:
        rate(100)
        line = ser.readline().decode('utf-8','ignore').strip()
        if not line:
            continue
        parts = line.replace(" ","").split(',')
        if len(parts) < 18:
            continue
        try:
            vals = [float(p) for p in parts[:18]]
        except:
            continue

        imu1_raw = vals[0:6]
        imu2_raw = vals[6:12]
        imu3_raw = vals[12:18]

        acc1, gyr1 = np.array(imu1_raw[0:3]), np.array(imu1_raw[3:6])
        acc2, gyr2 = np.array(imu2_raw[0:3]), np.array(imu2_raw[3:6])
        acc3, gyr3 = np.array(imu3_raw[0:3]), np.array(imu3_raw[3:6])

        # ---- 更新 quaternion ----
        q1 = madgwick1.updateIMU(prev_q1, gyr1, acc1)
        q2 = madgwick2.updateIMU(prev_q2, gyr2, acc2)
        q3 = madgwick3.updateIMU(prev_q3, gyr3, acc3)

        # ---- smoothing ----
        q1 = smooth_quat(prev_q1, q1)
        q2 = smooth_quat(prev_q2, q2)
        q3 = smooth_quat(prev_q3, q3)

        # ==== 1209 修正: 靜止時自動校正，減少長時間漂移 ====
        acc1_norm = np.linalg.norm(acc1)
        gyr1_norm = np.linalg.norm(gyr1)
        acc2_norm = np.linalg.norm(acc2)
        gyr2_norm = np.linalg.norm(gyr2)
        acc3_norm = np.linalg.norm(acc3)
        gyr3_norm = np.linalg.norm(gyr3)

        q1 = correct_if_still(q1, acc1_norm, gyr1_norm, acc1)
        q2 = correct_if_still(q2, acc2_norm, gyr2_norm, acc2)
        q3 = correct_if_still(q3, acc3_norm, gyr3_norm, acc3)

        # 原本程式：更新 prev（保留）
        prev_q1, prev_q2, prev_q3 = q1, q2, q3

        # ==== 1209 修正: 按 'r' 將目前姿態設為「基準姿態」 ====
        # 在手指伸直、你覺得是 0 度的姿勢按下 r/R，就會以這一刻為新的 neutral。
        if pressed_key:
            if pressed_key in ('r', 'R'):
                calib_q1 = quat_conj(q1)
                calib_q2 = quat_conj(q2)
                calib_q3 = quat_conj(q3)
                print("[1209] Calibration updated: current pose set as neutral")
            pressed_key = None  # 重置避免重複觸發

        # ==== 1209 修正: 套用校正 quaternion，得到顯示用姿態 ====
        q1_disp = quat_mul(calib_q1, q1)
        q2_disp = quat_mul(calib_q2, q2)
        q3_disp = quat_mul(calib_q3, q3)

        # ==== 1210: 由「校正後 roll」重建只彎曲的 quaternion ====
        r1_rad, r1_deg = quat_to_roll_deg(q1_disp)
        r2_rad, r2_deg = quat_to_roll_deg(q2_disp)
        r3_rad, r3_deg = quat_to_roll_deg(q3_disp)

        # 需要看的話可以暫時打開這行：
        print(f"roll1={r1_deg:.1f}, roll2={r2_deg:.1f}, roll3={r3_deg:.1f}")

        # 套用 gain & offset
        r1_cal = flex_gain1 * (r1_rad + flex_offset1)
        r2_cal = flex_gain2 * (r2_rad + flex_offset2)
        r3_cal = flex_gain3 * (r3_rad + flex_offset3)

        # 由彎曲角度建立「純 X 軸」四元數
        q1c = quat_from_roll(r1_cal)
        q2c = quat_from_roll(r2_cal)
        q3c = quat_from_roll(r3_cal)

        # ---- 合成世界 quaternion ----
        q1q2   = quat_mul(q1c, q2c)
        q1q2q3 = quat_mul(q1q2, q3c)

        # ---- 套用到 VPython ----
        apply_quat(imu1_box, q1c)
        apply_quat(imu2_box, q1q2)
        apply_quat(imu3_box, q1q2q3)

        # ---- 更新位置連接三節 (垂直) ----
        imu2_box.pos = imu1_box.pos + imu1_box.axis * imu1_box.height
        imu3_box.pos = imu2_box.pos + imu2_box.axis * imu2_box.height

except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    try:
        ser.close()
    except:
        pass