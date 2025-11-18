# imu_vpython_finger_roll_yaw_width_long_vertical.py
from vpython import canvas, box, vector, color, rate
import serial
import math
import numpy as np
from ahrs.filters import Madgwick

# ==== 串口設定 ====
SERIAL_PORT = "COM4"
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
    if n == 0: return np.array([1.0,0.0,0.0,0.0])
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

# ==== Madgwick 初始化 ====
madgwick1 = Madgwick()
madgwick2 = Madgwick()
madgwick3 = Madgwick()

# ==== 主迴圈 ====
try:
    while True:
        rate(100)
        line = ser.readline().decode('utf-8','ignore').strip()
        if not line: continue
        parts = line.replace(" ","").split(',')
        if len(parts)<12: continue
        try:
            vals = [float(p) for p in parts[:18]]
        except: continue

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
        prev_q1, prev_q2, prev_q3 = q1,q2,q3

        # ---- DoF 約束 ----
        q1c = constrain_quat(q1,'roll_only')
        q2c = constrain_quat(q2,'roll_only', yaw_ref=0)
        q3c = constrain_quat(q3,'roll_only', yaw_ref=0)

        # ---- 合成世界 quaternion ----
        q1q2 = quat_mul(q1c,q2c)
        q1q2q3 = quat_mul(q1q2,q3c)

        # ---- 套用到 VPython ----
        apply_quat(imu1_box,q1c)
        apply_quat(imu2_box,q1q2)
        apply_quat(imu3_box,q1q2q3)

        # ---- 更新位置連接三節 (垂直) ----
        imu2_box.pos = imu1_box.pos + imu1_box.axis * imu1_box.height
        imu3_box.pos = imu2_box.pos + imu2_box.axis * imu2_box.height

except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    try: ser.close()
    except: pass
