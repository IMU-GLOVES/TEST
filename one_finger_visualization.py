# imu_vpython_finger_roll_yaw_width_long.py
from vpython import canvas, box, vector, color, rate
import serial
import math

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

# 建立三段 box，長邊沿 width (手指長度)
imu1_box = box(pos=vector(0,0,0), length=0.2, height=1.25, width=BASE_LEN, color=color.red)
imu2_box = box(pos=vector(0,0,BASE_LEN), length=0.18, height=1.22, width=MID_LEN, color=color.green)
imu3_box = box(pos=vector(0,0,BASE_LEN+MID_LEN), length=0.15, height=1.2, width=TIP_LEN, color=color.blue)

for b in (imu1_box, imu2_box, imu3_box):
    b.axis = vector(0,1,0)  # 長邊方向沿 y 軸直立
    b.up   = vector(0,0,1)  # up 指向 z 軸

# ==== 四元數工具函式 ====
def quat_normalize(q):
    w,x,y,z = q
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n == 0: return (1.0,0.0,0.0,0.0)
    return (w/n, x/n, y/n, z/n)

def quat_mul(a,b):
    aw,ax,ay,az = a
    bw,bx,by,bz = b
    w = aw*bw - ax*bx - ay*by - az*bz
    x = aw*bx + ax*bw + ay*bz - az*by
    y = aw*by - ax*bz + ay*bw + az*bx
    z = aw*bz + ax*by - ay*bx + az*bw
    return (w,x,y,z)

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
    return (w,x,y,z)

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
    box_obj.axis = vector(R[0][1],R[1][1],R[2][1])  # 長邊沿 y 軸
    box_obj.up   = vector(R[0][2],R[1][2],R[2][2])

# ==== Serial ====
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# ==== smoothing ====
SMOOTH_ALPHA = 0.6
prev_q1 = prev_q2 = prev_q3 = (1.0,0.0,0.0,0.0)
def smooth_quat(prev,new):
    pw,px,py,pz = prev
    nw,nx,ny,nz = new
    q = (SMOOTH_ALPHA*nw+(1-SMOOTH_ALPHA)*pw,
         SMOOTH_ALPHA*nx+(1-SMOOTH_ALPHA)*px,
         SMOOTH_ALPHA*ny+(1-SMOOTH_ALPHA)*py,
         SMOOTH_ALPHA*nz+(1-SMOOTH_ALPHA)*pz)
    return quat_normalize(q)

# ==== 主迴圈 ====
try:
    while True:
        rate(100)
        line = ser.readline().decode('utf-8','ignore').strip()
        if not line: continue
        parts = line.replace(" ","").split(',')
        if len(parts)<12: continue
        try:
            vals = [float(p) for p in parts[:12]]
        except: continue

        # q1:手掌端, q2:中節, q3:指尖
        q1 = quat_normalize(vals[0:4])
        q2 = quat_normalize(vals[4:8])
        q3 = quat_normalize(vals[8:12])

        # smoothing
        q1 = smooth_quat(prev_q1,q1)
        q2 = smooth_quat(prev_q2,q2)
        q3 = smooth_quat(prev_q3,q3)
        prev_q1, prev_q2, prev_q3 = q1,q2,q3

        # === DoF 約束 ===
        # q1c = constrain_quat(q1,'yaw_roll')                     # base yaw+roll
        # q2c = constrain_quat(q2,'roll_only',yaw_ref=quat_to_euler(q1c)[0])  # mid roll only
        # q3c = constrain_quat(q3,'roll_only',yaw_ref=quat_to_euler(q1c)[0])  # tip roll only

        q1c = constrain_quat(q1,'roll_only')   # 只保留 roll，取消 yaw
        q2c = constrain_quat(q2,'roll_only', yaw_ref=0)
        q3c = constrain_quat(q3,'roll_only', yaw_ref=0)

        # === 合成世界 quaternion ===
        q1q2 = quat_mul(q1c,q2c)
        q1q2q3 = quat_mul(q1q2,q3c)

        # === 套用到 VPython ===
        apply_quat(imu1_box,q1c)
        apply_quat(imu2_box,q1q2)
        apply_quat(imu3_box,q1q2q3)

        # === 更新位置連接三節 ===
        imu2_box.pos = imu1_box.pos + imu1_box.axis * imu1_box.width
        imu3_box.pos = imu2_box.pos + imu2_box.axis * imu2_box.width

except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    try: ser.close()
    except: pass
