# %%
# imu_vpython_viewer.py
from vpython import canvas, box, vector, color, rate
import serial

# ==== 串口設定 ====
SERIAL_PORT = "COM3"   # <= 如果你不是 COM3，改這裡
BAUD_RATE   = 115200

# ==== Quaternion -> Rotation Matrix ====
def quat_to_matrix(w, x, y, z):
    """
    將四元數 (w, x, y, z) 轉成 3x3 旋轉矩陣
    回傳 [[m00,m01,m02],
          [m10,m11,m12],
          [m20,m21,m22]]
    """
    # 正規化（以防小誤差）
    norm = (w*w + x*x + y*y + z*z) ** 0.5
    if norm == 0:
        return [[1,0,0],[0,1,0],[0,0,1]]
    w /= norm
    x /= norm
    y /= norm
    z /= norm

    m00 = 1 - 2*(y*y + z*z)
    m01 = 2*(x*y - w*z)
    m02 = 2*(x*z + w*y)

    m10 = 2*(x*y + w*z)
    m11 = 1 - 2*(x*x + z*z)
    m12 = 2*(y*z - w*x)

    m20 = 2*(x*z - w*y)
    m21 = 2*(y*z + w*x)
    m22 = 1 - 2*(x*x + y*y)

    return [[m00, m01, m02],
            [m10, m11, m12],
            [m20, m21, m22]]

# ==== VPython 場景設定 ====
scene = canvas(title="IMU Quaternion Viewer (3x MPU6050)",
               width=1000, height=600, center=vector(0,0,0))

# 三顆 IMU 對應三個方塊
imu1_box = box(pos=vector(-2, 0, 0),
               length=1, height=0.2, width=0.5,
               color=color.red)
imu2_box = box(pos=vector(0, 0, 0),
               length=1, height=0.2, width=0.5,
               color=color.green)
imu3_box = box(pos=vector(2, 0, 0),
               length=1, height=0.2, width=0.5,
               color=color.blue)

# 初始 axis / up
for b in (imu1_box, imu2_box, imu3_box):
    b.axis = vector(1,0,0)  # x 軸方向
    b.up   = vector(0,1,0)  # y 軸方向

print(f"Opening serial port {SERIAL_PORT} @ {BAUD_RATE} ...")
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

def update_box_from_quat(box_obj, w, x, y, z):
    """用四元數更新某個方塊的姿態"""
    R = quat_to_matrix(w, x, y, z)

    # 這裡採用：box.axis = 物體的 x 軸，box.up = 物體的 y 軸
    axis_vec = vector(R[0][0], R[1][0], R[2][0])  # 第一欄 = x 軸
    up_vec   = vector(R[0][1], R[1][1], R[2][1])  # 第二欄 = y 軸

    box_obj.axis = axis_vec
    box_obj.up   = up_vec

print("開始接收資料並顯示 3D 姿態 (Ctrl+C 結束)")

while True:
    rate(100)  # 最多 100 Hz 更新速率

    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if not line:
        continue

    # 去掉多餘空白
    line = line.replace(" ", "")
    parts = line.split(',')

    # 期待 12 個數字：3 顆 IMU * 每顆 4 個 quaternion
    if len(parts) < 12:
        # print("資料長度不足:", line)
        continue

    try:
        vals = [float(p) for p in parts[:12]]
    except ValueError:
        # 解析失敗就忽略這一行
        # print("解析錯誤:", line)
        continue

    # 依序取出三顆 IMU 的 quaternion
    # 注意：Arduino 端輸出順序為 q0,q1,q2,q3 = w,x,y,z
    w1, x1, y1, z1 = vals[0:4]
    w2, x2, y2, z2 = vals[4:8]
    w3, x3, y3, z3 = vals[8:12]

    # 更新三個方塊姿態
    update_box_from_quat(imu1_box, w1, x1, y1, z1)
    update_box_from_quat(imu2_box, w2, x2, y2, z2)
    update_box_from_quat(imu3_box, w3, x3, y3, z3)



