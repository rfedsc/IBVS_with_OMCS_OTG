import numpy as np
from math import sin, cos, radians

def euler_to_quaternion(roll, pitch, yaw):
    """将欧拉角（roll, pitch, yaw）转换为四元数"""
    # 角度转弧度
    roll = radians(roll)
    pitch = radians(pitch)
    yaw = radians(yaw)

    # 计算每个旋转角度的四元数
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    # 四元数乘法：q = q_x * q_y * q_z
    q_x = [sr, 0, 0, cr]
    q_y = [0, sp, 0, cp]
    q_z = [0, 0, sy, cy]

    # 计算四元数乘法
    q = quaternion_multiply(quaternion_multiply(q_z, q_y), q_x)
    
    return q

def quaternion_multiply(q1, q2):
    """计算两个四元数的乘法"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    ]

# 示例：欧拉角 (roll=30, pitch=45, yaw=60)
roll = -90
pitch = 0
yaw = -90

q = euler_to_quaternion(roll, pitch, yaw)
print(f"Quaternion: {q}")

