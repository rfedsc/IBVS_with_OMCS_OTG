import numpy as np
from math import sin, cos, radians

def euler_to_quaternion(roll, pitch, yaw):
    """将欧拉角（roll, pitch, yaw）转换为四元数"""
    # 角度转弧度
    roll = radians(roll)
    pitch = radians(pitch)
    yaw = radians(yaw)

    # 计算旋转矩阵
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    q_w = cy * cp * cr + sy * sp * sr
    q_x = cy * cp * sr - sy * sp * cr
    q_y = sy * cp * sr + cy * sp * cr
    q_z = sy * cp * cr - cy * sp * sr

    return q_x, q_y, q_z, q_w

# 示例：欧拉角 (roll=30, pitch=45, yaw=60)
roll = -90
pitch = 0
yaw = -90

q_x, q_y, q_z, q_w = euler_to_quaternion(roll, pitch, yaw)
print(f"Quaternion: ({q_x}, {q_y}, {q_z}, {q_w})")

