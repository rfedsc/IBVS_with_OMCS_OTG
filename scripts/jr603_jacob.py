import numpy as np
import roboticstoolbox as rtb
from sympy import symbols

# 设置打印选项，避免科学计数法
np.set_printoptions(suppress=True)

# DH参数
a = [0, 284, -30, 0, 0, 0]  # 链接长度
d = [0, 0, 0, 286.5, 0, 81.5]  # 链接偏移
alpha = [-np.pi/2, 0, np.pi/2, -np.pi/2, np.pi/2, 0]  # 链接扭转角
offset = [0, 0, 0, 0, 0, 0]  # 偏移量

# 定义关节角度
theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta1 theta2 theta3 theta4 theta5 theta6')

# 创建DH 机器人模型
links = []
for i in range(6):
    links.append(rtb.RevoluteDH(a[i], alpha[i], d[i], q=offset[i]))  # 使用 q 代替 theta

# 创建机器人
robot = rtb.DHRobot(links)

# 打印机器人模型
print(robot)

# 计算末端执行器的雅可比矩阵
J = robot.jacob0([0, 0, 0, 0, 0, 0])
print("Jacobian Matrix:")
print(J)

