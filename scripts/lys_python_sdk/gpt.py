import numpy as np
import math

np.set_printoptions(suppress=True)

# DH矩阵每列的含义：连杆夹角、连杆长度、连杆偏距、初始关节角
DH = np.mat([[1.5708, 0, 131.22, 0],
             [0, -110.4, 0, -1.5708],
             [0, -96, 0, 0],
             [1.5708, 0, 64.4, -1.5708],
             [-1.5708, 0, 75.05, 1.5708],
             [0, 0, 50.6, 0]])

def transformToMatrix(alpha, a, d, theta):
    """从前一个坐标系到当前坐标系的变换,包括平移和旋转"""
    T1 = np.mat([[1, 0, 0, 0],
                 [0, math.cos(alpha), -math.sin(alpha), 0],
                 [0, math.sin(alpha), math.cos(alpha), 0],
                 [0, 0, 0, 1]])
    
    T2 = np.mat([[1, 0, 0, a],
                 [0, 1, 0, 0],
                 [0, 0, 1, d],
                 [0, 0, 0, 1]])
    
    T3 = np.mat([[math.cos(theta), -math.sin(theta), 0, 0],
                 [math.sin(theta), math.cos(theta), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])
    
    return T1 @ T2 @ T3

def forwardKinematic(DH, j0, j1, j2, j3, j4, j5):
    # 使用 DH 参数中的偏移量加上关节角度
    T0 = transformToMatrix(DH[0, 0], DH[0, 1], DH[0, 2], DH[0, 3] + j0)
    T1 = transformToMatrix(DH[1, 0], DH[1, 1], DH[1, 2], DH[1, 3] + j1)
    T2 = transformToMatrix(DH[2, 0], DH[2, 1], DH[2, 2], DH[2, 3] + j2)
    T3 = transformToMatrix(DH[3, 0], DH[3, 1], DH[3, 2], DH[3, 3] + j3)
    T4 = transformToMatrix(DH[4, 0], DH[4, 1], DH[4, 2], DH[4, 3] + j4)
    T5 = transformToMatrix(DH[5, 0], DH[5, 1], DH[5, 2], DH[5, 3] + j5)

    return T0 @ T1 @ T2 @ T3 @ T4 @ T5

def test():
    # 输出计算的变换矩阵
    print(forwardKinematic(DH, 0, 0, 0, 0, 0, 0))
  
if __name__ == '__main__':
    test()

