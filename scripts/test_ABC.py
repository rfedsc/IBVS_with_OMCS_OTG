import numpy as np

def rotation_matrix_to_zyx_euler(R):
    """
    从旋转矩阵提取ZYX顺序的欧拉角
    输入:
        R: 3x3旋转矩阵
    输出:
        (gamma, beta, alpha): 对应ZYX顺序的欧拉角（单位：弧度）
    """
    # 提取 beta
    beta = -np.arcsin(R[2, 0])  # R[3,1]对应旋转矩阵的第三行第一列
    if np.abs(R[2, 0]) < 1:  # 非奇异情况
        gamma = np.arctan2(R[1, 0], R[0, 0])  # 提取 gamma
        alpha = np.arctan2(R[2, 1], R[2, 2])  # 提取 alpha
    else:  # 奇异情况
        gamma = 0
        if R[2, 0] == -1:
            beta = np.pi / 2
            alpha = np.arctan2(R[0, 1], R[0, 2])
        else:
            beta = -np.pi / 2
            alpha = -np.arctan2(R[0, 1], R[0, 2])
    
    return gamma, beta, alpha

# 示例旋转矩阵
R_example = np.array([
    [ -0.99586638 ,  0.04795901 , -0.07713675],
    [  0.047969   ,  0.99884734 ,  0.00172442],
    [  0.07713054 , -0.00198288 , -0.99701903]
])

# 提取欧拉角
gamma, beta, alpha = rotation_matrix_to_zyx_euler(R_example)
print(f"gamma (绕Z轴): {np.degrees(gamma)}°")
print(f"beta  (绕Y轴): {np.degrees(beta)}°")
print(f"alpha (绕X轴): {np.degrees(alpha)}°")

