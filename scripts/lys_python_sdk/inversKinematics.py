import numpy as np

# Denavit-Hartenberg 参数到变换矩阵
def calcST(theta, d, a, alpha):
    T = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha),  a*np.cos(theta)],
                  [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha),  a*np.sin(theta)],
                  [0,              np.sin(alpha),               np.cos(alpha),               d],
                  [0,              0,                           0,                           1]])
    return T

# 逆运动学函数
def InverseKinematics(T, para):
    theta_solutions = []
    d1, a2, a3, d4, d5, d6 = para

    e = T[0, 3] - d6 * T[0, 2]
    f = T[1, 3] - d6 * T[1, 2]
    g = e**2 + f**2
    h = 2 * d4 * f
    k = d4**2 - e**2

    for ic1 in range(2):
        if ic1 == 0:
            c1 = (-h + np.sqrt(h**2 - 4*g*k)) / (2*g)
        else:
            c1 = (-h - np.sqrt(h**2 - 4*g*k)) / (2*g)

        for i1 in range(2):
            if i1 == 0:
                theta1 = np.arccos(c1)
            else:
                theta1 = -np.arccos(c1)

            B = np.dot(np.array([[np.cos(theta1), np.sin(theta1), 0, 0],
                                 [0, 0, 1, -d1],
                                 [np.sin(theta1), -np.cos(theta1), 0, 0],
                                 [0, 0, 0, 1]]), T)

            for i5 in range(2):
                if i5 == 0:
                    theta5 = np.arcsin(-B[2, 2])
                else:
                    if -B[2, 2] >= 0:
                        theta5 = np.pi - np.arcsin(-B[2, 2])
                    else:
                        theta5 = -np.pi - np.arcsin(-B[2, 2])

                if np.cos(theta5) >= 0:
                    theta6 = np.arctan2(-B[2, 1], B[2, 0])
                else:
                    theta6 = np.arctan2(B[2, 1], -B[2, 0])

                for i234 in range(6):
                    if i234 == 0:
                        theta234 = np.arccos(B[0, 2] / np.cos(theta5)) - 2 * np.pi
                    elif i234 == 1:
                        theta234 = np.arccos(B[0, 2] / np.cos(theta5))
                    elif i234 == 2:
                        theta234 = np.arccos(B[0, 2] / np.cos(theta5)) + 2 * np.pi
                    elif i234 == 3:
                        theta234 = -np.arccos(B[0, 2] / np.cos(theta5)) + 2 * np.pi
                    elif i234 == 4:
                        theta234 = -np.arccos(B[0, 2] / np.cos(theta5))
                    else:
                        theta234 = -np.arccos(B[0, 2] / np.cos(theta5)) - 2 * np.pi

                    m = B[0, 3] + d5 * np.sin(theta234) - d6 * np.cos(theta5) * np.cos(theta234)
                    n = B[1, 3] - d5 * np.cos(theta234) - d6 * np.cos(theta5) * np.sin(theta234)
                    p = m**2 + n**2 + a2**2 - a3**2
                    q = 2 * n * a2
                    r = 2 * m * a2
                    s = q**2 + r**2
                    t = 2 * p * q
                    u = p**2 - r**2
                    delta = t**2 - 4 * s * u

                    if delta < 0:
                        continue

                    for ic2 in range(2):
                        if ic2 == 0:
                            c2 = (-t + np.sqrt(delta)) / (2 * s)
                        else:
                            c2 = (-t - np.sqrt(delta)) / (2 * s)

                        for i2 in range(2):
                            if i2 == 0:
                                theta2 = np.arccos(c2)
                            else:
                                theta2 = -np.arccos(c2)

                            c23 = (n + a2 * np.cos(theta2)) / (-a3)

                            for i23 in range(4):
                                if i23 == 0:
                                    theta23 = np.arccos(c23)
                                elif i23 == 1:
                                    theta23 = 2 * np.pi - np.arccos(c23)
                                elif i23 == 2:
                                    theta23 = -np.arccos(c23)
                                else:
                                    theta23 = -2 * np.pi + np.arccos(c23)

                                if abs(theta234 - theta23) <= np.pi:
                                    theta4 = theta234 - theta23
                                else:
                                    continue

                                if abs(theta23 - theta2) <= np.pi:
                                    theta3 = theta23 - theta2
                                else:
                                    continue

                                T1 = calcST(theta1, d1, 0, np.pi/2)
                                T2 = calcST(theta2-np.pi/2, 0, a2, 0)
                                T3 = calcST(theta3, 0, a3, 0)
                                T4 = calcST(theta4-np.pi/2, d4, 0, np.pi/2)
                                T5 = calcST(theta5+np.pi/2, d5, 0, -np.pi/2)
                                T6 = calcST(theta6, d6, 0, 0)
                                T_end = T1 @ T2 @ T3 @ T4 @ T5 @ T6

                                if np.linalg.norm(T - T_end) < 1e-3:
                                    theta_solutions.append([theta1, theta2, theta3, theta4, theta5, theta6])

    return np.array(theta_solutions)

def test():
    # 测试逆运动学函数
    # angles_in_degrees = np.array([0,0,0,0,0,0])
    angles_in_degrees = 360*(np.random.rand(6)-0.5)
    angles_in_radians = np.radians(angles_in_degrees)
    theta = angles_in_radians
    print(f"Given theta in degrees: ",angles_in_degrees)
    print(f"Given theta in radians: ",angles_in_radians)

    # 定义参数
    d1 = 0; a2 =0.284; a3 = -30
    d4 = 0.2865; d5 = 0; d6 = 0.0815
    para = [d1, a2, a3, d4, d5, d6]

    # 计算给定角度下的 T
    T1 = calcST(theta[0],    d1,   0,    -np.pi/2)
    T2 = calcST(theta[1],     0,   a2,          0)
    T3 = calcST(theta[2],     0,   a3,      np.pi)
    T4 = calcST(theta[3],    d4,   0,    -np.pi/2)
    T5 = calcST(theta[4],    d5,   0,     np.pi/2)
    T6 = calcST(theta[5],    d6,   0,           0)
    T = T1 @ T2 @ T3 @ T4 @ T5 @ T6

    # 计算逆运动学解
    q = InverseKinematics(T, para)
    q = np.degrees(q)
    print("Solved theta: ")
    print(q)


if __name__ =="__main__":
    test()



