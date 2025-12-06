import numpy as np
import math
from pyquaternion import Quaternion

np.set_printoptions(suppress=True)
"""
# DH矩阵每列的含义：连杆夹角、连杆长度、连杆偏距、初始关节角
DH = np.mat([[             0,   0, 0.1,             0], 
             [ 0.5 * math.pi,   0,   0,             0], 
             [             0, 0.5,   0, 0.5 * math.pi], 
             [ 0.5 * math.pi,   0, 0.5,             0], 
             [-0.5 * math.pi,   0,   0,             0], 
             [ 0.5 * math.pi,   0,   0,             0]])
"""
# DH矩阵每列的含义：连杆夹角、连杆长度、连杆偏距、初始关节角
DH = np.mat([[ 1.5708,     0, 131.22,             0],
             [      0,-110.4,       0,      -1.5708],
             [      0,   -96,       0,            0],
             [ 1.5708,     0,    63.4,      -1.5708],
             [-1.5708,     0,   75.05,       1.5708],
             [      0,     0,    45.6,            0]])


def transformToMatrix(alpha, a, d, theta):
    T0 = np.eye(4)
    T1 = np.mat([[1,               0,                0, 0], 
                 [0, math.cos(alpha), -math.sin(alpha), 0], 
                 [0, math.sin(alpha),  math.cos(alpha), 0], 
                 [0,               0,                0, 1]])
    T2 = np.mat([[1, 0, 0, a], 
                 [0, 1, 0, 0], 
                 [0, 0, 1, d], 
                 [0, 0, 0, 1]])
    T3 = np.mat([[math.cos(theta), -math.sin(theta), 0, 0], 
                 [math.sin(theta),  math.cos(theta), 0, 0], 
                 [              0,                0, 1, 0], 
                 [              0,                0, 0, 1]])
    return T3 * T2 * T1

def forwardKinematic(DH, j0, j1, j2, j3, j4, j5):
    T0 = transformToMatrix(DH[0, 0], DH[0, 1], DH[0, 2], DH[0, 3] + j0)
    T1 = transformToMatrix(DH[1, 0], DH[1, 1], DH[1, 2], DH[1, 3] + j1)
    T2 = transformToMatrix(DH[2, 0], DH[2, 1], DH[2, 2], DH[2, 3] + j2)
    T3 = transformToMatrix(DH[3, 0], DH[3, 1], DH[3, 2], DH[3, 3] + j3)
    T4 = transformToMatrix(DH[4, 0], DH[4, 1], DH[4, 2], DH[4, 3] + j4)
    T5 = transformToMatrix(DH[5, 0], DH[5, 1], DH[5, 2], DH[5, 3] + j5)
    #print(T0 * T1 * T2 * T3 * T4 * T5)
    return T0 * T1 * T2 * T3 * T4 * T5

def calcu3ForwardJointAngle(DH, j0, x0, y0, x, y, b, js):
    if abs((x0 - x) * (x0 - x) + (y0 - y) * (y0 - y) - b*b) < 0.0001:
        js.append([j0])
        js[-1].append(math.atan2(y0, x0))
        js[-1].append(math.atan2(y - y0, x - x0) - math.atan2(y0, x0) + 0.5 * math.pi - DH[2, 3])

        js.append([j0 + math.pi])
        js[-1].append(math.pi - math.atan2(y0, x0))
        js[-1].append(math.atan2(y0, x0) - math.atan2(y - y0, x - x0) + 0.5 * math.pi - DH[2, 3])
    if abs((x0 - x) * (x0 - x) + (-y0 - y) * (-y0 - y) - b*b) < 0.0001:
        js.append([j0])
        js[-1].append(math.atan2(-y0, x0))
        js[-1].append(math.atan2(y + y0, x - x0) - math.atan2(-y0, x0) + 0.5 * math.pi - DH[2, 3])

        js.append([j0 + math.pi])
        js[-1].append(math.pi - math.atan2(-y0, x0))
        js[-1].append(math.atan2(-y0, x0) - math.atan2(y + y0, x - x0) + 0.5 * math.pi - DH[2, 3])

def quaternionToRotationMatrix(x, y, z, w):
    # a = math.sqrt(x*x + y*y + z*z)
    # if a == 0:
    #     return np.eye(3)
    # v1x = 0
    # v1y = -z
    # v1z = y
    # b = math.sqrt(v1x*v1x + v1y*v1y + v1z*v1z)
    # if b == 0:
    #     v1y = 1.0
    #     v1z = 0.0
    #     b = 1.0
    # v2 = np.cross(np.array([x, y, z]), np.array([v1x, v1y, v1z]))
    # #print(np.array([x, y, z]), np.array([v1x, v1y, v1z]), v2)
    # c = math.sqrt(v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2])
    # R01 = np.mat([[x / a, v1x / b, v2[0] / c],
    #               [y / a, v1y / b, v2[1] / c],
    #               [z / a, v1z / b, v2[2] / c]])
    
    # theta = 2 * math.acos(w)
    # #print(R01)
    # R12 = np.mat([[1,               0,                0],
    #               [0, math.cos(theta), -math.sin(theta)],
    #               [0, math.sin(theta),  math.cos(theta)]])
    # return R01 * R12 * np.linalg.inv(R01)

    a = math.sqrt(x*x + y*y + z*z + w*w)
    if a == 0:
        print('quaternion is error')
        return np.eye(3)
    x = x / a
    y = y / a
    z = z / a
    w = w / a
    return np.mat([[1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
                   [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
                   [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]])

def rotateMatrixToQuaternion(R):
    R1 = [[R[0, 0], R[0, 1], R[0, 2]],
          [R[1, 0], R[1, 1], R[1, 2]],
          [R[2, 0], R[2, 1], R[2, 2]]]
    q = Quaternion(matrix=np.array(R1))

    return q.x, q.y, q.z, q.w

def inverseKinematic(DH, x, y, z, ox, oy, oz, ow):
    q_length = math.sqrt(ox*ox + oy*oy + oz*oz + ow*ow)
    if q_length == 0:
        print('quaternion is error')
        return
    else:
        ox = ox / q_length
        oy = oy / q_length
        oz = oz / q_length
        ow = ow / q_length

    js = []
    j00 = math.atan2(y, x)
    a = DH[2, 1]
    b = DH[3, 2]
    c = math.sqrt(x*x + y*y + (z - DH[0, 2])*(z - DH[0, 2]))
    a0 = 4*(x*x+y*y) + 4*(z - DH[0, 2])*(z - DH[0, 2])
    a1 = -4*(a*a - b*b + c*c)*math.sqrt(x*x + y*y)
    a2 = (a*a - b*b + c*c) * (a*a - b*b + c*c) - 4 * (z - DH[0, 2]) * (z - DH[0, 2]) * a * a
    if a1*a1 - 4*a0*a2 > 0:
        x0 = (-a1 + math.sqrt(a1*a1 - 4*a0*a2)) / (2 * a0)
        y0 = math.sqrt(a*a - x0*x0)
        #print('x0: ', x0, 'y0: ', y0)
        calcu3ForwardJointAngle(DH, j00, x0, y0, math.sqrt(x*x+y*y), z - DH[0, 2], b, js)

        x0 = (-a1 - math.sqrt(a1*a1 - 4*a0*a2)) / (2 * a0)
        y0 = math.sqrt(a*a - x0*x0)
        #print('x1: ', x0, 'y1: ', y0)
        calcu3ForwardJointAngle(DH, j00, x0, y0, math.sqrt(x*x+y*y), z - DH[0, 2], b, js)
    elif a1*a1 - 4*a0*a2 == 0:
        x0 = (-a1) / (2 * a0)
        y0 = math.sqrt(a*a - x0*x0)
        #print('x0: ', x0, 'y0: ', y0)
        calcu3ForwardJointAngle(DH, j00, x0, y0, math.sqrt(x*x+y*y), z - DH[0, 2], b, js)
    else:
        print('no solve')
        js = [[]]

    new_js = []
    for j in js:
        R = quaternionToRotationMatrix(ox, oy, oz, ow)
        T01 = transformToMatrix(DH[0, 0], DH[0, 1], DH[0, 2], DH[0, 3] + j[0])
        T12 = transformToMatrix(DH[1, 0], DH[1, 1], DH[1, 2], DH[1, 3] + j[1])
        T23 = transformToMatrix(DH[2, 0], DH[2, 1], DH[2, 2], DH[2, 3] + j[2])
        T34 = transformToMatrix(DH[3, 0], DH[3, 1], DH[3, 2], DH[3, 3])
        R01 = np.mat([[T01[0, 0], T01[0, 1], T01[0, 2]],
                      [T01[1, 0], T01[1, 1], T01[1, 2]],
                      [T01[2, 0], T01[2, 1], T01[2, 2]]])
        R12 = np.mat([[T12[0, 0], T12[0, 1], T12[0, 2]],
                      [T12[1, 0], T12[1, 1], T12[1, 2]],
                      [T12[2, 0], T12[2, 1], T12[2, 2]]])
        R23 = np.mat([[T23[0, 0], T23[0, 1], T23[0, 2]],
                      [T23[1, 0], T23[1, 1], T23[1, 2]],
                      [T23[2, 0], T23[2, 1], T23[2, 2]]])
        R34 = np.mat([[T34[0, 0], T34[0, 1], T34[0, 2]],
                      [T34[1, 0], T34[1, 1], T34[1, 2]],
                      [T34[2, 0], T34[2, 1], T34[2, 2]]])
        R = (R34.T) * (R23.T) * (R12.T) * (R01.T) * R
        #print('RRRRRRRRR: \n', R)
        alpha = math.atan2(R[1, 2], R[0, 2])
        betla = math.atan2(math.sqrt(R[2, 0]*R[2, 0] + R[2, 1]*R[2, 1]), R[2, 2])
        gamal = math.atan2(R[2, 1], -R[2, 0])

        new_js.append([])
        new_js[-1].append(j[0])
        new_js[-1].append(j[1])
        new_js[-1].append(j[2])
        new_js[-1].append(alpha)
        new_js[-1].append(betla)
        new_js[-1].append(gamal)
        #print(new_js[-1])

        new_js.append([])
        new_js[-1].append(j[0])
        new_js[-1].append(j[1])
        new_js[-1].append(j[2])
        new_js[-1].append(alpha + math.pi)
        new_js[-1].append(-betla)
        new_js[-1].append(gamal + math.pi)
        #print(new_js[-1])

    return new_js

if __name__ == '__main__':
    print("hello world")
    #print(DH)
    print(forwardKinematic(DH,0 , 0, 0, 0, 0, 0))
    js = inverseKinematic(DH, 0, 0, 0, 0.5, 0.5,0.5,-0.5)
    print('##########################')
    for j in js:
        print('joint angle: ', j)
        T = forwardKinematic(DH, j[0], j[1], j[2], j[3], j[4], j[5])
        #print('P: ', T[0, 3], T[1, 3], T[2, 3])
        #print('Q: ', rotateMatrixToQuaternion(T))
    print('##########################')

