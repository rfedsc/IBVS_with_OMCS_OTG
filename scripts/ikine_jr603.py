import numpy as np
from math import pi
import math
from scipy.spatial.transform import Rotation
from pose2T import cartesian_to_homogeneous,euler_to_rotation_matrix

np.set_printoptions(suppress=True)

#绕x轴旋转的齐次变换矩阵
def trotx(theta):
    matrixs = np.array(np.zeros((4, 4)))
    sth = math.sin(theta)
    cth = math.cos(theta)
    matrixs[0, 0] = 1
    matrixs[1, 1] = cth
    matrixs[1, 2] = -sth
    matrixs[2, 1] = sth
    matrixs[2, 2] = cth
    matrixs[3, 3] = 1
    return matrixs
#绕y轴旋转的齐次变换矩阵
def troty(theta):
    matrixs = np.array(np.zeros((4, 4)))
    sth = math.sin(theta)
    cth = math.cos(theta)
    matrixs[0, 0] = cth
    matrixs[0, 2] = sth
    matrixs[1, 1] = 1
    matrixs[2, 0] = -sth
    matrixs[2, 2] = cth
    matrixs[3, 3] = 1
    return matrixs
#绕z轴旋转的齐次变换矩阵
def trotz(theta):
    matrixs = np.array(np.zeros((4, 4)))
    sth = math.sin(theta)
    cth = math.cos(theta)
    matrixs[0, 0] = cth
    matrixs[1, 0] = sth
    matrixs[0, 1] = -sth
    matrixs[1, 1] = cth
    matrixs[2, 2] = 1
    matrixs[3, 3] = 1
    return matrixs
#用于生成三维空间中的平移矩阵
def transl(px, py, pz):
    matrixs = np.array(np.identity(4))
    matrixs[0, 3] = px
    matrixs[1, 3] = py
    matrixs[2, 3] = pz
    return matrixs

# 根据DH计算齐次矩阵
def dhtransform(theta, d, a, alpha):
    matrixs = np.array(np.zeros((4, 4)))

    sth = math.sin(theta)
    cth = math.cos(theta)
    sa = math.sin(alpha)
    ca = math.cos(alpha)

    matrixs[0, 0] = cth
    matrixs[1, 0] = sth
    matrixs[0, 1] = -sth * ca
    matrixs[1, 1] = cth * ca
    matrixs[2, 1] = sa
    matrixs[0, 2] = sth * sa
    matrixs[1, 2] = -cth * sa
    matrixs[2, 2] = ca
    matrixs[0, 3] = a * cth
    matrixs[1, 3] = a * sth
    matrixs[2, 3] = d
    matrixs[3, 3] = 1

    return matrixs

# 正运动学
def forward_kinematics(theta):
    a2=284/1000
    a3=-30/1000
    d4=286.5/1000
    d6=81.5/1000

    T01 = dhtransform(theta[0],0,0,-pi/2)
    T12 = dhtransform(theta[1],0,a2,0)
    T23 = dhtransform(theta[2],0,a3,pi/2)
    T34 = dhtransform(theta[3],d4,0,-pi/2)
    T45 = dhtransform(theta[4],0,0,pi/2)
    T56 = dhtransform(theta[5],d6,0,0)
    T06 = T01@T12@T23@T34@T45@T56
    return T06

# 逆运动学
def inverse_kinematics(T):
    a2 = 284/1000
    a3 = -30/1000
    d4 = 286.5/1000
    d6 = 81.5/1000

    px = T[0, 3]
    py = T[1, 3]
    pz = T[2, 3]
    ax = T[0, 2]
    ay = T[1, 2]
    az = T[2, 2]

    wx = px - d6 * ax
    wy = py - d6 * ay
    wz = pz - d6 * az
    t11 = math.atan2(0, math.sqrt(wy * wy + wx * wx)) - math.atan2(wy, -wx)
    t12 = math.atan2(0, -math.sqrt(wy * wy + wx * wx)) - math.atan2(wy, -wx)

    A1 = wx * math.cos(t11) + wy * math.sin(t11)
    B1 = wz
    C1 = (A1 * A1 + B1 * B1 - a2 * a2 - a3 * a3 - d4 * d4) / (2 * a2)
    t31 = math.atan2(C1, math.sqrt(math.fabs(a3 * a3 + d4 * d4 - C1 * C1))) - math.atan2(a3, d4)
    t32 = math.atan2(C1, -math.sqrt(math.fabs(a3 * a3 + d4 * d4 - C1 * C1))) - math.atan2(a3, d4)
    A2 = wx * math.cos(t12) + wy * math.sin(t12)
    B2 = wz
    C2 = (A2 * A2 + B2 * B2 - a2 * a2 - a3 * a3 - d4 * d4) / (2 * a2)
    t33 = math.atan2(C2, math.sqrt(math.fabs(a3 * a3 + d4 * d4 - C2 * C2))) - math.atan2(a3, d4)
    t34 = math.atan2(C2, -math.sqrt(math.fabs(a3 * a3 + d4 * d4 - C2 * C2))) - math.atan2(a3, d4)

    D1 = (A1 * A1 + B1 * B1 + a2 * a2 - a3 * a3 - d4 * d4) / (2 * a2)
    D2 = (A2 * A2 + B2 * B2 + a2 * a2 - a3 * a3 - d4 * d4) / (2 * a2)
    t21 = math.atan2(D1, -math.sqrt(math.fabs(A1 * A1 + B1 * B1 - D1 * D1))) - math.atan2(A1,-B1)
    t22 = math.atan2(D1, math.sqrt(math.fabs(A1 * A1 + B1 * B1 - D1 * D1))) - math.atan2(A1,-B1)
    t23 = math.atan2(D2, -math.sqrt(math.fabs(A2 * A2 + B2 * B2 - D2 * D2))) - math.atan2(A2,-B2)
    t24 = math.atan2(D2, math.sqrt(math.fabs(A2 * A2 + B2 * B2 - D2 * D2))) - math.atan2(A2,-B2)

    R30_1 = np.array([[math.cos(t11) * math.cos(t21 + t31),math.sin(t11) * math.cos(t21 + t31),-math.sin(t21 + t31)],
                        [-math.sin(t11),math.cos(t11),0],
                        [math.cos(t11) * math.sin(t21 + t31),math.sin(t11) * math.sin(t21 + t31),math.cos(t21 + t31)]])

    R30_2 = np.array([[math.cos(t11) * math.cos(t22 + t32),math.sin(t11) * math.cos(t22 + t32),-math.sin(t22 + t32)],
                        [-math.sin(t11),math.cos(t11),0],
                        [math.cos(t11) * math.sin(t22 + t32),math.sin(t11) * math.sin(t22 + t32),math.cos(t22 + t32)]])

    R30_3 = np.array([[math.cos(t12) * math.cos(t23 + t33),math.sin(t12) * math.cos(t23 + t33),-math.sin(t23 + t33)],
                        [-math.sin(t12),math.cos(t12),0],
                        [math.cos(t12) * math.sin(t23 + t33),math.sin(t12) * math.sin(t23 + t33),math.cos(t23 + t33)]])

    R30_4 = np.array([[math.cos(t12) * math.cos(t24 + t34),math.sin(t12) * math.cos(t24 + t34),-math.sin(t24 + t34)],
                        [-math.sin(t12),math.cos(t12),0],
                        [math.cos(t12) * math.sin(t24 + t34),math.sin(t12) * math.sin(t24 + t34),math.cos(t24 + t34)]])

    R36_1 = np.dot(R30_1,T[:3, :3])
    R36_2 = np.dot(R30_2,T[:3, :3])
    R36_3 = np.dot(R30_3,T[:3, :3])
    R36_4 = np.dot(R30_4,T[:3, :3])

    nz36_1 = R36_1[2, 0]
    nz36_2 = R36_2[2, 0]
    nz36_3 = R36_3[2, 0]
    nz36_4 = R36_4[2, 0]
    ax36_1 = R36_1[0, 2]
    ax36_2 = R36_2[0, 2]
    ax36_3 = R36_3[0, 2]
    ax36_4 = R36_4[0, 2]
    ay36_1 = R36_1[1, 2]
    ay36_2 = R36_2[1, 2]
    ay36_3 = R36_3[1, 2]
    ay36_4 = R36_4[1, 2]
    az36_1 = R36_1[2, 2]
    az36_2 = R36_2[2, 2]
    az36_3 = R36_3[2, 2]
    az36_4 = R36_4[2, 2]
    oz36_1 = R36_1[2, 1]
    oz36_2 = R36_2[2, 1]
    oz36_3 = R36_3[2, 1]
    oz36_4 = R36_4[2, 1]

    t51 = math.acos(az36_1)
    t52 = -math.acos(az36_1)
    t53 = math.acos(az36_2)
    t54 = -math.acos(az36_2)
    t55 = math.acos(az36_3)
    t56 = -math.acos(az36_3)
    t57 = math.acos(az36_4)
    t58 = -math.acos(az36_4)

    t41 = math.atan2(ay36_1 / math.sin(t51), ax36_1 / math.sin(t51))
    t42 = math.atan2(ay36_1 / math.sin(t52), ax36_1 / math.sin(t52))
    t43 = math.atan2(ay36_2 / math.sin(t53), ax36_2 / math.sin(t53))
    t44 = math.atan2(ay36_2 / math.sin(t54), ax36_2 / math.sin(t54))
    t45 = math.atan2(ay36_3 / math.sin(t55), ax36_3 / math.sin(t55))
    t46 = math.atan2(ay36_3 / math.sin(t56), ax36_3 / math.sin(t56))
    t47 = math.atan2(ay36_4 / math.sin(t57), ax36_4 / math.sin(t57))
    t48 = math.atan2(ay36_4 / math.sin(t58), ax36_4 / math.sin(t58))

    t61 = math.atan2(oz36_1 / math.sin(t51), -nz36_1 / math.sin(t51))
    t62 = math.atan2(oz36_1 / math.sin(t52), -nz36_1 / math.sin(t52))
    t63 = math.atan2(oz36_2 / math.sin(t53), -nz36_2 / math.sin(t53))
    t64 = math.atan2(oz36_2 / math.sin(t54), -nz36_2 / math.sin(t54))
    t65 = math.atan2(oz36_3 / math.sin(t55), -nz36_3 / math.sin(t55))
    t66 = math.atan2(oz36_3 / math.sin(t56), -nz36_3 / math.sin(t56))
    t67 = math.atan2(oz36_4 / math.sin(t57), -nz36_4 / math.sin(t57))
    t68 = math.atan2(oz36_4 / math.sin(t58), -nz36_4 / math.sin(t58))
    #将弧度转换为度数
    q = np.array([[t11, t21, t31, t41, t51, t61],
    [t11, t21, t31, t42, t52, t62],
    [t11, t22, t32, t43, t53, t63],
    [t11, t22, t32, t44, t54, t64],
    [t12, t23, t33, t45, t55, t65],
    [t12, t23, t33, t46, t56, t66],
    [t12, t24, t34, t47, t57, t67],
    [t12, t24, t34, t48, t58, t68]])*180/pi

    qmin = np.array([-180,-155,-20,-180,-95,-360])
    qmax = np.array([180,5,240,180,95,360])

    for i in range(q.shape[0]):
        for j in range(q.shape[1]):
            if q[i, j] < qmin[j]:
                q[i, j]+= 360
            if q[i, j] > qmax[j]:
                q[i, j]-= 360
            if q[i,j]<= qmin[j] or q[i,j]>=qmax[j]:
                q[i,:]=-1

    return q

def posetomatrix(px,py,pz,rx,ry,rz):
    return transl(px,py,pz)@trotz(rz)@troty(ry)@trotx(rx)

def matrixtorpy(T):
    rpy = np.array(np.zeros((1, 3)))
    rpy[0,0] = math.atan2(T[1, 0], T[0, 0])
    sp = math.sin(rpy[0,0])
    cp = math.cos(rpy[0,0])
    rpy[0,1] = math.atan2(-T[2, 0], cp * T[0, 0] + sp * T[1, 0])
    rpy[0,2] = math.atan2(sp * T[0, 2] - cp * T[1, 2], cp * T[1, 1] - sp * T[0, 1])
    rpy = rpy * 180 / pi
    return rpy

def matrixtopose(T):
    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]
    rpy = matrixtorpy(T)
    rx = rpy[0,2]
    ry = rpy[0,1]
    rz = rpy[0,0]
    xyzrpy = [x,y,z,rx,ry,rz]
    return xyzrpy




if __name__ == "__main__":

    x,y,z = 354.331/1000,-0.002/1000,392.084/1000
    roll,pitch,yaw = np.radians(0.001),np.radians(71.305),np.radians(0.000)

    T = cartesian_to_homogeneous(x,y,z,roll,pitch,yaw)
    
    angles=inverse_kinematics(T)
    print("---angles---")
    print(angles)









