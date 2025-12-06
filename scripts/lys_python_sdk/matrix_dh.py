import numpy as np
import math
from dh import d,a,alp,offset

def transformToMatrix(alpha,a,d,theta):
  T = np.mat([[math.cos(theta),-math.sin(theta)*math.cos(alpha),math.sin(theta)*math.sin(alpha),a*math.cos(theta)],
               [math.sin(theta),math.cos(theta)*math.cos(alpha),-math.cos(theta)*math.sin(alpha),a*math.sin(theta)],
               [0,math.sin(alpha),math.cos(alpha),d],
               [0,0,0,1]
             ])
  return T


def createDH_matrices(q,offset,alp,a,d):
  """计算T0-T5"""
  DH_matrices = []
  for i in range(6):
    #偏移量加上关节角度
    theta = offset[i]+q[i]
    alpha_i = alp[i]
    a_i = a[i]
    d_i = d[i]
    DH_matrices.append(transformToMatrix(alpha_i,a_i,d_i,theta))
  return DH_matrices

def forward_kinematics(q):
  """计算正运动学,返回末端变换矩阵T06"""
  DH_matrices = createDH_matrices(q,offset,alp,a,d)
  T06 = np.eye(4) #初始为单位矩阵
  for T in DH_matrices:
    T06 = np.dot(T06,T)
  return T06

def inverse_kinematic(x,y,z):
  """计算逆运动学,根据末端位置(x,y,x)计算关节角度"""
  #假设在工作空间内
  T06 = np.eye(4)
  T06[:3,3] = [x,y,z]
  
  #计算T05
  T5 = np.dot(T06,np.linalg.inv(createDH_matrices([0]*6,offset,alp,a,d)[5]))
  X_t,Y_t,Z_t = T5[:3,3]  

  #计算目标位置(X_t,Y_t)的平面坐标平方和
  #即从基座投影到平面的距离的平方
  r_squared = X_t**2+Y_t**2
  #计算目标位置在X-Y平面上的距离(或径向距离),
  #即从基座到目标位置在平面的投影的距离
  r_gen = np.sqrt(r_squared)
  #计算目标位置的Z坐标相对于机械臂基座的高度差
  #d[0]通常是第一个关节到基座的距离
  #Z_t是目标点在Z轴上的坐标,S表示这个高度差
  S = Z_t - d[0]
  #从基座到目标点的直线距离的平方
  R_squared = S**2 + r_squared
  #计算从基座到目标点的直线距离
  R_gen = np.sqrt(R_squared)
  #计算第一个关节的角度
  theta1 = np.arctan2(Y_t,X_t)

  #计算连杆3和连杆4的合成长度a33
  #a[2]和a[3]分别是第三段和第四段连杆的长度
  #a33是这两段连杆组成的三角形的斜边,即他们的合成长度
  a33 = np.sqrt(a[2]**2+a[3]**2)
  #计算连杆3和连杆4合成长度a33的角度a33_angle
  #a[3]/a[2]是这两段连杆组成的直角三角形中,角a33_angle对应的正切值
  a33_angle = np.arctan(a[3]/a[2])
  #根据余弦定理计算cos(theta3_bu),其中theta3_bu是第三个关节角度的中间值
  cos_theta3_bu = (a[1]**2+a33**2-R_squared)/(2*a[1]*a33)
  cos_theta3_bu = np.clip(cos_theta3_bu,-1.0,1.0)
  #计算最终的第三个关节角度theta3
  theta3 = np.pi - np.arccos(cos_theta3_bu)-a33_angle
  #根据余弦定理计算cos(theta2_bu),其中theta2_bu是第二个关节角度的中间值
  #a[1]是第二段连杆的长度,R_gen是从基座到目标点的直线距离
  #a33是第三段和第四段连杆的合成长度
  cos_theta2_bu = (a[1]**2+R_squared-a33**2)/(2*a[1]*R_gen)
  cos_theta2_bu = np.clip(cos_theta2_bu,-1.0,1.0)
  #检查目标点在平面以上还是在平面以下
  #如果在平面以上
  if S > 0:
    theta2 = np.pi/2 - np.arccos(cos_theta2_bu) - np.arctan(S/r_gen)
  #如果在平面以下
  else:
    theta2 = np.pi/2 - np.arccos(cos_theta2_bu) + np.arctan(-S/r_gen)

  theta4,theta5,theta6 = 0,0,0

  return [theta1,theta2,theta3,theta4,theta5,theta6]

def test_ink():
  #输入测试的关节角度
  q = [0,0,0,0,0,0]
  T06 = forward_kinematics(q)
  print("Final Transformation Matrix:")
  print(T06)

  #假设要计算逆运动学
  x,y,z = T06[0,3],T06[1,3],T06[2,3]
  q_inv = inverse_kinematic(x,y,z)
  print("Inverse Kinematic Angles (in radians):",q_inv)


def test():
  q = [0,0,0,0,0,0]
  T06 = forward_kinematics(q)
  print("Final Transformation Matrix:")
  print(T06)

"""
#打印DH矩阵
for i,T in enumerate(DH_matrices):
  print(f"DH Matrix{i+1}: \n",T)
"""

"""
print("Final Transformation Matrix:")
print(T06)
"""
if __name__ == "__main__":
  test_ink()
