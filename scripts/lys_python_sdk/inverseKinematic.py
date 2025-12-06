import numpy as np
import math
from pyquaternion import Quaternion

np.set_printoptions(suppress=True)


# DH矩阵每列的含义：连杆夹角、连杆长度、连杆偏距、初始关节角
DH = np.mat([[ 1.5708,     0, 131.22,             0],
             [      0,-110.4,       0,      -1.5708],
             [      0,   -96,       0,            0],
             [ 1.5708,     0,    64.4,      -1.5708],
             [-1.5708,     0,   75.05,       1.5708],
             [      0,     0,    50.6,            0]])



def transformToMatrix(alpha,a,d,theta):
  """从前一个坐标系到当前坐标系的变换,包括平移和旋转"""
  T0 = np.eye(4)
  #T1沿X轴旋转alpha角度
  T1 = np.mat([[1,0,0,0],
               [0,math.cos(alpha),-math.sin(alpha),0],
               [0,math.sin(alpha),math.cos(alpha),0],
               [0,0,0,1]
              ])
  #T2沿X轴平移a,沿y轴平移d
  T2 = np.mat([[1,0,0,a],
               [0,1,0,0],
               [0,0,1,d],
               [0,0,0,1]
              ])
  #T3沿着Z轴旋闸theta
  T3 = np.mat([[math.cos(theta),-math.sin(theta),0,0],
               [math.sin(theta),math.cos(theta),0,0],
               [0,0,1,0],
               [0,0,0,1]
              ])
  return T3*T2*T1

def forwardKinematic(DH,j0,j1,j2,j3,j4,j5):
  T0 = transformToMatrix(DH[0,0],DH[0,1],DH[0,2],DH[0,3]+j0)
  T1 = transformToMatrix(DH[1,0],DH[1,1],DH[1,2],DH[1,3]+j1)
  T2 = transformToMatrix(DH[2,0],DH[2,1],DH[2,2],DH[2,3]+j2)
  T3 = transformToMatrix(DH[3,0],DH[3,1],DH[3,2],DH[3,3]+j3)
  T4 = transformToMatrix(DH[4,0],DH[4,1],DH[4,2],DH[4,3]+j4)
  T5 = transformToMatrix(DH[5,0],DH[5,1],DH[5,2],DH[5,3]+j5)
  
  return T0*T1*T2*T3*T4*T5


def calcu3ForwardJointAngle(DH,j0,x0,y0,x,y,b,js):
  """通过给定的DH参数、起始点和终止点的坐标来计算可能的关节角度组合"""
  if abs((x0-x)*(x0-x)+(y0-y)*(y0-y)-b*b) < 0.0001:
    js.append([j0])
    js[-1].append(math.atan2(y0,x0))
    js[-1].append(math.atan2(y-y0,x-x0)-math.atan2(y0,x0)+0.5*math.pi-DH[2,3])
    
    js.append([j0+math.pi])
    js[-1].append(math.pi-math.atan2(y0,x0))
    js[-1].append(math.atan2(y0,x0)-math.atan2(y-y0,x-x0)+0.5*math.pi-DH[2,3])
  if abs((x0-x)*(x0-x)+(-y0-y)*(-y0-y)-b*b)<0.0001:
    js.append([j0])
    js[-1].append(math.atan2(-y0,x0))
    js[-1].append(math.atan2(y+y0,x-x0)-math.atan2(-y0,x0)+0.5*math.pi-DH[2,3])

    js.append([j0+math.pi])
    js[-1].append(math.pi-math.atan2(-y0,x0))
    js[-1].append(math.atan2(-y0,x0)-math.atan2(y+y0,x-x0)+0.5*math.pi-DH[2,3])


def rotateMatrixToQuaternion(R):
  """将旋转矩阵转换为四元数"""
  R1 = [[R[0,0],R[0,1],R[0,2]],
        [R[1,0],R[1,1],R[1,2]],
        [R[2,0],R[2,1],R[2,2]]
        ]
  q = Quaternion(matrix=np.array(R1))
  
  return q.x,q.y,q.z,q.w

def quaternionToRotationMatrix(x,y,z,w):
  """将四元数转换为旋转矩阵"""
  a = math.sqrt(x*x+y*y+z*z+w*w)
  if a == 0:
    print('quaternion is error')
    return np.eye(3)
  x = x/a
  y = y/a
  z = z/a
  w = w/a
  return np.mat([[1-2*y*y-2*z*z,2*x*y-2*z*w,2*x*z+2*y*w],
                 [2*x*y+2*z*w,1-2*x*x-2*z*z,2*y*z-2*x*w],
                 [2*x*z-2*y*w,2*y*z+2*x*w,1-2*x*x-2*y*y]
                ])



def inverseKinematic(DH,x,y,z,ox,oy,oz,ow):
  #计算四元数的模长
  q_length = math.sqrt(ox*ox+oy*oy+oz*oz+ow)
  if q_length == 0:
    print('quaternion is error')
    return
  #对四元数进行归一化
  else:
    ox = ox/q_length
    oy = oy/q_length
    oz = oz/q_length
    ow = ow/q_length

  js = []
  #计算初始关节角度j00(从原点到目标位置在xy平面上的角度)
  j00 = math.atan2(y,x)
  a = DH[2,1]
  b = DH[3,2]
  #c是从当前末端执行器件位置到基座的距离
  c = math.sqrt(x*x+y*y+(z-DH[0,2])*(z-DH[0,2]))
  #构造二次方程a0*x^2+a1*x1+a2 = 0
  a0 = 4*(x*x+y*y)+4*(z-DH[0,2])*(z-DH[0,2])
  a1 = -4*(a*a-b*b+c*c)*math.sqrt(x*x+y*y)
  a2 = (a*a-b*b+c*c)*(a*a-b*b+c*c)-4*(z-DH[0,2])*(z-DH[0,2])*a*a
  # 确保 a0, a1, a2 是标量
  #print(f"a0: {a0}, a1: {a1}, a2: {a2}")
  #判别式大于零,方程有两个实数解
  if a1*a1 - 4*a0*a2 > 0:
    x0 = (-a1+math.sqrt(a1*a1-4*a0*a2))/(2*a0)
    y0 = math.sqrt(a*a-x0*x0)
    
    calcu3ForwardJointAngle(DH,j00,x0,y0,math.sqrt(x*x+y*y),z-DH[0,2],b,js)
    x0 = (-a1-math.sqrt(a1*a1-4*a0*a2))/(2*a0)
    y0 = math.sqrt(a*a-x0*x0)
    calcu3ForwardJointAngle(DH,j00,x0,y0,math.sqrt(x*x+y*y),z-DH[0,2],b,js)
  #如果判别式等于0,有一个实根
  elif a1*a1 - 4*a0*a2 == 0:
    x0 = (-a1)/(2*a0)
    y0 = math.sqrt(a*a-x0*x0)
    calcu3ForwardJointAngle(DH,j00,x0,y0,math.sqrt(x*x+y*y),z-DH[0,2],b,js)
  #判别式小于零,无解
  else:
    print('no solve')
    js = [[]]
  
  new_js = []
  for j in js:
    #将四元数转换为旋转矩阵
    R = quaternionToRotationMatrix(ox,oy,oz,ow)
    #计算每个关节的变换矩阵
    T01 = transformToMatrix(DH[0,0],DH[0,1],DH[0,2],DH[0,3]+j[0])
    T12 = transformToMatrix(DH[1,0],DH[1,1],DH[1,2],DH[1,3]+j[1])
    T23 = transformToMatrix(DH[2,0],DH[2,1],DH[2,2],DH[2,3]+j[2])
    T34 = transformToMatrix(DH[3,0],DH[3,1],DH[3,2],DH[3,3]) 
    #提取各变换矩阵中的旋转部分 
    R01 = np.mat([[T01[0,0],T01[0,1],T01[0,2]],
                  [T01[1,0],T01[1,1],T01[1,2]],
                  [T01[2,0],T01[2,1],T01[2,2]]
                ])
    R12 = np.mat([[T12[0,0],T12[0,1],T12[0,2]],
                  [T12[1,0],T12[1,1],T12[1,2]],
                  [T12[2,0],T12[2,1],T12[2,2]]
                ])
    R23 = np.mat([[T23[0,0],T23[0,1],T23[0,2]],
                  [T23[1,0],T23[1,1],T23[1,2]],
                  [T23[2,0],T23[2,1],T23[2,2]]
                ])
    R34 = np.mat([[T34[0,0],T34[0,1],T34[0,2]],
                  [T34[1,0],T34[1,1],T34[1,2]],
                  [T34[2,0],T34[2,1],T34[2,2]]
                ])
    #将提取出的旋转矩阵转置与四元数旋转矩阵R进行矩阵乘法
    R = (R34.T)*(R23.T)*(R12.T)*(R01.T)*R

    #计算欧拉角
    #alpha绕z轴的旋转角度
    alpha = math.atan2(R[1,2],R[0,2])
    #betla绕y轴的旋转角度
    betla = math.atan2(math.sqrt(R[2,0]*R[2,0]+R[2,1]*R[2,1]),R[2,2])
    #gamal绕x轴的旋转角度
    gamal = math.atan2(R[2,1],-R[2,0])
   
    #生成新的关节角度集合 
    new_js.append([])
    new_js[-1].append(j[0])
    new_js[-1].append(j[1])
    new_js[-1].append(j[2])
    new_js[-1].append(alpha)
    new_js[-1].append(betla)
    new_js[-1].append(gamal)

    #生成第二组关节角度集合
    new_js.append([])
    new_js[-1].append(j[0])
    new_js[-1].append(j[1])
    new_js[-1].append(j[2])
    new_js[-1].append(alpha+math.pi)
    new_js[-1].append(-betla)
    new_js[-1].append(gamal+math.pi)

  return new_js

def radians_to_degrees(radians):
  """将弧度制转换为角度"""
  return radians*(180/math.pi)



def test():
  print(forwardKinematic(DH,0,0,0,0,0,0))
  
  js = inverseKinematic(DH,0,0,0,0,0,0,1)
  print('####################')
  for j in js:
    print('joint angle:',j)
    angles_in_degrees = [radians_to_degrees(angle) for angle in j[3:]]
    print('degree:',angles_in_degrees)
  print('####################')
  

if __name__ == '__main__':
  test()


