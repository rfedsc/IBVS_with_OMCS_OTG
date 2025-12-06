import numpy as np
from calcST import calcST
from ikine_algebra import InverseKinematics

#np.set_printoptions(suppress=True)
#每个角度范围在-pi到pi之间
theta =np.array([1.0,1.0,1.0,1.0,1.0,1.0])
#theta =np.array([0.1701696,-0.1701696,0.16563175,-1.79315127,0.21781709,0.14713126])
#theta = 2*np.pi*(np.random.rand(6)-0.5)
print("given thata = ")
print(theta)
#print(type(theta))
#定义DH参数
d1,a2,a3,d4,d5,d6 = 0.13122,-0.1104,-0.096,0.0634,0.07505,0.0456
para = [d1,a2,a3,d4,d5,d6]

#计算齐次变换矩阵
T1 = calcST(theta[0],d1,0,np.pi/2)
T2 = calcST(theta[1]-np.pi/2,0,a2,0)
T3 = calcST(theta[2],0,a3,0)
T4 = calcST(theta[3]-np.pi/2,d4,0,np.pi/2)
T5 = calcST(theta[4]+np.pi/2,d5,0,-np.pi/2)
T6 = calcST(theta[5],d6,0,0)

#计算总的齐次变换矩阵
T = T1@T2@T3@T4@T5@T6
print(f'T = {T}')
q = InverseKinematics(T,para)
print("solve theta = ")
for i in range(len(q)):
  print(f'q{i}: {q[i]}')
