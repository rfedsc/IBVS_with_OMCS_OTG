import numpy as np
#theta:连杆绕其z轴旋转的角度(弧度)
#d:沿着x轴(从上一个连杆的z轴指向当前连杆的z轴)的偏移距离
#a:沿着z轴(从上一个连杆的x轴指向当前连杆的x轴)的连杆长度
#alpha:绕x轴扭转的角度(从上一个连杆的z轴到当前连杆的z轴的旋转角度,以x轴为轴)
def calcST(theta,d,a,alpha):
  """
  通过标准DH参数计算齐次变换矩阵
  """
  #确保theta和alpha是numpy数组
  theta = np.float64(theta)
  alpha = np.float64(alpha)
  #theta = np.asarray(theta,dtype=np.float64)
  #alpha = np.asarray(alpha,dtype=np.float64)
  
  #print(f"theta type:{type(theta)},value:{theta}")
  #print(f"d type:{type(d)},value:{d}")
  #print(f"a type:{type(a)},value:{a}")
  #print(f"alpha type:{type(alpha)},value:{alpha}")

  T = np.array([
      [np.cos(theta),-np.sin(theta)*np.cos(alpha),np.sin(theta)*np.sin(alpha),a*np.cos(theta)],
      [np.sin(theta),np.cos(theta)*np.cos(alpha),-np.cos(theta)*np.sin(alpha),a*np.sin(theta)],
      [0,np.sin(alpha),np.cos(alpha),d],
      [0,0,0,1]
  ])
  return T

