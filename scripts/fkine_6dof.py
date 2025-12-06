import numpy as np
import sympy as sp
from scipy.linalg import inv
from dh import d,a,alp,offset
np.set_printoptions(suppress=True)

def fkine_6dof(q):
  thd = q+offset #计算每个关节的实际角度,其中q是输入的关节角度数组,offset是每个关节角度的偏移量

  A1 = np.dot(trotz(thd[0]),np.dot(transl(0,0,d[0]),np.dot(trotx(alp[0]),transl(a[0],0,0))))
  A2 = np.dot(trotz(thd[1]),np.dot(transl(0,0,d[1]),np.dot(trotx(alp[1]),transl(a[1],0,0))))
  A3 = np.dot(trotz(thd[2]),np.dot(transl(0,0,d[2]),np.dot(trotx(alp[2]),transl(a[2],0,0))))
  A4 = np.dot(trotz(thd[3]),np.dot(transl(0,0,d[3]),np.dot(trotx(alp[3]),transl(a[3],0,0))))
  A5 = np.dot(trotz(thd[4]),np.dot(transl(0,0,d[4]),np.dot(trotx(alp[4]),transl(a[4],0,0))))
  A6 = np.dot(trotz(thd[5]),np.dot(transl(0,0,d[5]),np.dot(trotx(alp[5]),transl(a[5],0,0))))

  #最终变换矩阵
  T = A1@A2@A3@A4@A5@A6
  return T
  
def transl(x,y,z):
  """生成平移矩阵"""
  return np.array([[1,0,0,x],
                   [0,1,0,y],
                   [0,0,1,z],
                   [0,0,0,1]])

def trotx(theta):
  """生成绕x轴旋转的变换矩阵"""
  return np.array([[1,0,0,0],
                   [0,np.cos(theta),-np.sin(theta),0],
                   [0,np.sin(theta),np.cos(theta),0],
                   [0,0,0,1]])


def trotz(theta):
  """生成绕z轴旋转的变换矩阵"""
  return np.array([[np.cos(theta),-np.sin(theta),0,0],
                   [np.sin(theta),np.cos(theta),0,0],
                   [0,0,1,0],
                   [0,0,0,1]])
def test():
  q = np.array([0,-90,180,0,77.085,0])
  q = np.radians(q)
  T = fkine_6dof(q)
  print(T)

if __name__ == "__main__":
  test()


