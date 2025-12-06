import numpy as np
import sympy as sp
from scipy.linalg import inv
from dh import d,a,alp,offset
#from fkine_6dof import fkine_6dof,transl,trotx,trotz

def transl(x, y, z):
  """生成平移矩阵"""
  return sp.Matrix([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

def trotx(theta):
  """生成绕x轴旋转的变换矩阵"""
  return sp.Matrix([
        [1, 0, 0, 0],
        [0, sp.cos(theta), -sp.sin(theta), 0],
        [0, sp.sin(theta), sp.cos(theta), 0],
        [0, 0, 0, 1]
    ])

def trotz(theta):
  """生成绕z轴旋转的变换矩阵"""
  return sp.Matrix([
        [sp.cos(theta), -sp.sin(theta), 0, 0],
        [sp.sin(theta), sp.cos(theta), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

def newton_method_fdiff(T_d,q0):
  """
  计算雅可比矩阵,用于牛顿迭代法的逆运动学求解
  输入:
      T_d:目标变换矩阵(4x4)
      q0:关节初始角度
  输出:
      df:一阶雅可比矩阵
  """

  #定义符号变量
  th1,th2,th3,th4,th5,th6 = sp.symbols('th1 th2 th3 th4 th5 th6')
  th = [th1,th2,th3,th4,th5,th6]
  thd = [o+t for o,t in zip(offset,th)] #thd = offset+th

  #计算每个关节的变换矩阵
  T1 = trotz(thd[0])@transl(0,0,d[0])@trotx(alp[0])@transl(a[0],0,0)
  T2 = trotz(thd[1])@transl(0,0,d[1])@trotx(alp[1])@transl(a[1],0,0)
  T3 = trotz(thd[2])@transl(0,0,d[2])@trotx(alp[2])@transl(a[2],0,0)
  T4 = trotz(thd[3])@transl(0,0,d[3])@trotx(alp[3])@transl(a[3],0,0)
  T5 = trotz(thd[4])@transl(0,0,d[4])@trotx(alp[4])@transl(a[4],0,0)
  T6 = trotz(thd[5])@transl(0,0,d[5])@trotx(alp[5])@transl(a[5],0,0)

  #计算总变换矩阵
  T06 = T1@T2@T3@T4@T5@T6
  #定义误差函数
  f1  = T06[0,3]-T_d[0,3]
  f2  = T06[1,3]-T_d[1,3]
  f3  = T06[2,3]-T_d[2,3]
  f4  = T06[1,2]-T_d[1,2]
  f5  = T06[2,2]-T_d[2,2]
  f6  = T06[2,1]-T_d[2,1]
  f7  = T06[0,0]-T_d[0,0]
  f8  = T06[0,1]-T_d[0,1]
  f9  = T06[0,2]-T_d[0,2]
  f10 = T06[1,0]-T_d[1,0]
  f11 = T06[1,1]-T_d[1,1]
  f12 = T06[2,0]-T_d[2,0]

  #计算雅可比矩阵
  f = sp.Matrix([f1,f2,f3,f4,f5,f6,f7,f8,f9,f10,f11,f12])
  df = f.jacobian(th) #计算雅可比矩阵
  
  #代入初始关节角度
  # df_numeric = df.subs(dict(zip(th,q0))).evalf
  df_numeric = sp.lambdify(th,df,'numpy')
  
  #计算雅可比矩阵的数值
  df = np.array(df_numeric(*q0)).astype(np.float64)

  return df
 #return np.array(df_numeric).astype(np.float64) #返回数值类型的雅可比矩阵

#示例用法
if __name__ == "__main__":
  #假设目标变换矩阵 T_d
  T_d = np.eye(4)
  q0 = [0,0,0,0,0,0] #初始关节角度
  df = newton_method_fdiff(T_d,q0)
  print("Jacobian Matrix:\n",df)
