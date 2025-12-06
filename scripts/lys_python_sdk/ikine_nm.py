import numpy as np
from scipy.linalg import inv
from fkine_6dof import fkine_6dof,transl,trotx,trotz
from newton_method_fdiff import newton_method_fdiff
import math
import cProfile

"""
def newton_method_fdiff(T_d,q):
  #这个函数需要根据具体情况进行数值微分,以下为占位符
  df = np.eye(12,6) #占位符,应该替换为实际的数值微分计算
  return df
"""

def degrees_to_radians(degrees):
  #将度数转换为弧度
  return np.radians(degrees)

def radians_to_degrees(radian_angles):
  #将弧度转换为度数
  return np.degrees(radian_angles)

def ikine_nm(T_d,q0,error=1,max_iter=10):
  q = np.array(q0) #初始角度
  T_d = np.array(T_d) #将目标矩阵转换为numpy数组
  for i in range(max_iter): #最大迭代次数
    T06c = fkine_6dof(q)
    
    #计算位置和姿态误差
    F = np.array([
        T06c[0,3]-T_d[0,3],
        T06c[1,3]-T_d[1,3],
        T06c[2,3]-T_d[2,3],
        T06c[1,2]-T_d[1,2],
        T06c[2,2]-T_d[2,2],
        T06c[2,1]-T_d[2,1],
        T06c[0,0]-T_d[0,0],
        T06c[0,1]-T_d[0,1],
        T06c[0,2]-T_d[0,2],
        T06c[1,0]-T_d[1,0],
        T06c[1,1]-T_d[1,1],
        T06c[2,0]-T_d[2,0],
        ])
    
    #计算雅可比矩阵
    df = newton_method_fdiff(T_d,q)
  
    #更新关节角度
    q = q - inv(df.T @ df)@df.T @ F
    
    #处理角度范围
    q = np.mod(q,2*np.pi)
    q[q>np.pi] -= 2*np.pi

    q_error = np.linalg.norm(F)
    if q_error < error:
      print('求解完成')
      break

  return q

def test():
  print("开始运行逆运动学求解")

  T_d = [[1,0,0,254],
  [0,1,0,0],
  [0,0,1,368],
  [0,0,0,1]]

  q0 = [0,0,0,0,0,0] #初始关节角度(可以根据具体机械臂情况进行调整)
  q_solution = ikine_nm(T_d,q0)
  q_solution_degrees = radians_to_degrees(q_solution)
  print("计算得到的关节角度(弧度):",q_solution)
  print("计算得到的关节角度(度数):",q_solution_degrees)

if __name__ == "__main__":
  cProfile.run('test()')
