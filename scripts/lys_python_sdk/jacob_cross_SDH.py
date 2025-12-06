import numpy as np
from scipy.spatial.transform import Rotation as R
from dh import d,a,alp,offset
from fkine_6dof import fkine_6dof,transl,trotx,trotz

def t2r(T):
  return T[:3,:3]

def cross_product(a,b):
  return np.cross(a,b)

def jacob_cross_sdh(q):

  thd = q+offset
  #Transformation matrices
  T0 = trotz(0)@transl(0,0,0)@trotx(0)@transl(0,0,0)
  T1 = trotz(thd[0])@transl(0,0,d[0])@trotx(alp[0])@transl(a[0],0,0)
  T2 = trotz(thd[1])@transl(0,0,d[1])@trotx(alp[1])@transl(a[1],0,0)
  T3 = trotz(thd[2])@transl(0,0,d[2])@trotx(alp[2])@transl(a[2],0,0)
  T4 = trotz(thd[3])@transl(0,0,d[3])@trotx(alp[3])@transl(a[3],0,0)
  T5 = trotz(thd[4])@transl(0,0,d[4])@trotx(alp[4])@transl(a[4],0,0)
  T6 = trotz(thd[5])@transl(0,0,d[5])@trotx(alp[5])@transl(a[5],0,0)
  
  #Transformation matrices relative to the base frame
  T00 = T0
  T01 = T1
  T02 = T1@T2
  T03 = T1@T2@T3
  T04 = T1@T2@T3@T4
  T05 = T1@T2@T3@T4@T5
  T06 = T1@T2@T3@T4@T5@T6

  #Extract rotation matrices
  R00 = t2r(T00)
  R01 = t2r(T01)
  R02 = t2r(T02)
  R03 = t2r(T03)
  R04 = t2r(T04)
  R05 = t2r(T05)
  R06 = t2r(T06)

  #Z axes
  Z0 = R00[:,2]
  Z1 = R01[:,2]
  Z2 = R02[:,2]
  Z3 = R03[:,2]
  Z4 = R04[:,2]
  Z5 = R05[:,2]
  Z6 = R06[:,2]

  #Position vectors relative to the end-effector frame
  P06 = T06[:3,3]
  P16 = T02[:3,3]
  P26 = T03[:3,3]
  P36 = T04[:3,3]
  P46 = T05[:3,3]
  P56 = T06[:3,3]
  P66 = np.array([0,0,0])
  
  #Jacobian computation using vector cross products
  J1 = np.hstack((cross_product(Z0,R00@P06),Z0))
  J2 = np.hstack((cross_product(Z1,R01@P16),Z1))
  J3 = np.hstack((cross_product(Z2,R02@P26),Z2))
  J4 = np.hstack((cross_product(Z3,R03@P36),Z3))
  J5 = np.hstack((cross_product(Z4,R04@P46),Z4))
  J6 = np.hstack((cross_product(Z5,R05@P56),Z5))

  J = np.column_stack((J1,J2,J3,J4,J5,J6))

  return J


#通过数值微分验证雅可比矩阵的结果
def test_diff():
  #数值微分的步长,模拟关节角度的变化
  epsilon = 1e-6
  #用来存储通过数值微分计算出来的雅可比矩阵
  J_numeric = np.zeros((6,6))
  #关节角度向量,表示六个关节全为零的关节角度配置
  q = np.array([0,0,0,0,0,0])
  #通过向量积法计算雅可比矩阵
  J = jacob_cross_sdh(q)

  #循环计算数值雅可比矩阵
  for i in range(6):
    dq = np.zeros(6)
    dq[i] = epsilon
    
    T_plus = fkine_6dof(q+dq)
    T_minus = fkine_6dof(q-dq)
    pos_plus = T_plus[:3,3]
    pos_minus = T_minus[:3,3]
    
    # Assuming Euler angles for the rotation part
    rot_plus = R.from_matrix(T_plus[:3,:3]).as_euler('xyz',degrees=False)
    rot_minus = R.from_matrix(T_minus[:3,:3]).as_euler('xyz',degrees=False)

    diff_pos = (pos_plus-pos_minus)/(2*epsilon)
    diff_rot = (rot_plus-rot_minus)/(2*epsilon)
    J_numeric[:,i] = np.hstack((diff_pos,diff_rot))
    


  print("Numeric Jacobian:\n",J_numeric)
  print("Jacobian Matrix:\n",J)
  print("Difference between analytic and numeric Jacobian:\n",J-J_numeric)





def test():
  q = np.array([0,0,0,0,0,0])
  J = jacob_cross_sdh(q)
  print("Jacobian Matrix:\n",J)

if __name__ == '__main__':
  test()


