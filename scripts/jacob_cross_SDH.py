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


  #T06 = T1@T2@T3@T4@T5@T6
  T16 = T2@T3@T4@T5@T6
  T26 = T3@T4@T5@T6
  T36 = T4@T5@T6
  T46 = T5@T6
  T56 = T6

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
  P16 = T16[:3,3]
  P26 = T26[:3,3]
  P36 = T36[:3,3]
  P46 = T46[:3,3]
  P56 = T56[:3,3]
  P66 = np.array([0,0,0])
  
  #Jacobian computation using vector cross products
  J1 = np.hstack((cross_product(Z0,R00@P06),Z0))
  J2 = np.hstack((cross_product(Z1,R01@P16),Z1))
  J3 = np.hstack((cross_product(Z2,R02@P26),Z2))
  J4 = np.hstack((cross_product(Z3,R03@P36),Z3))
  J5 = np.hstack((cross_product(Z4,R04@P46),Z4))
  J6 = np.hstack((cross_product(Z5,R05@P56),Z5))

  J = np.column_stack((J1,J2,J3,J4,J5,J6))
  J_end = [R06.T@J[:3,:],R06.T@J[3:6,:]]
  J_end = np.vstack(J_end)

  return J_end



def test():
  q = np.array([10*np.pi/180,0,0,20*np.pi/180,0,0])
  J = jacob_cross_sdh(q)
  print("Jacobian Matrix:\n",J)

if __name__ == '__main__':
  test()


