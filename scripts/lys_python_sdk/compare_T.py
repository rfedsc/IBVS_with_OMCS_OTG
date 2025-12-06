import numpy as np
import math
from scipy.linalg import inv
from fkine_6dof import fkine_6dof
from elephant_robot import CvtRotationMatrixToEulerAngle,CvtEulerAngleToRotationMatrix
np.set_printoptions(suppress=True)


def degrees_to_radians(degrees):
  #将度数转换为弧度
  return degrees*math.pi/180

def radians_to_degrees():
  #将弧度转换为度数
  degree_angles = np.degrees(radian_angles)
  return degree_angles

def euler_to_rotation_matrix(roll,pitch,yaw):
  """将欧拉角转换为旋转矩阵"""
  R_x = np.array([[1,0,0],
                  [0,np.cos(roll),-np.sin(roll)],
                  [0,np.sin(roll),np.cos(roll)]
                ])
  R_y = np.array([[np.cos(pitch),0,np.sin(pitch)],
                  [0,1,0],
                  [-np.sin(pitch),0,np.cos(pitch)]
                ])
  R_z = np.array([[np.cos(yaw),-np.sin(yaw),0],
                  [np.sin(yaw),np.cos(yaw),0],
                  [0,0,1]
                  ])
  R = R_z@R_y@R_x
  return R

def actual_to_transformation_matrix(coords):
  """将实际坐标转换为变换矩阵"""
  x,y,z,roll,pitch,yaw = coords
  roll = np.radians(roll)
  pitch = np.radians(pitch)
  yaw = np.radians(yaw)
  R = euler_to_rotation_matrix(roll,pitch,yaw)
  
  T = np.eye(4)
  T[:3,:3] = R
  T[:3,3] = [x,y,z]
  return T

def test(): 
  actual_coords = [290.969,-15.654,304.096,98.975,1.095,165.750]
  actual_T = actual_to_transformation_matrix(actual_coords)
  #actual_T =CvtEulerAngleToRotationMatrix([rx,ry,rz])

  #计算得到的变换矩阵
  th1 = np.radians(0)
  th2 = np.radians(-90)
  th3 = np.radians(180)
  th4 = np.radians(0)
  th5 = np.radians(90)
  th6 = np.radians(0)

  q = np.array([th1,th2,th3,th4,th5,th6])
  computed_T = fkine_6dof(q)

  print("实际的变换矩阵:")
  print(actual_T)
  print("计算得到的变换矩阵")
  print(computed_T)

  #计算误差
  error = np.linalg.norm(actual_T-computed_T)
  print(f"误差:{error}")

if __name__ == "__main__":
  test()


