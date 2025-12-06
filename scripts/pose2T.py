import numpy as np
np.set_printoptions(suppress=True)  # 禁止科学计数法
def euler_to_rotation_matrix(roll,pitch,yaw):
    """
    将欧拉角转换为旋转矩阵(ZYX)
    roll:绕x轴的旋转角
    pitch:绕y轴的旋转角
    yaw:绕Z轴的旋转角
    """
    R_x = np.array([
                  [1,0,0],
                  [0,np.cos(roll),-np.sin(roll)],
                  [0,np.sin(roll),np.cos(roll)]
          ])
    R_y = np.array([
                  [np.cos(pitch),0,np.sin(pitch)],
                  [0,1,0],
                  [-np.sin(pitch),0,np.cos(pitch)]
          ])
    R_z = np.array([
                    [np.cos(yaw),-np.sin(yaw),0],
                    [np.sin(yaw),np.cos(yaw),0],
                    [0,0,1]
          ])
    return R_z@R_y@R_x

def cartesian_to_homogeneous(x,y,z,roll,pitch,yaw):
    """
    将笛卡尔坐标和欧拉角转换为齐次变换矩阵
    roll:绕x轴的旋转角
    pitch:绕y轴的旋转角
    yaw:绕z轴的旋转角
    """
    #转换为旋转矩阵
    R = euler_to_rotation_matrix(roll,pitch,yaw)
    #构建齐次变换矩阵
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = [x,y,z]
    return T

def test():
    x,y,z = 287.502/1000,-0.001/1000,232.239/1000
    roll,pitch,yaw = np.radians(180),np.radians(0),np.radians(180)
    T = cartesian_to_homogeneous(x,y,z,roll,pitch,yaw)
    print("齐次变换矩阵:")
    print(T)


if __name__ == "__main__":
    test()

















