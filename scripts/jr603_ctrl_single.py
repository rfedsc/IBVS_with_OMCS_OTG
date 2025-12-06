#!/usr/bin/env python3
import rospy
import tf
import numpy as np
from ikine_jr603 import forward_kinematics,inverse_kinematics
from socket_getJntData import send_receive_data,parse_response
from socket_moveTo import send_receive_data_moveTo,parse_response_moveTo
from dh import a,d,alp,offset,eMc


def get_homogeneous_transform(listener,from_frame,to_frame):
    #等待变换可用
    listener.waitForTransform(from_frame,to_frame,rospy.Time(0),rospy.Duration(4.0))
    try:
        #获取变换信息
        (trans,rot) = listener.lookupTransform(from_frame,to_frame,rospy.Time(0))
        #创建齐次变换矩阵
        T = np.eye(4)
        #填充平移部分
        T[0:3,3] = trans
        #将四元数转换为旋转矩阵
        quat = tf.transformations.quaternion_matrix(rot)
        T[0:3,0:3] = quat[0:3,0:3]
        return T
    except(tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException):
        rospy.logerr(f"Could not get transform from {from_frame} to {to_frame}")
        return None

def publish_transform(T,parent_frame,child_frame):
    broadcaster = tf.TransformBroadcaster()
    #提取平移和旋转
    translation = T[0:3,3]
    rotation = tf.transformations.quaternion_from_matrix(T)
    #发布TF变换
    broadcaster.sendTransform(translation,rotation,rospy.Time.now(),child_frame,parent_frame)

def test():
    rospy.init_node('jr603_ctrl_node')
    server_ip = '10.10.56.214'
    server_port = 23333
    serial_number = 1
    
    #创建一个tf广播器
    broadcaster = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    


    #获取机械臂关节数据
    command = "mot.getJntData(0)"
    #发送数据并接收响应
    response = send_receive_data(server_ip,server_port,serial_number,command)
    #解析收到的响应
    Data = parse_response(response)
    q = Data[0:6]

    #计算T06
    T06 = forward_kinematics(q)

    #发布从基坐标系到机械末端坐标系的tf变换
    publish_transform(T06,"base_frame","usb_cam")
    #发布从机械臂末端坐标系到相机坐标系的tf变换
    #publish_transform(eMc,"end_frame","usb_cam")


    #从TF获取AprilTag的位姿
    listener = tf.TransformListener()
    listener.waitForTransform("usb_cam","Tag36h11_10",rospy.Time(0),rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform("usb_cam","Tag36h11_10",rospy.Time(0))

    #将平移和旋转矩阵转换为齐次变换矩阵
    T_apriltag = np.eye(4)
    T_apriltag[0:3,3] = trans
    quat = tf.transformations.quaternion_matrix(rot)
    T_apriltag[0:3,0:3] = quat[0:3,0:3]

    T_apriltag_inverse = np.linalg.inv(T_apriltag)
    eMc_inverse = np.linalg.inv(eMc)

    #eMc为相机外参,即相机相对于机械臂末端的齐次变换矩阵
    T_object = eMc@T06
    q_result = inverse_kinematics(T_object)
    
    #选择所有有效的解
    valid_solution = []
    for q_sol in q_result:
    #检查所有值是否均不为-1
        if np.all(q_sol != -1):
            valid_solution.append(q_sol)
    valid_solution = np.array(valid_solution)
    if valid_solution.size > 0:
        #选择一组有效的解
        q_1 = valid_solution[0]
        #格式化为字符串
        solution_str = "{"+",".join(f"{theta:.6f}" for theta in q_1)+"}"
        print(f"Selection solution:{solution_str}")
    else:
        print("No valid solution found")


    rospy.loginfo(f"Data is:{Data}")
    rospy.loginfo(f"q is:{q}")
    rospy.loginfo(f"T06 is:\n {T06}")
    rospy.loginfo(f"T_object is:\n{T_object}")
    rospy.loginfo(f"T_apriltag is:\n{T_apriltag}")
    rospy.loginfo(f"q_result is:\n{q_result}")
    rospy.loginfo(f"valid solution is:\n {valid_solution}")

    
    #给机械臂发送关节角度
    gpId = 0
    isJoint = 'true'
    ufNum = -1
    utNum = -1
    config = 0
    strPos = solution_str
    isLinear = 'false'
    command = f'mot.moveTo({gpId},{isJoint},{ufNum},{utNum},{config},"{strPos}",{isLinear})'
    #发送数据并接收响应
    response = send_receive_data_moveTo(server_ip,server_port,serial_number,command)
    #解析收到的响应
    Data = parse_response_moveTo(response)



if __name__ == '__main__':
    test()

'''
#矩阵相乘
result = T06

q = inverse_kinematics(result)

#打印结果
print(f"Result of oMc*eMc*T06:\n {result}")
print(f"q is:\n {q}")
'''




