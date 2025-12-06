#!/usr/bin/env python3
import rospy
import time
from jacob_cross_SDH import jacob_cross_sdh
from socket_getJntData import send_receive_data, parse_response
from socket_moveTo import send_receive_data_moveTo, parse_response_moveTo
from geometry_msgs.msg import Twist
from lys_visp_demo.msg import PixelCoordinates
from lys_visp_demo.msg import AprilTagCorners
from std_msgs.msg import Float32MultiArray
from pid_ctrl import PIDController
import numpy as np

#假设每次控制周期为0.1秒
#定义关节角度限位
joint_limits = {
    1:(-180,180),
    2:(-155,5),
    3:(-20,240),
    4:(-180,180),
    5:(-95,95),
    6:(-360,360),

}

global target_pixel, current_pixel
target_pixel = np.zeros((4,2))
current_pixel = np.zeros((4, 2))
q = []
global q_output
q_output = [0]*6

flag = False

def pixel_callback(msg):
    '''处理像素坐标的回调函数'''
    global target_pixel
    pixel_coords = list(msg.pixel)

    # 检查传入的数据是否足够
    if len(pixel_coords) < 8:
        rospy.logwarn("接收到的像素数据不足，无法处理")
        return

    reordered_pixels = [
        [int(pixel_coords[6]), int(pixel_coords[7])],  # 第4位
        [int(pixel_coords[4]), int(pixel_coords[5])],  # 第3位
        [int(pixel_coords[2]), int(pixel_coords[3])], # 第2位
        [int(pixel_coords[0]), int(pixel_coords[1])]   # 第1位
    ]

    if len(target_pixel) >= 4:
        target_pixel = []
        rospy.loginfo("接收到4对素,清空数据以开始下一次循环")

    target_pixel = np.array(reordered_pixels)  # 更新为直接重新赋值为新的数组
    rospy.loginfo(f"接收到的目标像素:\r\n {target_pixel}")
    


def corners_callback(msg):
    '''回调处理AprilTag角度点坐标'''
    global current_pixel
    if len(msg.corners) == 4:
        current_pixel = np.zeros((4,2))#清零
        current_pixel = np.array([[p.x,p.y] for p in msg.corners])
        rospy.loginfo(f"接收到的Apriltag角点坐标:\r\n {current_pixel}")
    else:
        rospy.logwarn("AprilTag角点数量不正确")


def error_callback(msg):
    #处理误差消息
    error_vector = np.array(msg.data)

def check_limits(new_q):
    '''检查新计算出来的关节角度是否超出限位,并调整超出部分 '''
    within_limits = True
    for i in range(6):
        joint_min,joint_max = joint_limits[i+1]
        if new_q[i]<joint_min:
            #超过最小限值,将角度设置为最小值
            error = joint_min - new_q[i]
            new_q[i] = joint_min
            print(f"关节{i+1}的角度超出了最小限位({joint_min}),已调整为最小值{new_q[i]},误差:{error:.6f}")
            within_limits = False
        elif new_q[i]>joint_max:
            #超过最大限值,将角度设置为最大数值
            error = new_q[i] - joint_max
            new_q[i] = joint_max
            print(f"关节{i+1}的角度超出了最大限位({joint_max}),已经调整为最大值{new_q[i]},误差:{error:.6f}")
            within_limits = False
        
    return within_limits,new_q



def pid_ctrl1():
    '''PID控制器实现'''
    if len(target_pixel)==4 and len(current_pixel)==4:
        target = target_pixel[0][0]
        current = current_pixel[0][0]
        pid = PIDController(0.001,0.001,0.001,target)
        output = pid.compute(current)
        if abs(pid.prev_error) > abs(pid.error):
            output1 = -1*output
        else:
            output1 = output 
        q_output[0] = q[0] + output
        rospy.loginfo(f"PID控制输出1: {output}, 调整后的关节1角度: {q_output[0]}")

def pid_ctrl2():
    '''PID控制器实现'''
    if len(target_pixel)==4 and len(current_pixel)==4:
        target = target_pixel[0][1]
        current = current_pixel[0][1]
        pid = PIDController(0.001,0.001,0.001,target)
        output = pid.compute(current)
        if abs(pid.prev_error) > abs(pid.error):
            output = -1*output
        else:
            output = output
        q_output[1] = q[1]+ output
        rospy.loginfo(f"PID控制输出2: {output}, 调整后的关节2角度: {q_output[1]}")
def pid_ctrl3():
    '''PID控制器实现'''
    if len(target_pixel)==4 and len(current_pixel)==4:
        target = target_pixel[1][0]
        current = current_pixel[1][0]
        pid = PIDController(0.001,0.001,0.001,target)
        output = pid.compute(current)
        if abs(pid.prev_error) > abs(pid.error):
            output1 = -1*output
        else:
            output1 = output
        q_output[2] = q[2] + output
        rospy.loginfo(f"PID控制输出3: {output}, 调整后的关节3角度: {q_output[2]}")

def pid_ctrl4():
    '''PID控制器实现'''
    if len(target_pixel)==4 and len(current_pixel)==4:
        target = target_pixel[1][1]
        current = current_pixel[1][1]
        pid = PIDController(0.001,0.001,0.001,target)
        output = pid.compute(current)
        if abs(pid.prev_error) > abs(pid.error):
            output = -1*output
        else:
            output = output
        q_output[3] = q[3]+ output
        rospy.loginfo(f"PID控制输出4: {output}, 调整后的关节4角度: {q_output[3]}")

def pid_ctrl5():
    '''PID控制器实现'''
    if len(target_pixel)==4 and len(current_pixel)==4:
        target = target_pixel[2][0]
        current = current_pixel[2][0]
        pid = PIDController(0.001,0.001,0.001,target)
        output = pid.compute(current)
        if abs(pid.prev_error) > abs(pid.error):
            output1 = -1*output
        else:
            output1 = output
        q_output[4] = q[4] + output
        rospy.loginfo(f"PID控制输出5: {output}, 调整后的关节5角度: {q_output[4]}")

def pid_ctrl6():
    '''PID控制器实现'''
    if len(target_pixel)==4 and len(current_pixel)==4:
        target = target_pixel[2][1]
        current = current_pixel[2][1]
        pid = PIDController(0.001,0.001,0.001,target)
        output = pid.compute(current)
        if abs(pid.prev_error) > abs(pid.error):
            output = -1*output
        else:
            output = output
        q_output[5] = q[5]+ output
        #角度更新完成
        global flag 
        flag = True
        rospy.loginfo(f"PID控制输出6: {output}, 调整后的关节6角度: {q_output[0]}")




def move_to_target_joints(q_out):

    server_ip = '10.10.56.214'
    server_port = 23333
    serial_number = 1

    print(f"计算出的新角度是:{q_out}")

    ret,q_rectified = check_limits(q_out)
    if not ret:
        print(f"修正后的新角度是:{q_rectified}")
    else:
        print(f"新角度在限位范围内:{q_rectified}")
    
    #给机械臂发送关节角度
    gpId = 0
    isJoint = 'true'
    ufNum = -1
    utNum = -1
    config = 0
    strPos = "{" + ",".join(map(lambda x: f"{x:.6f}", q_rectified)) + ",}"
    isLinear = 'false'
    command = f'mot.moveTo({gpId},{isJoint},{ufNum},{utNum},{config},"{strPos}",{isLinear})'
    print(f"发送的命令是{command}")
    #发送数据并接收响应
    response = send_receive_data_moveTo(server_ip,server_port,serial_number,command)
    #解析收到的响应
    Data = parse_response_moveTo(response)



   
def test_pid():
    rospy.init_node('jr603_pid_ctrl_node',anonymous=True)
    rospy.Subscriber('/image_pixel',PixelCoordinates,pixel_callback)
    rospy.Subscriber('/apriltag_corners',AprilTagCorners,corners_callback)
    server_ip = '10.10.56.214'
    server_port = 23333
    serial_number = 1

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # 获取机械臂关节数据
        command = "mot.getJntData(0)"
        # 发送数据并接收响应
        response = send_receive_data(server_ip, server_port, serial_number, command)
        # 解析收到的响应
        Data = parse_response(response)
        global q
        q = Data[0:6]

        if len(target_pixel)==4 and len(current_pixel)==4:
            pid_ctrl1()
            pid_ctrl2()
            pid_ctrl3()
            pid_ctrl4()
            pid_ctrl5()
            pid_ctrl6()
            global flag
            if flag:
                move_to_target_joints(q_output)
                flag = False
            else:
                print(f"角度没有更新完成")
            
        rate.sleep()    

if __name__ == '__main__':
    test_pid()



 
