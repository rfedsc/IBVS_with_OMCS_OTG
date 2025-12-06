#!/usr/bin/env python3
import rospy
import time
from jacob_cross_SDH import jacob_cross_sdh
from socket_getJntData import send_receive_data, parse_response
from socket_moveTo import send_receive_data_moveTo, parse_response_moveTo
from geometry_msgs.msg import Twist
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
        else:
            print("角度没有超过限位")
        
    return within_limits,new_q



def move_to_target_joints(q_velocity):

    time_step = 0.1
    server_ip = '10.10.56.214'
    server_port = 23333
    serial_number = 1

    # 获取机械臂关节数据
    command = "mot.getJntData(0)"
    # 发送数据并接收响应
    response = send_receive_data(server_ip, server_port, serial_number, command)
    # 解析收到的响应
    Data = parse_response(response)
    current_q = Data[0:6]

    #将q_velocity由rad/s转换为deg/s
    q_velocity = q_velocity*(180/np.pi)
    #计算新的关节角度(增量)
    delta_q = q_velocity * time_step
    new_q = current_q + delta_q

    print(f"计算出的新角度是:{new_q}")

    ret,q_rectified = check_limits(new_q)
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

def get_end_velocity():
    rospy.init_node('get_velocity',anonymous=True)
    end_v_msg = rospy.wait_for_message('/end_velocity',Twist)
    #获取末端速度
    end_v = np.array([end_v_msg.linear.x,
                      end_v_msg.linear.y,
                      end_v_msg.linear.z,
                      end_v_msg.angular.z,
                      end_v_msg.angular.y,
                      end_v_msg.angular.x])

    print(f"获取到的末端速度:{end_v}")
    return end_v

    
def test_jacob():   
        server_ip = '10.10.56.214'
        server_port = 23333
        serial_number = 1

        # 获取机械臂关节数据
        command = "mot.getJntData(0)"
        # 发送数据并接收响应
        response = send_receive_data(server_ip, server_port, serial_number, command)
        # 解析收到的响应
        Data = parse_response(response)
        q = Data[0:6]
        #将角度转换为弧度
        q = np.deg2rad(q)
        #当前角度
        print(f"当前角度(弧度):\n{q}")
        #定义机械臂末端速度
        end_velocity = get_end_velocity()
        
        print(f"期望的末端速度:\n{end_velocity}")

        #计算关节速度
        J = jacob_cross_sdh(q)
        print(f"当前角度(弧度)下的雅可比矩阵:\n{J}")

        try:    
            q_velocity = np.linalg.inv(J).dot(end_velocity)
        except np.linalg.LinAlgError:
            q_velocity = np.linalg.pinv(J).dot(end_velocity)

        print(f"计算出的关节速度:\n{q_velocity}")
        return q_velocity

def test():
    time_step = 0.1
    start_time = time.time()  # 记录开始时间
    while time.time() - start_time < 5000:  # 持续运行5秒
        q_velocity = test_jacob()  # 计算关节速度
        move_to_target_joints(q_velocity)  # 移动机械臂
        time.sleep(time_step)  # 等待下一次控制周期

if __name__ == '__main__':
    test()



 
