#!/usr/bin/env python3
import rospy
import time
from jacob_cross_SDH import jacob_cross_sdh
from socket_getJntData import send_receive_data, parse_response_getJntData
from socket_moveTo import send_receive_data_moveTo,parse_response_moveTo
from socket_ethercat_get_initial_pluse import send_receive_data_ethercat,parse_init_pulses,get_joint_initial_pulses
from socket_ethercat_get_joints import send_receive_data_ethercat,parse_response_ethercat,get_joints_angle

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np

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

def get_end_velocity():
    end_v_msg = rospy.wait_for_message('/end_velocity', Twist)
    end_v = np.array([
        end_v_msg.linear.x,
        end_v_msg.linear.y,
        end_v_msg.linear.z,
        end_v_msg.angular.x,
        end_v_msg.angular.y,
        end_v_msg.angular.z
    ])
    print(f"获取到的末端速度:{end_v}")
    return end_v

def get_joints_velocity():  
    # 初始化ROS节点
    rospy.init_node('get_joints_pos_node', anonymous=True)
    #rate = rospy.Rate(100)
    server_ip = '10.10.56.214'
    server_port = 23333
    serial_number = 1
    
    #用户输入参数
    gpId = 0
    isJoint = 'true'
    ufNum = -1
    utNum = -1
    config = 0
    strPos = '"{287.502,-0.001,232.240,180.000,-0.000,180.000}"'
    isLinear = 'false'
    time_step = 0.15
    while not rospy.is_shutdown():
        try:
            # 获取机械臂关节角度
            command_getLocData = "mot.getJntData(0)"
            response = send_receive_data(server_ip, server_port, serial_number, command_getLocData)
            # 解析机械臂关节角度
            JntData = parse_response_getJntData(response)
            print(f"JntData is:{JntData}")
                  
            # 不转换为弧度
            # q_curr = JntData[0:6]
            # print(f"当前角度:\n{q_curr}")
            #转换为弧度
            q_curr = np.deg2rad(JntData[0:6])
            print(f"当前角度(弧度):\n{q_curr}")
            # 计算雅可比矩阵
            J = jacob_cross_sdh(q_curr)
            print(f"当前角度下的雅可比矩阵:\n{J}")
            # 获取末端速度
            end_velocity = get_end_velocity()
            # 求关节速度
            try:
                q_velocity = np.linalg.inv(J).dot(end_velocity)
            except np.linalg.LinAlgError:
                q_velocity = np.linalg.pinv(J).dot(end_velocity)
            print(f"计算出的关节速度:\n{q_velocity}")
            q_new_rad = q_curr+q_velocity*time_step
            q_new_deg = np.rad2deg(q_new_rad)
            within,q_new_deg_limited = check_limits(q_new_deg)
            print(f"新的角度rad:\n{q_new_rad}")
            print(f"新的角度deg:\n{q_new_deg}")
            print(f"新的角度deg(限位后):\n{q_new_deg_limited}")
            #new_strPos = "{" + ",".join(map(lambda x: f"{x:.6f}", q_new_deg)) + ",}"
            new_strPos = "{" + ",".join(map(lambda x: f"{x:.6f}", q_new_deg_limited)) + "}"
            command_moveTo = f'mot.moveTo({gpId},{isJoint},{ufNum},{utNum},{config},"{new_strPos}",{isLinear})'
            response = send_receive_data_moveTo(server_ip,server_port,serial_number,command_moveTo)
            #解析收到的响应
            Data = parse_response_moveTo(response)
            print(f"Data is:{Data}")
            
        except Exception as e:
            rospy.logerr(f"Error in get_joints_pos loop: {e}")
        #rate.sleep()

if __name__ == '__main__':
    try:
        get_joints_velocity()
    except rospy.ROSInterruptException:
        pass

