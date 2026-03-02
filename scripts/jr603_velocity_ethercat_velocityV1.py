#!/usr/bin/env python3
import rospy
import time
from jacob_cross_SDH import jacob_cross_sdh
from socket_getJntData import send_receive_data, parse_response
from socket_ethercat_get_initial_pluse import send_receive_data_ethercat, parse_init_pulses, get_joint_initial_pulses
from socket_ethercat_get_joints import send_receive_data_ethercat, parse_response_ethercat, get_joints_angle

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np

def get_end_velocity():
    """获取末端速度"""
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
    rospy.init_node('get_joints_velocity_node', anonymous=True)
    # 创建 joints_velocity 发布者
    joints_vel_pub = rospy.Publisher(
        '/joints_velocity',
        Float64MultiArray,
        queue_size=10
    )
    rate = rospy.Rate(10)  # 10Hz
    server_ip = '10.10.56.214'
    server_port = 23333
    serial_number = 1  
    
    CMD = "{0,0,0,0,0,0}"  # 可根据实际需求调整命令

    # 节点启动时获取初始关节脉冲
    initial_pulse = get_joint_initial_pulses(server_ip, server_port, CMD)
    
    # 若未获取到初始脉冲，直接退出节点
    if not initial_pulse:
        rospy.logerr("未获取到初始关节脉冲，节点退出！")
        return
    
    print(f"成功获取初始关节脉冲（仅获取一次）：{initial_pulse}")
    # --------------------------------------------------------------------------------
    while not rospy.is_shutdown():
        try:
            angles = get_joints_angle(server_ip, server_port, CMD, initial_pulse)                  
            angle_list = [angles['q1'],angles['q2'],angles['q3'],angles['q4'],angles['q5'],angles['q6']]
            # 转换为弧度
            q = np.deg2rad(angle_list)
            print(f"当前角度(弧度):\n{q}")
            
            # 计算雅可比矩阵
            J = jacob_cross_sdh(q)
            print(f"当前角度(弧度)下的雅可比矩阵:\n{J}")
            
            # 获取末端速度
            end_velocity = get_end_velocity()
            
            # 求关节速度（伪逆处理奇异性）
            try:
                q_velocity = np.linalg.inv(J).dot(end_velocity)
            except np.linalg.LinAlgError:
                q_velocity = np.linalg.pinv(J).dot(end_velocity)
            
            print(f"计算出的关节速度:\n{q_velocity}")
            
            # 发布关节速度
            msg = Float64MultiArray()
            msg.data = q_velocity.tolist()
            joints_vel_pub.publish(msg)
            
        except Exception as e:
            rospy.logerr(f"循环执行错误: {e}")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        get_joints_velocity()
    except rospy.ROSInterruptException:
        pass
