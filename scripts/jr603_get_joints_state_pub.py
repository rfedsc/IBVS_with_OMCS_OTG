#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from jacob_cross_SDH import jacob_cross_sdh
from socket_getJntData import send_receive_data, parse_response_getJntData
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

# 全局设置：禁用科学计数法，打印精度6位小数
np.set_printoptions(suppress=True, precision=6)


def get_end_velocity():
    """
    订阅末端速度话题 /end_velocity
    返回:
        np.array: 6维末端速度 [vx, vy, vz, wx, wy, wz] (m/s, rad/s)
    """
    msg = rospy.wait_for_message("/end_velocity", Twist)
    return np.array([
        msg.linear.x,
        msg.linear.y,
        msg.linear.z,
        msg.angular.x,
        msg.angular.y,
        msg.angular.z
    ], dtype=float)


def get_joints_velocity():
    """主循环：获取关节状态并发布相关话题"""
    # 初始化ROS节点
    rospy.init_node("jr603_get_joints_state_pub_node", anonymous=True)

    # 定义ROS发布器
    pub_q_curr = rospy.Publisher("/joint_curr_position_rad",Float64MultiArray,queue_size=10)
    pub_q_vel_last = rospy.Publisher("/joint_last_velocity_rad_s",Float64MultiArray,queue_size=10)
    pub_q_vel_next = rospy.Publisher("/joint_next_velocity_rad_s",Float64MultiArray,queue_size=10)
    pub_q_target_rad = rospy.Publisher("/joint_target_position_rad",Float64MultiArray,queue_size=10)

    # 初始化上一时刻关节速度
    q_velocity_last = np.zeros(6, dtype=float)

    # Socket通信参数
    SERVER_IP = '10.10.56.214'
    SERVER_PORT = 23333
    SERIAL_NUMBER = 1
    TIME_STEP = 0.15  # 积分时间步长 (s)

    # 主循环
    rate = rospy.Rate(100)  # 100Hz循环
    while not rospy.is_shutdown():
        try:
            # 1. 获取机械臂当前关节角度（deg→rad）
            cmd = "mot.getJntData(0)"
            response = send_receive_data(SERVER_IP, SERVER_PORT, SERIAL_NUMBER, cmd)
            jnt_data = parse_response_getJntData(response)
            
            print(f"获取的机械臂当前角度(deg): {jnt_data}\n")
            q_curr_rad = np.deg2rad(np.array(jnt_data[0:6], dtype=float))
            print(f"获取的机械臂当前角度(rad): {q_curr_rad}\n")

            # 发布当前关节角度 (rad)
            pub_q_curr.publish(Float64MultiArray(data=q_curr_rad.tolist()))
            print(f"发布的机械臂当前角度(rad): {q_curr_rad}\n")

            # 2. 计算雅可比矩阵
            J = jacob_cross_sdh(q_curr_rad)

            # 3. 获取末端速度
            end_v = get_end_velocity()
            print(f"计算得到的末端速度(v_end): {end_v}\n")

            # 4. 求解关节速度（逆雅可比，奇异值鲁棒处理）
            try:
                q_velocity = np.linalg.inv(J).dot(end_v)
            except np.linalg.LinAlgError:
                q_velocity = np.linalg.pinv(J).dot(end_v)

            # 5. 发布关节速度相关话题
            # 发布下一次目标关节速度 (rad/s)
            pub_q_vel_next.publish(Float64MultiArray(data=q_velocity.tolist()))
            print(f"目标关节速度(q_velocity): {q_velocity}\n")
            
            # 发布上一时刻关节速度 (rad/s)
            pub_q_vel_last.publish(Float64MultiArray(data=q_velocity_last.tolist()))
            print(f"上次的目标关节速度(q_velocity_last): {q_velocity_last}\n")
            
            # 更新上一时刻速度
            q_velocity_last = q_velocity.copy()

            # 6. 积分计算下一时刻目标关节角度 (rad)
            q_new_rad = q_curr_rad + q_velocity * TIME_STEP
            q_new_deg = np.rad2deg(q_new_rad)  # 更规范的弧度转角度
            
            print(f"目标关节角度(不加ruckig_pos, rad): {q_new_rad}\n")
            print(f"目标关节角度(不加ruckig_pos, deg): {q_new_deg}\n")
            
            # 发布积分后的目标关节角度 (rad)
            pub_q_target_rad.publish(Float64MultiArray(data=q_new_rad.tolist()))

            rate.sleep()  # 按100Hz循环

        except Exception as e:
            rospy.logerr(f"运行错误: {e}")
            rospy.sleep(0.01)


if __name__ == '__main__':
    """主函数入口"""
    try:
        get_joints_velocity()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被中断，正常退出")
