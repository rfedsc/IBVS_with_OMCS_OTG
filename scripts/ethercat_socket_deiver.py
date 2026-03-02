#!/usr/bin/env python3
import rospy
import socket
import re
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger, TriggerResponse

SERVER_IP = "10.10.56.214"
SERVER_PORT = 23333
RECONNECT_INTERVAL = 1.0  

# 全局Socket连接
global_socket = None
initial_pulse = None


def init_socket():
    """初始化Socket连接（阻塞重试直到成功）"""
    global global_socket  # 必须声明全局变量
    if global_socket is not None:
        return True  # 已有有效连接，直接返回
    
    while not rospy.is_shutdown():
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((SERVER_IP, SERVER_PORT))
            global_socket = s
            rospy.loginfo(f"成功连接到EtherCAT服务器 {SERVER_IP}:{SERVER_PORT}")
            return True
        except socket.error as e:
            rospy.logerr(f"Socket连接失败：{e}，{RECONNECT_INTERVAL}秒后重试")
            global_socket = None
            rospy.sleep(RECONNECT_INTERVAL)
    return False
    
    
"""
def init_socket():
    global global_socket
    if global_socket is not None:
        return True
    while not rospy.is_shutdown():
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((SERVER_IP, SERVER_PORT))
            global_socket = s
            rospy.loginfo(f"成功连接到EtherCAT服务器 {SERVER_IP}:{SERVER_PORT}")
            return True
        except socket.error as e:
            rospy.logerr(f"Socket连接失败: {e}，{RECONNECT_INTERVAL}秒后重试...")
            global_socket = None
            rospy.sleep(RECONNECT_INTERVAL)
    return False
"""

def send_receive(command):
    global global_socket
    if not init_socket():
        return None
    
    while not rospy.is_shutdown():
        try:
            global_socket.sendall(command.encode('utf-8'))
            rospy.logdebug(f"发送指令: {command}")
            response = global_socket.recv(1024).decode('utf-8')
            return response
        except socket.error as e:
            rospy.logerr(f"Socket通信错误: {e}，尝试重连...")
            try:
                global_socket.close()
            except:
                pass
            global_socket = None
            init_socket()
    return None

def parse_init_pulses(response):
    if not response:
        return None
    pattern = r'-?\d+'
    nums_str = re.findall(pattern, response)
    try:
        nums = [int(num) for num in nums_str]
        if len(nums) >= 6:
            return {
                'q1': nums[0], 'q2': nums[1], 'q3': nums[2],
                'q4': nums[3], 'q5': nums[4], 'q6': nums[5]
            }
        else:
            rospy.logerr(f"初始脉冲解析失败：仅找到{len(nums)}个整数，需6个")
            return None
    except ValueError as e:
        rospy.logerr(f"初始脉冲解析失败: {e}")
        return None

def parse_joint_angles(response, init_pulse):
    """解析关节角度"""
    if not response or not init_pulse:
        return None
    pattern = r'-?\d+'
    nums_str = re.findall(pattern, response)
    try:
        nums = [int(num) for num in nums_str]
        if len(nums) >= 6:
            pulse1, pulse2, pulse3, pulse4, pulse5, pulse6 = nums[:6]
            # 角度计算逻辑
            angle1 = ((pulse1 - init_pulse['q1']) / 8388608 * 360) / 80.98 + 0
            angle2 = ((pulse2 - init_pulse['q2']) / 8388608 * 360) / 80.98 - 90
            angle3 = ((pulse3 - init_pulse['q3']) / 8388608 * 360) / 80.98 + 180
            angle4 = ((pulse4 - init_pulse['q4']) / 8388608 * 360) / 80.98 + 0
            angle5 = ((pulse5 - init_pulse['q5']) / 8388608 * 360) / 80.98 + 90
            angle6 = ((pulse6 - init_pulse['q6']) / 8388608 * 360) / 80.98 + 0
            return [angle1, angle2, angle3, angle4, angle5, angle6]
        else:
            rospy.logerr(f"关节角度解析失败：仅找到{len(nums)}个整数，需6个")
            return None
    except (ValueError, KeyError) as e:
        rospy.logerr(f"关节角度解析失败: {e}")
        return None

# -------------------------- ROS 接口 --------------------------
def get_joints_service(req):
    """ROS服务：获取关节角度（阻塞直到获取成功/节点关闭）"""
    global initial_pulse
    while initial_pulse is None and not rospy.is_shutdown():
        rospy.logwarn("未获取初始脉冲，重试中...")
        response = send_receive("get_joint")
        initial_pulse = parse_init_pulses(response)
        if initial_pulse is None:
            rospy.sleep(RECONNECT_INTERVAL)
    if rospy.is_shutdown():
        return TriggerResponse(success=False, message="节点已关闭")
        
    # 获取关节角度
    while not rospy.is_shutdown():
        joint_response = send_receive("get_joint")
        angles = parse_joint_angles(joint_response, initial_pulse)
        if angles is not None:
            return TriggerResponse(
                success=True,
                message=";".join([f"{a:.6f}" for a in angles])
            )
        rospy.logwarn("关节角度获取失败，重试中...")
        rospy.sleep(RECONNECT_INTERVAL)
    
    return TriggerResponse(success=False, message="节点已关闭")
    
def vel_cmd_callback(msg):
    """ROS订阅：接收速度指令并发送（修复全局变量+空值判断）"""
    # 1. 强制声明使用全局的global_socket（核心修复点）
    global global_socket
    # 2. 校验指令长度
    if len(msg.data) != 6:
        rospy.logerr("速度指令长度错误，需6个值（当前：%d）", len(msg.data))
        return
    # 3. 格式化指令为服务器识别的格式
    cmd = "{{{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}}}".format(
        msg.data[0], msg.data[1], msg.data[2],
        msg.data[3], msg.data[4], msg.data[5]
    )
    # 4. 阻塞重试发送（确保global_socket有效）
    while not rospy.is_shutdown():
        # 先检查Socket连接是否有效
        if global_socket is None:
            rospy.logwarn("Socket连接未建立，尝试重新连接...")
            # 调用init_socket重建连接（init_socket需也声明global）
            if not init_socket():
                rospy.sleep(RECONNECT_INTERVAL)
                continue
        # 5. 发送指令（此时global_socket必非空）
        try:
            global_socket.sendall(cmd.encode('utf-8'))
            rospy.logdebug(f"成功发送速度指令：{cmd}")
            break  # 发送成功，退出重试循环
        except socket.error as e:
            rospy.logerr(f"发送指令失败：{e}，{RECONNECT_INTERVAL}秒后重试")
            # 关闭无效连接，重置全局变量
            try:
                global_socket.close()
            except:
                pass
            global_socket = None
            rospy.sleep(RECONNECT_INTERVAL)
 
"""    
def vel_cmd_callback(msg):
    """ROS订阅：接收速度指令并发送"""
    # 关键：显式声明使用全局的 global_socket
    global global_socket
    if len(msg.data)!=6:
        rospy.logerr("速度指令长度错误，需6个值")
        return
    # 格式化速度指令
    cmd = "{{{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}}}".format(
        msg.data[0],msg.data[1],msg.data[2],
        msg.data[3],msg.data[4],msg.data[5]
    )
    # 阻塞直到连接成功并发送
    while not rospy.is_shutdown():
        # 先确保init_socket成功，且global_socket非空
        if init_socket() and global_socket is not None:
            try:
                global_socket.sendall(cmd.encode('utf-8'))
                rospy.logdebug(f"发送速度指令: {cmd}")
                break  # 发送成功，退出重试循环
            except socket.error as e:
                rospy.logerr(f"发送速度指令失败: {e}，重试中...")
                try:
                    global_socket.close()
                except:
                    pass
                global_socket = None  # 重置连接状态
                rospy.sleep(RECONNECT_INTERVAL)
        else:
            # init_socket失败，等待后重试
            rospy.logwarn("Socket连接未建立，重试中...")
            rospy.sleep(RECONNECT_INTERVAL)
"""
"""
def vel_cmd_callback(msg):
    """ROS订阅：接收速度指令并发送"""
    if len(msg.data) != 6:
        rospy.logerr("速度指令长度错误，需6个值")
        return
    # 格式化速度指令
    cmd = "{{{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}}}".format(
        msg.data[0], msg.data[1], msg.data[2],
        msg.data[3], msg.data[4], msg.data[5]
    )
    # 阻塞直到连接成功并发送
    while not rospy.is_shutdown():
        if init_socket():
            try:
                global_socket.sendall(cmd.encode('utf-8'))
                rospy.logdebug(f"发送速度指令: {cmd}")
                break
            except socket.error as e:
                rospy.logerr(f"发送速度指令失败: {e}，重试中...")
                try:
                    global_socket.close()
                except:
                    pass
                global_socket = None
                rospy.sleep(RECONNECT_INTERVAL)
"""

def main():
    global initial_pulse
    rospy.init_node("ethercat_socket_driver", anonymous=False)
    rospy.loginfo("EtherCAT Socket驱动节点启动，等待与服务器建立通信...")

    # 1. 强制初始化：必须获取到初始脉冲才启动后续逻辑（阻塞直到成功）
    rospy.loginfo("获取初始关节脉冲...")
    while initial_pulse is None and not rospy.is_shutdown():
        response = send_receive("get_joint")
        initial_pulse = parse_init_pulses(response)
        if initial_pulse is None:
            rospy.logerr("初始脉冲获取失败，重试中...")
            rospy.sleep(RECONNECT_INTERVAL)
    
    # 若节点被关闭，直接退出
    if rospy.is_shutdown():
        rospy.loginfo("节点被关闭，退出初始化")
        return
    
    rospy.loginfo(f"成功获取初始脉冲：{initial_pulse}")

    # 2. 注册ROS服务（仅在初始脉冲获取成功后注册）
    rospy.Service('/get_joints_angle', Trigger, get_joints_service)
    rospy.loginfo("ROS服务 /get_joints_angle 已注册")

    # 3. 注册ROS订阅（仅在初始脉冲获取成功后注册）
    rospy.Subscriber("/otg_velocity", Float64MultiArray, vel_cmd_callback, queue_size=10)
    rospy.loginfo("ROS订阅 /otg_velocity 已注册")

    # 4. 节点循环（带退出清理）
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("接收到退出信号")
    finally:
        # 关闭Socket连接
        if global_socket is not None:
            try:
                global_socket.close()
            except:
                pass
            rospy.loginfo("Socket连接已关闭")

if __name__ == '__main__':
    main()
