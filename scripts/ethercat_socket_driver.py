#!/usr/bin/env python3
import rospy
import socket
import re
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger, TriggerResponse  # 导入服务类型

SERVER_IP = "10.10.56.214"
SERVER_PORT = 23333
global_socket = None  # 全局Socket对象
RECONNECT_INTERVAL = 1.0
INIT_PULSE = None 

import re
import rospy

def parse_init_pulses(response):
    """
    解析带大括号的初始脉冲响应（格式：{1,2,3,4,5,6}）
    :param response: 控制器返回的原始字符串（如"{123,-456,789,0,111,-222}"）
    :return: 字典格式初始脉冲 / None（解析失败）
    """
    # 1. 先校验响应是否为空
    if not response:
        rospy.logerr("初始脉冲响应为空")
        return None
    
    rospy.loginfo(f"待解析的原始响应：{response}")
    
    # 2. 第一步：去除大括号（关键！）
    # 把 {和} 替换为空，得到 "1,2,3,4,5,6" 格式
    pulse_str = response.strip().replace("{", "").replace("}", "")
    if not pulse_str:  # 去除大括号后为空（比如响应是"{}"）
        rospy.logerr("去除大括号后无有效数据")
        return None
    
    # 3. 第二步：按逗号分割成单个数字字符串
    nums_str = pulse_str.split(",")
    rospy.loginfo(f"分割后的数字字符串列表：{nums_str}")
    
    try:
        # 4. 转换为整数（过滤空字符串，兼容可能的多余逗号）
        nums = []
        for s in nums_str:
            s_stripped = s.strip()  # 去除空格（比如"{1, 2, 3}"中的空格）
            if s_stripped:  # 跳过空字符串
                nums.append(int(s_stripped))
        
        # 5. 校验是否至少6个关节的脉冲值
        if len(nums) >= 6:
            # 封装为字典（q1~q6对应6个关节）
            init_pulse = {
                'q1': nums[0], 'q2': nums[1], 'q3': nums[2],
                'q4': nums[3], 'q5': nums[4], 'q6': nums[5]
            }
            rospy.loginfo(f"初始脉冲解析成功：{init_pulse}")
            return init_pulse
        else:
            rospy.logerr(f"初始脉冲解析失败：仅找到{len(nums)}个整数，需6个")
            return None
    
    except ValueError as e:
        # 转换整数失败（比如字符串不是数字："{1,2,a,4,5,6}"）
        rospy.logerr(f"初始脉冲解析失败：数字转换错误 → {e}")
        return None

def parse_joint_angles(response, init_pulse):
    """解析关节角度（基于初始脉冲计算）"""
    if not response or not init_pulse:
        rospy.logerr("响应为空或初始脉冲未初始化，解析失败")
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

# -------------------------- Socket核心操作 --------------------------
def init_socket():
    """初始化Socket连接（全局变量操作）"""
    global global_socket
    if global_socket is not None:
        return True
    try:
        global_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 设置Socket超时（避免阻塞）
        global_socket.settimeout(5.0)
        global_socket.connect((SERVER_IP, SERVER_PORT))
        rospy.loginfo(f"成功连接到EtherCAT服务器 {SERVER_IP}:{SERVER_PORT}")
        # 初始化成功后，获取初始脉冲
        init_initial_pulse()
        return True
    except socket.error as e:
        rospy.logerr(f"Socket连接失败：{e}，{RECONNECT_INTERVAL}秒后重试")
        global_socket = None
        return False

def send_socket_cmd(cmd, expect_response=True):
    """
    发送Socket指令，可选接收响应
    :param cmd: 要发送的指令字符串
    :param expect_response: 是否需要接收响应
    :return: 响应字符串（None表示失败）
    """
    global global_socket
    # 先确保Socket连接正常
    if not init_socket():
        return None
    
    try:
        # 发送指令
        global_socket.sendall(cmd.encode('utf-8'))
        rospy.logdebug(f"发送Socket指令：{cmd}")
        
        # 如需接收响应，则读取返回数据
        if expect_response:
            # 读取响应（缓冲区设为4096，适配多数场景）
            response = global_socket.recv(4096).decode('utf-8').strip()
            rospy.logdebug(f"收到Socket响应：{response}")
            return response
        return "success"
    except socket.timeout:
        rospy.logerr("Socket通信超时")
        global_socket.close()
        global_socket = None
        return None
    except socket.error as e:
        rospy.logerr(f"Socket通信失败：{e}，尝试重连")
        global_socket.close()
        global_socket = None
        init_socket()
        return None

def init_initial_pulse():
    """初始化时获取初始脉冲值（仅执行一次）"""
    global INIT_PULSE
    if INIT_PULSE is not None:
        return
    
    # 发送获取初始脉冲的指令
    send_socket_cmd("get_joint")
    response = send_socket_cmd("get_joint")
    INIT_PULSE = parse_init_pulses(response)
    if INIT_PULSE:
        rospy.loginfo(f"初始脉冲初始化成功：{INIT_PULSE}")
    else:
        rospy.logerr("初始脉冲初始化失败，后续角度计算可能异常！")

# -------------------------- ROS服务：获取关节角度 --------------------------
def handle_get_joints_angle(req):
    """
    /get_joints_angle服务回调函数
    处理客户端请求，返回6维关节角度（deg）
    """
    # 1. 检查初始脉冲是否初始化
    if INIT_PULSE is None:
        return TriggerResponse(
            success=False,
            message="初始脉冲未初始化，无法计算关节角度"
        )
    
    # 2. 发送get_joint指令获取当前脉冲
    send_socket_cmd("get_joint")
    response = send_socket_cmd("get_joint")
    if not response:
        return TriggerResponse(
            success=False,
            message="发送get_joint指令失败，未收到控制器响应"
        )
    
    # 3. 解析关节角度
    angle_list = parse_joint_angles(response, INIT_PULSE)
    if not angle_list:
        return TriggerResponse(
            success=False,
            message=f"解析关节角度失败，原始响应：{response}"
        )
    
    # 4. 封装响应（按原客户端约定：分号分隔字符串）
    angle_str = ";".join([f"{angle:.2f}" for angle in angle_list])
    return TriggerResponse(
        success=True,
        message=angle_str
    )

# -------------------------- ROS话题：速度指令回调 --------------------------
def otg_vel_callback(msg):
    """处理/otg_velocity话题，发送Socket指令到控制器"""
    # 1. 校验数据格式
    if len(msg.data) != 6:
        rospy.logerr("Invalid otg velocity size != 6")
        return
    
    # 2. 封装指令：{v1,v2,v3,v4,v5,v6} 格式（保留2位小数）
    vel_values = [f'{v:.2f}' for v in msg.data]
    cmd = f"{{{','.join(vel_values)}}}"
    
    # 3. 发送指令（速度指令无需等待响应）
    send_socket_cmd(cmd, expect_response=False)

# -------------------------- 主函数 --------------------------
def main():
    rospy.init_node("ethercat_socket_driver", log_level=rospy.INFO)
    rospy.loginfo("EtherCAT Socket驱动节点启动，等待与服务器建立通信...")
    
    # 1. 初始化Socket（含初始脉冲）
    init_socket()
    
    # 2. 注册ROS服务：获取关节角度
    rospy.Service("/get_joints_angle", Trigger, handle_get_joints_angle)
    rospy.loginfo("ROS服务 /get_joints_angle 已注册，等待查询请求...")
    
    # 3. 订阅ROS话题：速度指令
    rospy.Subscriber("/joints_velocity", Float64MultiArray, otg_vel_callback, queue_size=10)
    rospy.loginfo("ROS订阅 /joints_velocity 已注册，开始监听速度指令...")
    
    #rospy.Subscriber("/otg_velocity", Float64MultiArray, otg_vel_callback, queue_size=10)
    #rospy.loginfo("ROS订阅 /otg_velocity 已注册，开始监听速度指令...")
    
    # 4. 保持节点运行
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("关闭Socket连接...")
        global global_socket
        if global_socket is not None:
            global_socket.close()

if __name__ == '__main__':
    main()
