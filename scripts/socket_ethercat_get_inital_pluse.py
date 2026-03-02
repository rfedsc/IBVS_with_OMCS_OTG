#!/usr/bin/env python3
import re
import socket

# 全局socket连接
global_socket = None

def send_receive_data_ethercat(server_ip, server_port, command):
    global global_socket
    # 建立连接（如果尚未连接）
    if global_socket is None:
        try:
            global_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            global_socket.connect((server_ip, server_port))
            print("成功连接到服务器")
        except socket.error as e:
            print(f"连接错误: {e}")
            return None
    try:
        # 发送命令
        global_socket.sendall(command.encode('utf-8'))
        print(f"send_command: {command}")
        # 接收响应
        response = global_socket.recv(1024).decode('utf-8')
        return response
    except socket.error as e:
        print(f"Socket error: {e}")
        global_socket = None  # 重置连接
        return None

def parse_init_pulses(response):
    """
    解析响应数据，提取初始关节脉冲值
    """
    print(f"Raw init response: {response}")
    pattern = r'-?\d+'
    nums_str = re.findall(pattern, response)
    
    try:
        nums = [int(num) for num in nums_str]
        if len(nums) >= 6:
            init_pulses = {
                'q1': nums[0],
                'q2': nums[1],
                'q3': nums[2],
                'q4': nums[3],
                'q5': nums[4],
                'q6': nums[5]
            }
            print(f"初始关节脉冲：{init_pulses}")
            return init_pulses
        else:
            print(f"初始脉冲解析失败：响应数据中仅找到{len(nums)}个整数，需要6个")
            return None
    except ValueError as e:
        print(f"初始脉冲解析失败：数字转换为整数出错 - {e}")
        return None

        
def get_joint_initial_pulses(server_ip, server_port, command):
    # 第一次发送命令
    send_receive_data_ethercat(server_ip, server_port, command)
    # 第二次发送命令并接收响应
    response = send_receive_data_ethercat(server_ip, server_port, command)
    if not response:
        print("未收到第二次命令的响应（获取初始脉冲）")
        return None
    # 解析响应得到初始脉冲
    init_pulses = parse_init_pulses(response)
    return init_pulses
    
       
# 使用示例
if __name__ == '__main__':
    # 配置参数
    SERVER_IP = '10.10.56.214'
    SERVER_PORT = 23333
    CMD = "get_joint"  # 可根据实际需求调整命令
    
    # 获取初始关节脉冲
    initial_pulses = get_joint_initial_pulses(SERVER_IP, SERVER_PORT, CMD)
    if initial_pulses:
        print(f"成功获取初始关节脉冲：{initial_pulses}")
    else:
        print("获取初始关节脉冲失败")
    
    
    
