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

def parse_response_ethercat(response, initial_pulse):
    print(f"Raw response: {response}")
    pattern = r'-?\d+'
    nums_str = re.findall(pattern, response)
    try:
        nums = [int(num) for num in nums_str]
        if len(nums) >= 6:
            # 各关节脉冲值
            pulse1, pulse2, pulse3, pulse4, pulse5, pulse6 = nums[:6]
            # 计算各关节角度（deg）
            angle1 = ((pulse1 - initial_pulse['q1']) / 8388608 * 360) / 80.98 + 0
            angle2 = ((pulse2 - initial_pulse['q2']) / 8388608 * 360) / 80.98 - 90
            angle3 = ((pulse3 - initial_pulse['q3']) / 8388608 * 360) / 80.98 + 180
            angle4 = ((pulse4 - initial_pulse['q4']) / 8388608 * 360) / 80.98 + 0
            angle5 = ((pulse5 - initial_pulse['q5']) / 8388608 * 360) / 80.98 + 90
            angle6 = ((pulse6 - initial_pulse['q6']) / 8388608 * 360) / 80.98 + 0
            
            joint_angles = {
                'q1': angle1,
                'q2': angle2,
                'q3': angle3,
                'q4': angle4,
                'q5': angle5,
                'q6': angle6
            }
            print(f"解析后的关节角度（deg）：q1={angle1:.6f}, q2={angle2:.6f}, q3={angle3:.6f}, q4={angle4:.6f}, q5={angle5:.6f}, q6={angle6:.6f}")
            return joint_angles
        else:
            print(f"解析失败：响应数据中仅找到{len(nums)}个整数，需要6个")
            return None
    except ValueError as e:
        print(f"解析失败：数字转换为整数出错 - {e}")
        return None
    except KeyError as e:
        print(f"解析失败：初始脉冲值缺少{e}关节的数据")
        return None
        
def get_joints_angle(server_ip, server_port,command,initial_pluse):
    send_receive_data_ethercat(server_ip, server_port,command)
    response = send_receive_data_ethercat(server_ip, server_port,command)
    if not response:
        print("未收到第二次命令的响应")
        return None
    joints_angle = parse_response_ethercat(response,initial_pluse)
    return joints_angle
    
       
# 使用示例（与之前相同）
if __name__ == '__main__':
    # 配置参数
    SERVER_IP = '10.10.56.214'
    SERVER_PORT = 23333
    CMD = "getJoints"
    # 各关节初始脉冲值
    INITIAL_PULSE = {
        'q1': 12345,
        'q2': 67890,
        'q3': 11111,
        'q4': 22222,
        'q5': 33333,
        'q6': 44444
    }
    # 调用函数获取关节角度
    angles = get_joints_angle(SERVER_IP, SERVER_PORT, CMD, INITIAL_PULSE)
    if angles:
        print(f"最终关节角度：{angles}")
    
    
    
    
