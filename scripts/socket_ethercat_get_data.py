#!/usr/bin/env python3
import re
import socket

# 全局socket连接
global_socket = None

def send_receive_data_moveTo_ethercat(server_ip, server_port, command):
    """
    发送数据到服务器并接收响应（自动管理连接）
    """
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

def parse_response_moveTo_ethercat(response):
    print(f"Raw response: {response}")

# 使用示例（与之前相同）
if __name__ == '__main__':
    server_ip = '10.10.56.214'
    server_port = 23333
    command = "{0,0,0,0,0,0}"

    # 发送数据并接收响应（只连接一次）
    response = send_receive_data_moveTo_ethercat(server_ip, server_port, command)
    if response:
        parse_response_moveTo_ethercat(response)
    # 可以继续发送更多命令，不会重新连接
    response2 = send_receive_data_moveTo_ethercat(server_ip, server_port, "{0.1,0.2,0.3,0,0.2,0}")
    response = send_receive_data_moveTo_ethercat(server_ip, server_port, command)
    if response:
        parse_response_moveTo_ethercat(response)
    
    
    
    
