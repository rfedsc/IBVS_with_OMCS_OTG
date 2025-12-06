#!/usr/bin/env python3
import re
import socket
import time

def send_receive_data_moveTo(server_ip, server_port, command):
    """
    发送数据到服务器并接收响应
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        # 连接到服务器
        sock.connect((server_ip, server_port))
        # 发送消息
        message = command
        sock.sendall(message.encode('utf-8'))
        print(f"send_command: {message}")
        # 接收服务器返回的消息
        response = sock.recv(1024).decode('utf-8')
        return response
    except socket.error as e:
        print(f"Socket error: {e}")
        return None
    finally:
        # 关闭套接字
        sock.close()

def parse_response_moveTo(response):
    if response:
        print(f"Raw response: {response}")
        if "e:0" in response:
            print("Command executed successfully.")
        else:
            print("Command failed")
    else:
        print("No response")


# ======================= 主程序 =======================
if __name__ == '__main__':
    server_ip = '10.10.56.214'
    server_port = 23333
    command = "{0.01,0.02,0.03,0,0.02,0}"
    print("Start sending command every 1 second... (Press Ctrl+C to stop)")
    try:
        while True:
            # 发送并接收数据
            response = send_receive_data_moveTo(server_ip, server_port, command)
            # 解析返回结果
            parse_response_moveTo(response)
            # 每 1 秒循环一次
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopped by user.")

