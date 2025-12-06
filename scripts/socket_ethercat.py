#!/usr/bin/env python3
import re
import socket
def send_receive_data_moveTo_ethercat(server_ip,server_port,command):
    """
    发送数据到服务器并接收响应
    :param server_ip:服务器的IP地址
    :param server_port:服务器的端口号
    :param serial_number:流水号(uint)
    :param command:终端命令
    :return: 服务器的响应
    """
    sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    try:
        #连接到服务器
        sock.connect((server_ip,server_port))
        #格式化发送的消息
        message = command
        sock.sendall(message.encode('utf-8'))
        print(f"send_command:{message}")
        #接收服务器返回的消息
        response = sock.recv(1024).decode('utf-8')
        return response
    except socket.error as e:
        print(f"Socket error:{e}")
        return None
    finally:
        #关闭套接字
        sock.close()

def parse_response_moveTo_ethercat(response):
    if response:
        print(f"Raw response:{response}")
        #这里根据响应格式解析
        if "e:0" in response:
            print("Command executed successfully.")
        else:
            print("Command failed")
    else:
        print(f"No response")


#使用示例
if __name__ == '__main__':
    server_ip = '10.10.56.214'
    server_port = 23333
    #server_ip = '192.168.1.11'
    #server_port = 8080
    #用户输入参数
    #command = "{0.1,0.2,0.3,0,0.2,0}"
    #command = "{-0.1,-0.2,-0.3,0,-0.2,0}"
    command = "{0,0,0,0,0,0}"

    #发送数据并接收响应
    response = send_receive_data_moveTo_ethercat(server_ip,server_port,command)
    #解析收到的响应
    Data = parse_response_moveTo_ethercat(response)

    print(f"Data is:{Data}")

