#!/usr/bin/env python3
import socket
import time

def send_receive_data_moveTo(server_ip, server_port, serial_number, command):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((server_ip, server_port))
        message = f"i:{serial_number},c:{command}@hs@"
        sock.sendall(message.encode('utf-8'))
        print(f"send_command: {message}")
        response = sock.recv(1024).decode('utf-8')
        return response
    except socket.error as e:
        print(f"Socket error: {e}")
        return None
    finally:
        sock.close()

def parse_response_moveTo(response):
    if response:
        print(f"Raw response: {response}")
        if "e:0" in response:
            print("Command executed successfully.")
        else:
            print("Command failed.")
    else:
        print("No response.")

if __name__ == '__main__':
    server_ip = '10.10.56.214'
    server_port = 23333
    serial_number = 1

    # 通用参数
    gpId = 0
    isJoint = 'true'
    ufNum = -1
    utNum = -1
    config = 0
    isLinear = 'false'

    # 三个位姿，注意加了双引号和大括号，确保是服务器期望格式
    positions = [
        '"{85.37,-122.519,196.329,24.107,0,0}"',
        '"{0,-90,180,0,90,0}"',
        '"{-2.326,-58.02,192.865,8.209,53,83.758}"'
    ]

    for i, pos in enumerate(positions):
        command = f"mot.moveTo({gpId},{isJoint},{ufNum},{utNum},{config},{pos},{isLinear})"
        response = send_receive_data_moveTo(server_ip, server_port, serial_number, command)
        parse_response_moveTo(response)
        serial_number += 1

        if i == 0:
            time.sleep(5)
        elif i == 1:
            time.sleep(5)

    print("All commands sent.")

