#!/usr/bin/env python3
import socket

def send_receive_data_moveTo_ethercat(server_ip, server_port, command):
    """
    发送数据到服务器并接收完整响应（可多条）
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        sock.connect((server_ip, server_port))

        # 发送指令
        message = command
        sock.sendall(message.encode('utf-8'))
        print(f"send_command:{message}")

        # 设置短超时，读取所有返回
        sock.settimeout(0.2)

        all_data = ""
        while True:
            try:
                chunk = sock.recv(1024)
                if not chunk:
                    break
                all_data += chunk.decode('utf-8')
            except socket.timeout:
                # 没有更多数据了，跳出循环
                break

        return all_data

    except socket.error as e:
        print(f"Socket error: {e}")
        return None

    finally:
        sock.close()


def parse_response_moveTo_ethercat(response):
    if response:
        print(f"Raw response:\n{response}")

        # 判断成功标志 e:0
        if "e:0" in response:
            print("Command executed successfully.")
        else:
            print("Command failed")

    else:
        print("No response")


# 使用示例
if __name__ == '__main__':
    server_ip = '10.10.56.214'
    server_port = 23333

    command = "{0,0,0,0,0,0}"

    # 发送数据并接收响应
    response = send_receive_data_moveTo_ethercat(server_ip, server_port, command)

    # 解析收到的响应
    Data = parse_response_moveTo_ethercat(response)

    print(f"Data is:{Data}")

