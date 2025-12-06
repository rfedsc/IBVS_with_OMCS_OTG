#!/usr/bin/env python3
import socket
import time
# ====================== Socket 连接部分 ======================
def create_socket_client(server_ip, server_port):
    """
    创建并连接 socket
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((server_ip, server_port))
        print("Socket connected.")
        return sock
    except socket.error as e:
        print(f"Connect error: {e}")
        return None

def close_socket(sock):
    """
    关闭 socket
    """
    if sock:
        try:
            sock.close()
            print("Socket closed.")
        except:
            pass

# ====================== 数据发送 / 接收部分 ======================
def send_data(sock, command):
    """
    发送数据
    """
    try:
        sock.sendall(command.encode('utf-8'))
        print(f"send_command: {command}")
        return True
    except socket.error as e:
        print(f"Send error: {e}")
        return False
def receive_data(sock, buf_size=1024):
    """
    接收数据
    """
    try:
        response = sock.recv(buf_size).decode('utf-8')
        return response
    except socket.error as e:
        print(f"Receive error: {e}")
        return None


def parse_response_moveTo(response):
    """
    解析服务器返回
    """
    if response:
        print(f"Raw response: {response}")
        if "e:0" in response:
            print("Command executed successfully.")
        else:
            print("Command failed")
    else:
        print("No response")

# ====================== 主程序 ======================
if __name__ == '__main__':
    server_ip = '10.10.56.214'
    server_port = 23333
    command = "{0.01,0.02,0.03,0,0.02,0}"
    print("Start sending command every 1 second... (Ctrl+C to stop)")
    sock = create_socket_client(server_ip, server_port)
    if sock is None:
        print("Socket connection failed, exit.")
        exit(1)
    try:
        while True:
            # 发送
            ok = send_data(sock, command)
            if not ok:
                break
            # 接收
            response = receive_data(sock)
            parse_response_moveTo(response)
            # 每 1 秒循环
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        close_socket(sock)

