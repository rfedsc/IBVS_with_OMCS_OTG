#!/usr/bin/env python3
import socket
import time

def parse_response_moveTo(response):
    if response:
        print(f"Raw response: {response}")
        if "e:0" in response:
            print("Command executed successfully.")
        else:
            print("Command failed")
    else:
        print("No response")

if __name__ == '__main__':
    server_ip = '10.10.56.214'
    server_port = 23333
    command = "{0.01,0.02,0.03,0,0.02,0}"

    print("Connecting to server...")

    try:
        # ✅ 只创建一次 socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((server_ip, server_port))
        print("Connected. Start sending every 1 second... (Ctrl+C to stop)")

        while True:
            # 发送数据
            sock.sendall(command.encode('utf-8'))
            print(f"send_command: {command}")

            # 接收返回数据
            response = sock.recv(1024).decode('utf-8')
            parse_response_moveTo(response)

            # 每 1 秒发送一次
            time.sleep(0.01)

    except socket.error as e:
        print(f"Socket error: {e}")

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        # ✅ 程序退出时关闭 socket
        try:
            sock.close()
            print("Socket closed.")
        except:
            pass

