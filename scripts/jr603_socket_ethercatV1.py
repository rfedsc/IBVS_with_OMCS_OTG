#!/usr/bin/env python3
import rospy
import socket
from std_msgs.msg import Float64MultiArray

SERVER_IP = "10.10.56.214"
SERVER_PORT = 23333

# 全局socket连接
global_socket = None

def send_socket(SERVER_IP, SERVER_PORT, command):
    global global_socket
    # 建立连接（如果尚未连接）
    if global_socket is None:
        try:
            global_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            global_socket.connect((SERVER_IP, SERVER_PORT))
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

def otg_vel_callback(msg):
    if len(msg.data)!=6:
        rospy.logerr("Invalid otg velocity size != 6")
        return
    cmd = "{{{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}}}".format(
            msg.data[0],msg.data[1],msg.data[2],
            msg.data[3],msg.data[4],msg.data[5]
            )
    send_socket(SERVER_IP,SERVER_PORT,cmd)

def main():
    rospy.init_node("socket_ethercat_node",anonymous=True)
    rospy.loginfo("socket_ethercat_node started!")

    #订阅Ruckig平滑后的速度
    rospy.Subscriber("/otg_velocity",Float64MultiArray,otg_vel_callback)

    #ROS循环
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down socket_ethercat_node...")

if __name__ == '__main__':
    main()



