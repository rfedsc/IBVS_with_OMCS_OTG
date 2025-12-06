#!/usr/bin/env python3
import rospy
import socket
from std_msgs.msg import Float64MultiArray

SERVER_IP = "10.10.56.214"
SERVER_PORT = 23333

def send_socket(cmd_str):
    sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    try:
        sock.connect((SERVER_IP,SERVER_PORT))
        sock.sendall(cmd_str.encode("utf-8"))
        print("[Socket] Send:",cmd_str)

        resp = sock.recv(1024).decode("utf-8")
        print("[Socket] Response:",resp)

    except Exception as e:
        print("[Socket Error]",e)

    finally:
        sock.close()

def otg_vel_callback(msg):
    if len(msg.data)!=6:
        rospy.logerr("Invalid otg velocity size != 6")
        return
    cmd = "{{{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}}}".format(
            msg.data[0],msg.data[1],msg.data[2],
            msg.data[3],msg.data[4],msg.data[5]
            )
    send_socket(cmd)

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



