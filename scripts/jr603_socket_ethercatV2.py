#!/usr/bin/env python3
import rospy
import socket
from std_msgs.msg import Float64MultiArray

def otg_vel_callback(msg):
    """仅校验速度指令格式，无需发送 Socket，驱动节点会监听/otg_velocity 话题"""
    if len(msg.data)!=6:
        rospy.logerr("Invalid otg velocity size != 6")
        return
    rospy.logdebug(f"已转发OTG速度指令到/otg_velocity话题: {msg.data}")

def main():
    rospy.init_node("socket_ethercat_node",anonymous=True)
    rospy.loginfo("socket_ethercat_node started!（已适配ethercat_socket_driver）")

    # 订阅Ruckig平滑后的速度（仅校验格式，驱动节点会监听同一话题并发送Socket）
    rospy.Subscriber("/otg_velocity",Float64MultiArray,otg_vel_callback)

    # ROS循环
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down socket_ethercat_node...")

if __name__ == '__main__':
    main()




