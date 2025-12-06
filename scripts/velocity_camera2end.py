#!/usr/bin/env python3

import rospy
import numpy as np
from dh import eMc
from geometry_msgs.msg import Twist


def camera_velocity_callback(msg):
    #获取相机的线速度v_camera和角速度w_camera
    v_camera = np.array([msg.linear.x,msg.linear.y,msg.linear.z],dtype=float)
    w_camera = np.array([msg.angular.x,msg.angular.y,msg.angular.z],dtype=float)

    rospy.loginfo(f"Received camera linear velocity:{v_camera}")
    rospy.loginfo(f"Received camera angular velocity:{w_camera}")
    
    #提取旋转矩阵R_eMc和平移向量T_eMc
    R_eMc = eMc[:3,:3]
    p_eMc = eMc[:3,3]
    #线速度转换
    v_end = np.cross(p_eMc,R_eMc@w_camera)+R_eMc@v_camera
    #角速度转换
    w_end = R_eMc@w_camera

    v_end = np.array(v_end,dtype=float)
    w_end = np.array(w_end,dtype=float)

    rospy.loginfo(f"v_end is :{v_end}")
    rospy.loginfo(f"w_end is :{w_end}")
    
    #创建Twist消息发布末端执行器件的速度
    vel_msg = Twist()
    vel_msg.linear.x = float(v_end[0])
    vel_msg.linear.y = float(v_end[1])
    vel_msg.linear.z = float(v_end[2])
    vel_msg.angular.x = float(w_end[0])
    vel_msg.angular.y = float(w_end[1])
    vel_msg.angular.z = float(w_end[2])
   #rospy.loginfo(f"End-effector angular velocity:{w_end}")

    #发布末端执行器的速度
    vel_pub.publish(vel_msg)
    rospy.loginfo("Published end-effector velocity.")
    

def main():
    #初始化ROS节点
    rospy.init_node('velocity_camera2end',anonymous=True)
    #创建发布来末端执行器速度的ROS话题
    global vel_pub
    vel_pub = rospy.Publisher('/end_velocity',Twist,queue_size=10)
    #订阅相机速度的ROS话题
    rospy.Subscriber('/camera_velocity',Twist,camera_velocity_callback)
    #保持节点运行,等待回调
    rospy.spin()
    

if __name__ == '__main__':
    main()







