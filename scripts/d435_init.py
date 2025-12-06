#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
import tf.transformations as tft

tool_rotation = [-np.pi,0,0]

# 全局变量存储深度图像
depth_image = None
depth_intrin = None
#根据深度相机缩放因子设置
cam_depth_scale = 0.001

real_point_pub = None

def depth_callback(msg):
    global depth_image
    bridge = CvBridge()
    # 将ROS消息转换为Numpy数组
    depth_image = bridge.imgmsg_to_cv2(msg,desired_encoding="16UC1")

def camera_info_callback(msg):
    global depth_intrin
    #从CameraInfo消息中获取相机内参
    depth_intrin = rs.intrinsics()
    depth_intrin.width = msg.width
    depth_intrin.height = msg.height
    depth_intrin.fx = 596.40600459
    depth_intrin.fy = 596.56811863
    depth_intrin.ppx = 321.65880473
    depth_intrin.ppy = 266.5707466
    depth_intrin.model = rs.distortion.none  # 根据需要调整畸变模型

def mouse_click(event, x, y, flags, param):
    global depth_image,depth_intrin,real_point_pub
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"点击位置: ({x}, {y})")  # 输出点击位置
        if depth_image is not None:
            print("深度图像已获取")
        else:
            print("深度图像未获取")
        
        if depth_intrin is not None:
            print("相机内参已获取")
        else:
            print("相机内参未获取")

        if depth_image is not None and depth_intrin is not None:
            # 获取点击位置的深度值
            click_z = depth_image[y, x] * cam_depth_scale
            click_x = (x-depth_intrin.ppx)*(click_z/depth_intrin.fx)
            click_y = (y-depth_intrin.ppy)*(click_z/depth_intrin.fy)
            #若click_z为零,则表示改点无效
            if click_z == 0:
                print("深度值无效")
                return
            #将点击点在相机坐标系下的[click_x,click_y,click_z]组成一个3x1的数组
            click_point = np.asarray([click_x,click_y,click_z])
            click_point.shape = (3,1)
            print(f"空间坐标: X: {click_x:.6f}m, Y: {click_y:.6f}m, Z: {click_z:.6f}m")
            
            #将工具的旋转部分(欧拉角)转换为四元数
            quat = tft.quaternion_from_euler(tool_rotation[0],tool_rotation[1],tool_rotation[2])
            if real_point_pub is not None:
                 #发布空间坐标和旋转
                 pose_msg = Pose()
                 pose_msg.position.x = click_x
                 pose_msg.position.y = click_y
                 pose_msg.position.z = click_z
                 pose_msg.orientation.x = quat[0]
                 pose_msg.orientation.y = quat[1]
                 pose_msg.orientation.z = quat[2]
                 pose_msg.orientation.w = quat[3]
                 real_point_pub.publish(pose_msg)
                 # 打印Pose信息到ROS日志
                 rospy.loginfo(f"发布的位置信息:X:{click_x:.6f}m,Y: {click_y:.6f}m,Z:{click_z:.6f}m")
                 rospy.loginfo(f"发布的旋转信息:Quaternion(x:{quat[0]:.6f}, y:{quat[1]:.6f},z:{quat[2]:.6f},w:{quat[3]:.6f})")
def main():
    # 初始化ROS节点
    rospy.init_node('realsense_cam_node', anonymous=True)
    global real_point_pub
    # 创建发布者
    color_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
    real_point_pub = rospy.Publisher('/camera/real_pose',Pose,queue_size=10)

    # 创建订阅者，订阅深度图像
    rospy.Subscriber('/camera/depth/image_raw', Image, depth_callback)
    rospy.Subscriber('/camera/depth/camera_info', CameraInfo, camera_info_callback)

    # 创建CvBridge对象
    bridge = CvBridge()

    # 配置RealSense流
    config = rs.config()
    pipeline = rs.pipeline()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)

    # 创建OpenCV窗口并绑定鼠标点击事件
    cv2.namedWindow('Color Image')
    cv2.setMouseCallback('Color Image', mouse_click)

    try:
        while not rospy.is_shutdown():
            # 等待帧
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            # 检查是否获取到帧
            if not color_frame:
                continue

            # 将图像转换为Numpy数组
            color_image = np.asanyarray(color_frame.get_data())

            # 将图像转换为ROS消息并发布
            color_msg = bridge.cv2_to_imgmsg(color_image,encoding="bgr8")
            color_pub.publish(color_msg)

            # 显示彩色图像
            cv2.imshow('Color Image', color_image)

            # 按'q'键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # 停止流并释放资源
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

