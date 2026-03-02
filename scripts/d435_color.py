#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
import pyrealsense2 as rs
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

def main():
    # 初始化ROS节点
    rospy.init_node('d435_color_node', anonymous=True)
    
    # 创建发布者
    color_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
    info_pub = rospy.Publisher('/camera/color/camera_info', CameraInfo, queue_size=10)
    
    # 创建CvBridge对象
    bridge = CvBridge()
    
    # 配置RealSense流
    config = rs.config()
    pipeline = rs.pipeline()
    config.enable_stream(rs.stream.color,640 ,480 , rs.format.bgr8, 15)
    
    # 启动流
    pipeline.start(config)
    
    try:
        while not rospy.is_shutdown():
            # 等待帧
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            
            # 检查是否获取到彩色图像帧
            if not color_frame:
                continue
            
            # 获取彩色图像的内参
            color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
            
            # 创建CameraInfo消息
            camera_info_msg = CameraInfo()
            camera_info_msg.header.stamp = rospy.Time.now()
            camera_info_msg.header.frame_id = "camera_color_frame"
            camera_info_msg.height = color_frame.height
            camera_info_msg.width = color_frame.width
            camera_info_msg.distortion_model = "plumb_bob"
            camera_info_msg.K = [color_intrin.fx, 0, color_intrin.ppx,
                                 0, color_intrin.fy, color_intrin.ppy,
                                 0, 0, 1]
            camera_info_msg.P = [color_intrin.fx, 0, color_intrin.ppx, 0,
                                 0, color_intrin.fy, color_intrin.ppy, 0,
                                 0, 0, 1, 0]

            # 将彩色图像转换为Numpy数组
            color_image = np.asanyarray(color_frame.get_data())

            # 将彩色图像转换为ROS消息并发布
            color_msg = bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            color_pub.publish(color_msg)

            # 发布CameraInfo
            info_pub.publish(camera_info_msg)

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

