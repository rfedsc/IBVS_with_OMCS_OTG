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
    rospy.init_node('realsense_depth_node', anonymous=True)
    # 创建发布者
    depth_pub = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=10)
    info_pub = rospy.Publisher('camera/depth/camera_info',CameraInfo,queue_size=10)
    # 创建CvBridge对象
    bridge = CvBridge()
    # 配置RealSense流
    config = rs.config()
    pipeline = rs.pipeline()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    # 启动流
    pipeline.start(config)
    try:
        while not rospy.is_shutdown():
            # 等待帧
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            # 检查是否获取到帧
            if not depth_frame:
                continue
            #获取相机内参
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
            print(f'depth_intrin is:{depth_intrin}')
            # 创建CameraInfo消息
            camera_info_msg = CameraInfo()
            camera_info_msg.header.stamp = rospy.Time.now()
            camera_info_msg.header.frame_id = "camera_depth_frame"  # 可根据需要修改
            camera_info_msg.height = depth_frame.height
            camera_info_msg.width = depth_frame.width
            camera_info_msg.distortion_model = "plumb_bob"  # 通常使用的失真模型
            camera_info_msg.K = [depth_intrin.fx, 0, depth_intrin.ppx,
                                  0, depth_intrin.fy, depth_intrin.ppy,
                                  0, 0, 1]
            camera_info_msg.P = [depth_intrin.fx, 0, depth_intrin.ppx, 0,
                                  0, depth_intrin.fy, depth_intrin.ppy, 0,
                                  0, 0, 1, 0]
            # 将深度图像转换为Numpy数组
            depth_image = np.asanyarray(depth_frame.get_data())
            # 处理深度信息并显示
            depth_image_display = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # 将深度图像转换为ROS消息并发布
            depth_msg = bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
            depth_pub.publish(depth_msg)
            info_pub.publish(camera_info_msg)
            # 显示深度图像
            cv2.imshow('Depth Image', depth_image_display)
            # 按'q'键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # 停止流并释放资源
        pipeline.stop()
        cv2.destroyAllWindows()
if __name__ == '__main__':
    main()

