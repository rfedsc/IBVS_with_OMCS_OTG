#!/usr/bin/env/ python3

import rospy
import cv2                        #Opencv图像处理库
from sensor_msgs.msg import Image #图像消息类型
from cv_bridge import CvBridge    #ROS与OpenCV图像转换类
import time

class ImagePublisher:
  def __init__(self):
    #初始化 ROS节点
    rospy.init_node('topic_webcam_pub',anonymous=True)
    
    #创建一个发布者,发布Image消息到话题'image_raw'
    self.publisher = rospy.Publisher('/camera/image_raw',Image,queue_size=10)
    #创建一个CvBridge对象用于图像转换
    self.cv_bridge = CvBridge()
    #打开摄像头
    self.cap = cv2.VideoCapture(0,cv2.CAP_V4L2)
    if not self.cap.isOpened():
      rospy.logerr("无法打开相机")

    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 正确设置宽度
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 正确设置高度

    
    #设置发布频率
    self.rate = rospy.Rate(10) #10Hz

  def start_publishing(self):
    while not rospy.is_shutdown():
      #从摄像头读取一帧图像
      ret,frame = self.cap.read()
      if ret:
        #将OpenCV图像转换为ROS图像消息并发布
        image_message = self.cv_bridge.cv2_to_imgmsg(frame,'bgr8')
        self.publisher.publish(image_message)
        rospy.loginfo("Publishing video frame")

      #休眠以保持10Hz的频率
      self.rate.sleep()

if __name__ == '__main__':
  try:
    image_publisher = ImagePublisher()
    image_publisher.start_publishing()
  except rospy.ROSInterruptException:
    pass

