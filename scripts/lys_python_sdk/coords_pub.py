from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT,PI_BAUD
import numpy as np
import rospy
import tf
import math

class MyCobotTFPublisher:
  def __init__(self):
    #初始化ROS节点
    rospy.init_node('mycobot_tf_publisher')
    self.mycobot = MyCobot(PI_PORT,PI_BAUD)
    self.tf_broadcaster = tf.TransformBroadcaster()

  def publish_transfrom(self):
    #获取当前的坐标和姿态
    coords = self.mycobot.get_coords()
    if coords:
      x,y,z = coords[0],coords[1],coords[2]
      rx,ry,rz = coords[3],coords[4],coords[5]
      #将欧拉角转换为四元数
      qx=math.sin(rx/2)*math.cos(ry/2)*math.cos(rz/2)-math.cos(rx/2)*math.sin(ry/2)*math.sin(rz/2)
      qy=math.cos(rx/2)*math.sin(ry/2)*math.cos(rz/2)+math.sin(rx/2)*math.cos(ry/2)*math.sin(rz/2)
      qz=math.cos(rx/2)*math.cos(ry/2)*math.sin(rz/2)-math.sin(rx/2)*math.sin(ry/2)*math.cos(rz/2)
      qw=math.cos(rx/2)*math.cos(ry/2)*math.cos(rz/2)+math.sin(rx/2)*math.sin(ry/2)*math.sin(rz/2)
      #发布tf变换
      self.tf_broadcaster.sendTransform(
        (x,y,z),
        #更新后的四元数
        (qx,qy,qz,qw),
        rospy.Time.now(),
        "camera_frame",
        "world"
      )

if __name__ == "__main__":
  mycobot_tf_publisher = MyCobotTFPublisher()
  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    mycobot_tf_publisher.publish_transfrom()
    #正在发布相机相对于基坐标系的齐次变换矩阵
    rospy.loginfo("正在发布相机相对于基坐标系的齐次变换矩阵")
    rate.sleep()



