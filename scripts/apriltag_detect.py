import sys
sys.path.append('/home/lys/visp-ros_ws/src/lys_visp_demo/scripts')
import rospy
import cv2
import numpy as np
import apriltag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from dh import a,d,alp,offset,camera_matrix,dist_coeffs
#导入消息类型
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
from lys_visp_demo.msg import AprilTagCorners
#引入齐次变换矩阵消息
from lys_visp_demo.msg import HomogeneousTransform 
from lys_visp_demo.msg import PixelCoordinates 
import tf


def invert_homogeneous_matrix(T):
    #提取旋转部分
    R = T[:3,3]
    p = T[:3,3]

    #计算逆
    R_inv = R.T
    p_inv = -R_inv@p
    #构造逆变换矩阵
    T_inv = np.eye(4)
    T_inv[:3,:3] = R_inv
    T_inv[:3,3] = p_inv

    return T_inv

class VISO:
    def __init__(self):
      #初始化相机内参矩阵和畸变系数
      self.camera_matrix = camera_matrix
      #畸变系数
      self.dist_coeffs = dist_coeffs
      #初始化AprilTag检测器
      options = apriltag.DetectorOptions(families="tag36h11")
      self.detector = apriltag.Detector(options)
      self.measurements = []
      #标签的物理尺寸,单位米
      self.markerLength = 0.051
      #记录四个角点的运动轨迹
      self.corner_trajectories = [[]for _ in range(4)]
      #初始化CvBridge
      self.bridge = CvBridge()
      #初始化tf发布器
      self.br = tf.TransformBroadcaster()
      #ROS发布器,向话题/apriltag_corners发布消息
      self.corners_pub = rospy.Publisher('/apriltag_corners',AprilTagCorners,queue_size=10)
      self.jacobian_pub = rospy.Publisher('/apriltag_jacobian',Float32MultiArray,queue_size=10)
      #发布齐次变换矩阵
      self.homogeneous_pub = rospy.Publisher('/apriltag_homogeneous',HomogeneousTransform,queue_size=10)
      #订阅期望的位置话题/pd_publish
      self.pd_sub = rospy.Subscriber('/pd_publish',Point,self.pd_callback)
      #订阅图像像素坐标话题/img_pixel
      self.pixel_pub = rospy.Subscriber('/image_pixel',PixelCoordinates,self.pixel_callback)
      #期望位置初始化
      self.target_point = None
      #存储期望的像素坐标
      self.pixel_coords = None

    def pd_callback(self,msg):
      """
      处理从 /pd_publish获取期望的位置
      """
      #提取目标位置
      self.target_point = msg
      rospy.loginfo(f"Received target position:{self.target_point}")
    
    def pixel_callback(self,msg):
      """
      处理从/image_pixel获取的图像坐标
      """
      self.pixel_coords = list(msg.pixel)
      rospy.loginfo(f"Received pixel coordinates:({self.pixel_coords}")

    def process_image(self,image):
      """
      处理传入的图像,并检测AprilTag标签
      """
      gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
      #使用AprilTag检测器检测标签
      detections = self.detector.detect(gray)

    
      if self.pixel_coords:
        #定义角点的序号,对应[4,3,2,1]
        corner_order = [4,3,2,1] 
        pixel_position=[]
        #遍历像素坐标,每次取两个元素作为一个角点的(u,v)
        for i in range(0,len(self.pixel_coords),2):
            u = int(self.pixel_coords[i])
            v = int(self.pixel_coords[i+1])
            #计算当前角点的序号,通过corner_order映射
            corner_idx = i//2
            corner_number = corner_order[corner_idx]
            pixel_position.append((u,v))
            if len(pixel_position) == 4:
                for j in range (4):
                    cv2.line(image,pixel_position[j],pixel_position[(j+1)%4],(0,0,255),2)
            #在图像上绘制圆圈标记
            cv2.drawMarker(image,(u,v),(0,0,255),markerType=cv2.MARKER_CROSS,markerSize=10,thickness=2)
            #标注出序号
            cv2.putText(image,f"{corner_number}",(u+10,v-10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2,cv2.LINE_AA)
            cv2.putText(image,f"({u},{v})",(u+10,v+10),cv2.FONT_HERSHEY_SIMPLEX,0.3,(0,255,0),1)
            
      if len(detections) > 0:
        print(f"Detected {len(detections)} AprilTags")
        #处理检测到的每个标签
        for detection in detections:
          #角点信息
          corners = detection.corners
          tvec,rvec = self.estimate_pose(corners)

          rospy.loginfo(f"Apriltag tvec is:{tvec}")
          rospy.loginfo(f"Apriltag rvec is:{rvec}")
          
          corners = np.int32(corners).reshape(-1,2)

          #发布apriltag相对于相机的位姿
          self.publish_tf(tvec,rvec,detection.tag_id)

          #发布齐次变换矩阵
          self.publish_transformation(tvec,rvec,detection.tag_id)
          rospy.loginfo("Publishing apriltag transformation...")


          #发布角点信息
          self.publish_corners(corners)
          #将角点信息通过日志输出
          rospy.loginfo(f"Tag ID:{detection.tag_id},Corners:{corners}")

          #计算同质变换矩阵
          transformation_matrix = self.get_transformation_matrix(tvec,rvec)

          #计算雅可比矩阵
          jacobian_matrix = self.calculate_jacobian(corners,transformation_matrix)
          self.publish_jacobian(jacobian_matrix)
          rospy.loginfo(f"Jacobian Matrix for Tag ID {detection.tag_id}:\n{jacobian_matrix}")
          
          #记录角点位置并绘制运动轨迹
          for i in range(4):
            self.corner_trajectories[i].append(tuple(corners[i]))
            if len(self.corner_trajectories[i]) > i:
              for j in range(1,len(self.corner_trajectories[i])):
                cv2.line(image,self.corner_trajectories[i][j-1],self.corner_trajectories[i][j],(255,0,0),1)

          #绘制角点和边框
          for i in range(4):
            cv2.line(image,tuple(corners[i]),tuple(corners[(i+1)%4]),(0,255,0),2)
            cv2.drawMarker(image,tuple(corners[i]),(0,255,0),markerType=cv2.MARKER_CROSS,markerSize=10,thickness=2)
            #cv2.circle(image,tuple(corners[i]),3,(0,255,255),-1)
            #在每个角点旁边显示序号和像素坐标
            coord_text = f"({corners[i][0]},{corners[i][1]})"
            cv2.putText(image,f"{i+1}",tuple(corners[i]+np.array([5,-5])),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2,cv2.LINE_AA)
            cv2.putText(image,coord_text,tuple(corners[i]+np.array([10,10])),cv2.FONT_HERSHEY_SIMPLEX,0.3,(0,255,0),1)

          
          
      
      #显示结果图像
      cv2.imshow("AprilTag Detection targe",image)
      cv2.waitKey(1)

    def publish_corners(self,corners):
      """
      发布Apriltag的四个角点坐标
      """
      corners_msg =AprilTagCorners ()
      corners_msg.id = 10
      corners_msg.corners = [Point(c[0],c[1],0) for c in corners]
      #发布消息
      self.corners_pub.publish(corners_msg)
    
    def estimate_pose(self,corners):
      """ 
      估计AprilTag的位姿,返回平移向量和旋转向量
      """
      #使用solvePnP计算平移和旋转
      obj_points = np.array([[-self.markerLength/2,-self.markerLength/2,0],#左下角
                             [self.markerLength/2,-self.markerLength/2,0],#右下角
                             [self.markerLength/2,self.markerLength/2,0],#右上角
                             [-self.markerLength/2,self.markerLength/2,0],#左上角
                           ])
      success,rvec,tvec = cv2.solvePnP(obj_points,corners,self.camera_matrix,self.dist_coeffs)
      if not success:
            raise ValueError("Failed to estimate pose")
      return tvec,rvec
    
    def publish_tf(self,tvec,rvec,tag_id):
      """
      发布AprilTag相对于相机的tf变换
      """ 
      #创建TransformStamped消息
      t = TransformStamped()
      t.header.stamp = rospy.Time.now()
      t.header.frame_id = "camera_frame"
      t.child_frame_id =  "apriltag_10"
      
      #设置平移
      t.transform.translation.x = tvec[0]
      t.transform.translation.y = tvec[1]
      t.transform.translation.z = tvec[2]
       
      #将旋转向量转换成旋转矩阵
      rotation_matrix,_ = cv2.Rodrigues(rvec)
      #构建齐次变换矩阵
      homogeneous_matrix = np.eye(4)
      #填入旋转矩阵
      homogeneous_matrix[:3,:3] = rotation_matrix
      #填入平移向量
      homogeneous_matrix[:3,3] = tvec.flatten()
      print(f"T_apriltag is :\n{homogeneous_matrix}")
     
      inverse_matrix = np.linalg.inv(homogeneous_matrix) 
      #打印逆矩阵
      print(f"inverse_matrix is: \n{inverse_matrix}")
        
      inverse_rotation_matrix = inverse_matrix[:3,:3]
      inverse_translation = inverse_matrix[:3,3]
      '''
      t.transform.translation.x = inverse_translation[0]
      t.transform.translation.y = inverse_translation[1]
      t.transform.translation.z = inverse_translation[2]
      '''
       
      # 将旋转矩阵转换为四元数
      '''      
      quaternion = tf.transformations.quaternion_from_matrix(
      np.array([[inverse_rotation_matrix[0][0],inverse_rotation_matrix[0][1],inverse_rotation_matrix[0][2], 0],
                [inverse_rotation_matrix[1][0],inverse_rotation_matrix[1][1],inverse_rotation_matrix[1][2], 0],
                [inverse_rotation_matrix[2][0],inverse_rotation_matrix[2][1],inverse_rotation_matrix[2][2], 0],
                [0, 0, 0, 1]]))     
      '''
      quaternion = tf.transformations.quaternion_from_matrix(
      np.array([[rotation_matrix[0][0],rotation_matrix[0][1],rotation_matrix[0][2], 0],
                [rotation_matrix[1][0],rotation_matrix[1][1],rotation_matrix[1][2], 0],
                [rotation_matrix[2][0],rotation_matrix[2][1],rotation_matrix[2][2], 0],
                [0, 0, 0, 1]]))


      t.transform.rotation.x = quaternion[0]
      t.transform.rotation.y = quaternion[1]
      t.transform.rotation.z = quaternion[2]
      t.transform.rotation.w = quaternion[3]
      #输出tf信息到日志
      #print(f"Tag ID: {tag_id}")
      #print(f"Translation: x={t.transform.translation.x}, y={t.transform.translation.y}, z={t.transform.translation.z}")
      #print(f"Rotation: x={t.transform.rotation.x}, y={t.transform.rotation.y}, z={t.transform.rotation.z}, w={t.transform.rotation.w}")
      
      #发布tf变换
      '''
      self.br.sendTransform((inverse_translation[0],inverse_translation[1],inverse_translation[2]),
                            (quaternion[0],quaternion[1],quaternion[2],quaternion[3]),
                            rospy.Time.now(),"apriltag_10","camera_frame")
      '''
      self.br.sendTransform(( tvec[0],tvec[1],tvec[2]),
                            (quaternion[0],quaternion[1],quaternion[2],quaternion[3]),
                            rospy.Time.now(),"apriltag_10","camera_frame")

    #corners:AprilTag标签的四个角点的像素坐标
    def calculate_jacobian(self,corners,transformation_matrix):
      """
      计算雅可比
      """
      Z = transformation_matrix[2,3]

      #定义obj_points:是AprilTag标签的四个角点在3D世界坐标系中的坐标
      obj_points = np.array([[-self.markerLength/2,-self.markerLength/2,1],
                             [self.markerLength/2,-self.markerLength/2,1],
                             [self.markerLength/2,self.markerLength/2,1],
                             [-self.markerLength/2,self.markerLength/2,1]])

      corners = np.array(corners,dtype=np.float32)

      #初始化雅可比矩阵
      #对于每个角点将两个方向(x和y)方向
      jacobian =  np.zeros((2*len(corners),2*len(corners)))
      
      for i in range(len(corners)):
        x,y = corners[i]
        X,Y,_ = obj_points[i]
        Z = max(Z,1e-3)

        jacobian[2*i,0] = -self.camera_matrix[0,0]/Z
        jacobian[2*i,1] = 0
        jacobian[2*i,2] = (x - self.camera_matrix[0,2])/Z
        jacobian[2*i,3] = -X*(x-self.camera_matrix[0,2])/(Z**2)
        jacobian[2*i,4] = 0
        jacobian[2*i,5] = -X/Z
        
        jacobian[2*i+1,0] = 0
        jacobian[2*i+1,1] = -self.camera_matrix[1,1]/Z
        jacobian[2*i+1,2] = (y-self.camera_matrix[1,2])/Z
        jacobian[2*i+1,3] = 0
        jacobian[2*i+1,4] = -Y*(y-self.camera_matrix[1,2])/(Z**2)
        jacobian[2*i+1,5] = -Y/Z
        
      return jacobian

    def publish_jacobian(self,jacobian_matrix):
        """
        发布雅可比矩阵
        """
        jacobian_msg = Float32MultiArray()
        jacobian_msg.data = jacobian_matrix.flatten().tolist()
        self.jacobian_pub.publish(jacobian_msg) 

    def publish_transformation(self,tvec,rvec,tag_id):
        """
        发布齐次变换矩阵
        """
        #创建HomogeneousTransform消息
        homogeneous_msg = HomogeneousTransform()
        #填充平移向量
        homogeneous_msg.translation = tvec.flatten().tolist() #将平移向量转换为列表
        #计算旋闸矩阵
        rotation_matrix,_ = cv2.Rodrigues(rvec)
        homogeneous_msg.rotation = rotation_matrix.flatten().tolist() #将旋转矩阵转换为以为列表
        #发布消息
        self.homogeneous_pub.publish(homogeneous_msg)
        



       
    def get_transformation_matrix(self,tvec,rvec):
        """
        用平移向量和旋转向量计算齐次变换矩阵
        """
        #将旋转向量转换为旋转矩阵
        R,_ = cv2.Rodrigues(rvec)
        #创建齐次变换矩阵
        transformation_matrix = np.eye(4)
        #旋转部分
        transformation_matrix[:3,:3] = R
        #平移部分
        transformation_matrix[:3,3] = tvec.flatten()
        return transformation_matrix


    def calculate_average_position(self):
      """
      计算检测到的位置信息平均值
      """
      measurements_array = np.array(self.measurements)
      avg_diff = np.mean(measurements_array,axis=0)
      
      movement_x = avg_diff[2][0]  #z方向(相机坐标系)
      movement_y = -avg_diff[0][0] #x方向(相机坐标系)
      movement_z = -avg_diff[1][0] #y方向(相机坐标系)
      
      print(f"Average Movement Position:X:{movement_x},Y:{movement_y},Z:{movement_z}")
      
      #清空测量数据
      self.measurements = []

    def image_callback(self,msg):
      """图像回调函数"""
      try:
        #将ROS图像消息转换为OpenCV图像
        cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        self.process_image(cv_image)
      except Exception as e:
        rospy.logerr(f"Error converting image:{e}")



def main():
  #初始化ROS节点
  rospy.init_node('apriltag_detect_node',anonymous=True)
  viso = VISO()
  #订阅图像话题
  image_sub = rospy.Subscriber('/camera/color/image_raw',Image,viso.image_callback)
  #循环等待ROS消息
  rospy.spin()


      
#测试函数
def test():
  #初始化VISO对象
  viso = VISO()
  #打开摄像头或读取视频
  cap = cv2.VideoCapture(0)
  while True:
    ret,frame = cap.read()
    if not ret:
      break
    #处理每一帧图像
    viso.process_image(frame)

  cap.release()
  cv2.destroyAllWindows()


if __name__ == "__main__":
  main()

 
