import cv2
import numpy as np
import apriltag

class VISO:
    def __init__(self):
      #初始化相机内参矩阵和畸变系数
      self.camera_matrix = np.array([[1511.0519,0,936.0316],
                                     [0,1508.4997,532.0807],
                                     [0,0,1]])
      #畸变系数
      self.dist_coeffs = np.array([[-0.1032,0.3736,-0.0061,0.0028,-0.5330]])
      #初始化AprilTag检测器
      options = apriltag.DetectorOptions(families="tag36h11")
      self.detector = apriltag.Detector(options)
      self.measurements = []
      #标签的物理尺寸,单位米
      self.markerLength = 0.10
      #记录四个角点的运动轨迹
      self.corner_trajectories = [[]for _ in range(4)]

    def process_image(self,image):
      """
      处理传入的图像,并检测AprilTag标签
      """
      gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
      #使用AprilTag检测器检测标签
      detections = self.detector.detect(gray)

      if len(detections) > 0:
        print(f"Detected {len(detections)} AprilTags")
        #处理检测到的每个标签
        for detection in detections:
          #角点信息
          corners = detection.corners
          tvec,rvec = self.estimate_pose(corners)
          #保存检测到的位置信息
          self.measurements.append(tvec)
          #如果检测到5个标签,计算平均数值
          if len(self.measurements) == 5:
            self.calculate_average_position()
          
          corners = np.int32(corners).reshape(-1,2)
          
          #记录角点位置并绘制运动轨迹
          for i in range(4):
            self.corner_trajectories[i].append(tuple(corners[i]))
            if len(self.corner_trajectories[i]) > i:
              for j in range(1,len(self.corner_trajectories[i])):
                cv2.line(image,self.corner_trajectories[i][j-1],self.corner_trajectories[i][j],(255,0,0),2)

          #绘制角点和边框
          for i in range(4):
            cv2.line(image,tuple(corners[i]),tuple(corners[(i+1)%4]),(0,255,0),2)
            cv2.circle(image,tuple(corners[i]),5,(0,0,255),-1)

      #显示结果图像
      cv2.imshow("AprilTag Detection",image)
      cv2.waitKey(1)

    def estimate_pose(self,corners):
      """
      估计AprilTag的位姿,返回平移向量和旋转向量
      """
      #使用solvePnP计算平移和旋转
      obj_points = np.array([[-self.markerLength/2,-self.markerLength/2,0],
                             [self.markerLength/2,-self.markerLength/2,0],
                             [self.markerLength/2,self.markerLength/2,0],
                             [-self.markerLength/2,self.markerLength/2,0],
                           ])
      _,rvec,tvec = cv2.solvePnP(obj_points,corners,self.camera_matrix,self.dist_coeffs)
      return tvec,rvec

    #corners:AprilTag标签的四个角点的像素坐标
    def calculate_jacobian(self,corners):
      """
      计算雅可比
      """
      #定义obj_points:是AprilTag标签的四个角点在3D世界坐标系中的坐标
      obj_points = np.array([[-self.markerLength/2,-self.markerLength/2,0],
                             [self.markerLength/2,-self.markerLength/2,0],
                             [self.markerLength/2,self.markerLength/2,0],
                             [-self.markerLength/2,self.markerLength/2,0]])

      corners = np.array(corners,dtype=np.float32)

      #初始化雅可比矩阵
      #对于每个角点将两个方向(x和y)方向
      jacobian =  np.zeros((2*len(corners),2*len(corners)))
      
      for i in range(len(corners)):
        x,y = corners[i]
        X,Y,Z = obj_points[i]
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
  test()

 
